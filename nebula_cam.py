#!/usr/bin/env python3
"""
nebula-cam - autonomous photogrammetry capture service
Raspberry Pi Zero 2 W + IMX708 (Camera Module 3) + ArduPilot over MAVLink.

Design rules:
  * The autopilot owns the trigger (CAM_TRIGG_DIST). We only listen.
  * Position is interpolated at the *sensor exposure* timestamp, not at
    the moment we noticed the trigger.
  * Nothing on the trigger path touches the SD card.
  * Any unhandled state is a restart, not a hang. systemd owns our life.
"""

import os
import sys
import csv
import time
import math
import queue
import socket
import signal
import threading
import subprocess
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # py<3.11
    import tomli as tomllib

# Must be set before pymavlink is imported: SERIAL2_PROTOCOL=2 on the FC.
os.environ.setdefault("MAVLINK20", "1")

import piexif
from pymavlink import mavutil
from picamera2 import Picamera2
from libcamera import controls

try:
    import simplejpeg           # ships with python3-picamera2; ~2x faster than PIL
except ImportError:             # pragma: no cover
    simplejpeg = None

# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------

CONFIG_SEARCH = [
    "/boot/firmware/nebula-cam.toml",
    "/boot/nebula-cam.toml",
    "/etc/nebula-cam.toml",
    str(Path(__file__).with_name("nebula-cam.toml")),
]

DEFAULTS = {
    "link": {
        "device": "/dev/ttyAMA0",
        "baud": 921600,
        "source_system": 1,          # same sysid as the FC: we are a component of it
        "source_component": 191,     # MAV_COMP_ID_ONBOARD_COMPUTER
        "stream_rate_hz": 10,
    },
    "camera": {
        "width": 4608,
        "height": 2592,
        "jpeg_quality": 92,
        "lens_position": 0.0,        # 0.0 = infinity on IMX708
        "max_shutter_us": 1000,      # motion-blur ceiling
        "max_analogue_gain": 8.0,
        "ae_settle_s": 2.0,
        "exposure_midpoint": True,
        "extra_latency_ms": 0.0,     # residual trim after bench calibration
    },
    "storage": {
        "root": "/data/flights",
        "min_free_mb": 512,
    },
    "mission": {
        "fallback_trigger_dist_m": 0.0,   # 0 = disabled; FC is the trigger
        "fallback_after_s": 20.0,
    },
    "system": {
        "set_clock_from_gps": True,
        "status_period_s": 5.0,
    },
}


def deep_merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config():
    cfg = DEFAULTS
    for path in CONFIG_SEARCH:
        if os.path.exists(path):
            try:
                with open(path, "rb") as fh:
                    cfg = deep_merge(cfg, tomllib.load(fh))
                log(f"config: {path}")
                break
            except Exception as exc:
                log(f"config parse failed ({path}): {exc} - using defaults")
    return cfg


# --------------------------------------------------------------------------
# Small utilities
# --------------------------------------------------------------------------

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def boottime():
    """CLOCK_BOOTTIME - same base as libcamera's SensorTimestamp."""
    return time.clock_gettime(time.CLOCK_BOOTTIME)


class SdNotify:
    """Minimal sd_notify so we can feed the systemd watchdog with no deps."""

    def __init__(self):
        self.addr = os.environ.get("NOTIFY_SOCKET")
        self.sock = None
        if self.addr:
            if self.addr.startswith("@"):
                self.addr = "\0" + self.addr[1:]
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send(self, state):
        if not self.sock:
            return
        try:
            self.sock.sendto(state.encode(), self.addr)
        except OSError:
            pass

    def ready(self):
        self.send("READY=1")

    def ping(self):
        self.send("WATCHDOG=1")

    def status(self, text):
        self.send(f"STATUS={text}")


def free_mb(path):
    try:
        st = os.statvfs(path)
        return (st.f_bavail * st.f_frsize) / 1e6
    except OSError:
        return 0.0


def lerp(a, b, t):
    return a + (b - a) * t


def wrap_lerp_deg(a, b, t):
    """Interpolate heading across the 0/360 seam."""
    d = ((b - a + 180.0) % 360.0) - 180.0
    return (a + d * t) % 360.0


# --------------------------------------------------------------------------
# Telemetry ring buffer
# --------------------------------------------------------------------------

class StateBuffer:
    """Rolling window of vehicle state, stamped on CLOCK_BOOTTIME."""

    WINDOW_S = 12.0

    def __init__(self):
        self.lock = threading.Lock()
        self.pos = deque()    # (t, lat, lon, alt_amsl, alt_rel)
        self.gps = deque()    # (t, lat, lon, alt_ellip, fix, sats, eph)
        self.att = deque()    # (t, roll, pitch, yaw)

    def _push(self, dq, item):
        with self.lock:
            dq.append(item)
            cutoff = item[0] - self.WINDOW_S
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def add_pos(self, *a):
        self._push(self.pos, a)

    def add_gps(self, *a):
        self._push(self.gps, a)

    def add_att(self, *a):
        self._push(self.att, a)

    @staticmethod
    def _bracket(dq, t):
        """Return (before, after, frac) around time t, or None."""
        if len(dq) < 2:
            return None
        if t <= dq[0][0]:
            return dq[0], dq[0], 0.0
        if t >= dq[-1][0]:
            return dq[-1], dq[-1], 0.0
        lo, hi = 0, len(dq) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if dq[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = dq[lo], dq[hi]
        span = b[0] - a[0]
        return a, b, 0.0 if span <= 0 else (t - a[0]) / span

    def sample(self, t):
        """Interpolated fix at boottime t. Returns dict or None."""
        with self.lock:
            pos = self._bracket(self.pos, t)
            gps = self._bracket(self.gps, t)
            att = self._bracket(self.att, t)
        if not pos:
            return None

        a, b, f = pos
        out = {
            "lat": lerp(a[1], b[1], f),
            "lon": lerp(a[2], b[2], f),
            "alt_amsl": lerp(a[3], b[3], f),
            "alt_rel": lerp(a[4], b[4], f),
            "alt_ellip": float("nan"),
            "fix": 0, "sats": 0, "eph": float("nan"),
            "roll": float("nan"), "pitch": float("nan"), "yaw": float("nan"),
        }
        if gps:
            a, b, f = gps
            out["alt_ellip"] = lerp(a[3], b[3], f)
            out["fix"], out["sats"], out["eph"] = b[4], b[5], b[6]
            if b[4] >= 3:  # prefer the raw GNSS horizontal solution
                out["lat"] = lerp(a[1], b[1], f)
                out["lon"] = lerp(a[2], b[2], f)
        if att:
            a, b, f = att
            out["roll"] = math.degrees(lerp(a[1], b[1], f))
            out["pitch"] = math.degrees(lerp(a[2], b[2], f))
            out["yaw"] = wrap_lerp_deg(math.degrees(a[3]) % 360.0,
                                       math.degrees(b[3]) % 360.0, f)
        return out


# --------------------------------------------------------------------------
# EXIF / XMP
# --------------------------------------------------------------------------

def _deg_to_dms_rational(value):
    value = abs(value)
    d = int(value)
    m_full = (value - d) * 60
    m = int(m_full)
    s = round((m_full - m) * 60 * 10000)
    return ((d, 1), (m, 1), (s, 10000))


def build_exif(fix, utc_dt, meta):
    lat, lon = fix["lat"], fix["lon"]
    alt = fix["alt_ellip"]
    if not math.isfinite(alt):
        alt = fix["alt_amsl"]

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0),
        piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
        piexif.GPSIFD.GPSLatitude: _deg_to_dms_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
        piexif.GPSIFD.GPSLongitude: _deg_to_dms_rational(lon),
        piexif.GPSIFD.GPSAltitudeRef: 0 if alt >= 0 else 1,
        piexif.GPSIFD.GPSAltitude: (int(round(abs(alt) * 1000)), 1000),
        piexif.GPSIFD.GPSMapDatum: "WGS-84",
    }
    if math.isfinite(fix["yaw"]):
        gps_ifd[piexif.GPSIFD.GPSImgDirectionRef] = "T"
        gps_ifd[piexif.GPSIFD.GPSImgDirection] = (int(round(fix["yaw"] * 100)), 100)
    if utc_dt:
        gps_ifd[piexif.GPSIFD.GPSDateStamp] = utc_dt.strftime("%Y:%m:%d")
        gps_ifd[piexif.GPSIFD.GPSTimeStamp] = (
            (utc_dt.hour, 1), (utc_dt.minute, 1),
            (int(utc_dt.second * 1000 + utc_dt.microsecond / 1000), 1000),
        )

    stamp = (utc_dt or datetime.now(timezone.utc)).strftime("%Y:%m:%d %H:%M:%S")
    exif_ifd = {
        piexif.ExifIFD.DateTimeOriginal: stamp,
        piexif.ExifIFD.DateTimeDigitized: stamp,
        piexif.ExifIFD.FocalLength: (474, 100),        # IMX708, 4.74 mm
        piexif.ExifIFD.FocalLengthIn35mmFilm: 27,
    }
    if meta.get("ExposureTime"):
        exif_ifd[piexif.ExifIFD.ExposureTime] = (1, max(1, int(1e6 / meta["ExposureTime"])))
    if meta.get("AnalogueGain"):
        exif_ifd[piexif.ExifIFD.ISOSpeedRatings] = int(round(meta["AnalogueGain"] * 100))

    zeroth = {
        piexif.ImageIFD.Make: "Raspberry Pi",
        piexif.ImageIFD.Model: "IMX708",
        piexif.ImageIFD.Software: "nebula-cam",
        piexif.ImageIFD.DateTime: stamp,
    }
    return piexif.dump({"0th": zeroth, "Exif": exif_ifd, "GPS": gps_ifd, "1st": {}, "thumbnail": None})


XMP_TEMPLATE = (
    '<?xpacket begin="\ufeff" id="W5M0MpCehiHzreSzNTczkc9d"?>'
    '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
    '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
    '<rdf:Description rdf:about=""'
    ' xmlns:Camera="http://pix4d.com/camera/1.0/"'
    ' xmlns:drone-dji="http://www.dji.com/drone-dji/1.0/"'
    ' Camera:Yaw="{yaw:.2f}" Camera:Pitch="{pitch:.2f}" Camera:Roll="{roll:.2f}"'
    ' drone-dji:GimbalYawDegree="{yaw:.2f}"'
    ' drone-dji:GimbalPitchDegree="{gpitch:.2f}"'
    ' drone-dji:GimbalRollDegree="{roll:.2f}"'
    ' drone-dji:AbsoluteAltitude="{alt:.3f}"'
    ' drone-dji:RelativeAltitude="{relalt:.3f}"/>'
    '</rdf:RDF></x:xmpmeta><?xpacket end="w"?>'
)


def build_xmp(fix, nadir=True):
    yaw = fix["yaw"] if math.isfinite(fix["yaw"]) else 0.0
    pitch = fix["pitch"] if math.isfinite(fix["pitch"]) else 0.0
    roll = fix["roll"] if math.isfinite(fix["roll"]) else 0.0
    alt = fix["alt_ellip"] if math.isfinite(fix["alt_ellip"]) else fix["alt_amsl"]
    return XMP_TEMPLATE.format(
        yaw=yaw, pitch=pitch, roll=roll,
        gpitch=(-90.0 + pitch) if nadir else pitch,
        alt=alt, relalt=fix["alt_rel"],
    ).encode("utf-8")


def insert_xmp(jpeg: bytes, xmp: bytes) -> bytes:
    """Splice an XMP APP1 segment in after the existing APPn block(s)."""
    payload = b"http://ns.adobe.com/xap/1.0/\x00" + xmp
    seg = b"\xff\xe1" + len(payload + b"\x00\x00").to_bytes(2, "big") + payload
    i = 2
    while i + 4 <= len(jpeg) and jpeg[i] == 0xFF and 0xE0 <= jpeg[i + 1] <= 0xEF:
        i += 2 + int.from_bytes(jpeg[i + 2:i + 4], "big")
    return jpeg[:i] + seg + jpeg[i:]


# --------------------------------------------------------------------------
# Flight session (directory + sidecar CSV)
# --------------------------------------------------------------------------

CSV_HEADER = [
    "filename", "utc_time", "latitude", "longitude",
    "alt_ellipsoid_m", "alt_msl_m", "alt_agl_m",
    "yaw_deg", "pitch_deg", "roll_deg",
    "fix_type", "sats", "eph_m", "exposure_us", "gain", "fc_img_idx",
]


class FlightSession:
    def __init__(self, root):
        name = datetime.now(timezone.utc).strftime("flight_%Y%m%d_%H%M%SZ")
        self.dir = Path(root) / name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.count = 0
        self._csv_path = self.dir / "geotags.csv"
        self._fh = open(self._csv_path, "a", newline="")
        self._w = csv.writer(self._fh)
        if self._csv_path.stat().st_size == 0:
            self._w.writerow(CSV_HEADER)
            self._flush()
        log(f"flight session -> {self.dir}")

    def _flush(self):
        self._fh.flush()
        os.fsync(self._fh.fileno())

    def write_row(self, row):
        self._w.writerow(row)
        self._flush()

    def close(self):
        try:
            self._flush()
            self._fh.close()
            fd = os.open(str(self.dir), os.O_DIRECTORY)
            os.fsync(fd)
            os.close(fd)
        except OSError:
            pass
        log(f"flight closed: {self.count} frames in {self.dir.name}")


# --------------------------------------------------------------------------
# Service
# --------------------------------------------------------------------------

# What a frame gets tagged with when the telemetry window has nothing for it.
NO_FIX = {k: float("nan") for k in
          ("lat", "lon", "alt_amsl", "alt_rel", "alt_ellip", "roll", "pitch", "yaw")}
NO_FIX.update({"fix": 0, "sats": 0, "eph": float("nan")})

# One raw 4608x2592 BGR888 frame is 35.8 MB. The Zero 2 W has 512 MB and no
# swap (install.sh removes it), so this queue *is* the memory budget:
# 3 frames ~ 107 MB. Do not raise it without measuring.
WRITE_QUEUE_FRAMES = 3

# Queue sentinel. (CLOSE, session) flows capture_q -> write_q behind every
# frame captured before it, so a session closes only after its last frame.
CLOSE = "CLOSE"


class NebulaCam:
    def __init__(self, cfg):
        self.cfg = cfg
        self.buf = StateBuffer()
        self.notify = SdNotify()
        self.stop = threading.Event()

        self.capture_q = queue.Queue(maxsize=4)
        self.write_q = queue.Queue(maxsize=WRITE_QUEUE_FRAMES)

        # pymavlink does not lock its writer; four threads send on this link.
        self.tx_lock = threading.Lock()

        self.armed = False
        self.session = None            # non-None == recording
        self.session_lock = threading.Lock()
        self.img_idx = 0
        self.dropped = 0
        self.clock_set = False
        self.last_feedback = 0.0
        self.last_fallback_pos = None
        self.gps_epoch_offset = None   # unix - boottime, from SYSTEM_TIME
        self.mav = None
        self.linked = False
        self.picam = None
        self.cam_ready = False

    # -- camera ----------------------------------------------------------
    def camera_start(self):
        cam = self.cfg["camera"]
        self.picam = Picamera2()
        # libcamera names formats little-endian: "BGR888" is R,G,B in memory,
        # which is what every JPEG encoder wants. No channel swap on the Zero.
        config = self.picam.create_still_configuration(
            main={"size": (cam["width"], cam["height"]), "format": "BGR888"},
            buffer_count=2,
            queue=False,
        )
        self.picam.configure(config)
        self.picam.options["quality"] = cam["jpeg_quality"]
        # Only set controls the sensor actually advertises; picamera2 raises
        # on unknown ones and that would put us in a restart loop.
        wanted = {"AwbEnable": True, "AeEnable": True}
        if hasattr(controls, "AfModeEnum") and "AfMode" in self.picam.camera_controls:
            wanted["AfMode"] = controls.AfModeEnum.Manual
            wanted["LensPosition"] = cam["lens_position"]
        self.picam.set_controls(wanted)
        self.picam.start()
        time.sleep(1.0)
        self.cam_ready = True
        log("camera up")

    def lock_exposure(self):
        """Freeze AE/AWB at arm time. Consistent radiometry, capped blur."""
        cam = self.cfg["camera"]
        try:
            self.picam.set_controls({"AeEnable": True, "AwbEnable": True})
            time.sleep(cam["ae_settle_s"])
            meta = self.picam.capture_metadata()
            exp = int(meta.get("ExposureTime", 2000))
            gain = float(meta.get("AnalogueGain", 1.0))
            cap = int(cam["max_shutter_us"])
            if exp > cap:
                gain = min(float(cam["max_analogue_gain"]), gain * (exp / cap))
                exp = cap
            locked = {
                "AeEnable": False,
                "AwbEnable": False,
                "ExposureTime": exp,
                "AnalogueGain": gain,
            }
            if "AfMode" in self.picam.camera_controls:
                locked["AfMode"] = controls.AfModeEnum.Manual
                locked["LensPosition"] = cam["lens_position"]
            cg = meta.get("ColourGains")
            if cg:
                locked["ColourGains"] = cg
            self.picam.set_controls(locked)
            time.sleep(0.3)
            log(f"exposure locked: {exp} us @ gain {gain:.2f}")
            self.statustext(6, f"CAM exp {exp}us gain {gain:.1f}")
        except Exception as exc:
            log(f"exposure lock failed: {exc} (staying on auto)")

    # -- MAVLink ---------------------------------------------------------
    def mav_connect(self):
        """Block until the autopilot's heartbeat is seen. Never gives up, and
        never holds up the rest of the service: the camera is READY and the
        watchdog is fed whether or not the FC is powered."""
        link = self.cfg["link"]
        self.linked = False
        while not self.stop.is_set():
            try:
                if self.mav is None:
                    self.mav = mavutil.mavlink_connection(
                        link["device"], baud=link["baud"],
                        source_system=link["source_system"],
                        source_component=link["source_component"],
                        autoreconnect=True,
                    )
                    log(f"waiting for heartbeat on {link['device']}")
                self.mav.wait_heartbeat(timeout=5)
                if self.mav.target_system:
                    self.linked = True
                    log(f"linked to system {self.mav.target_system}")
                    self.request_streams()
                    return
            except Exception as exc:
                log(f"link error: {exc}")
                try:
                    self.mav.close()
                except Exception:
                    pass
                self.mav = None
                self.stop.wait(2.0)

    def _send(self, name, *args):
        """Every TX goes through here so packets never interleave."""
        mav = self.mav
        if mav is None:
            return
        try:
            with self.tx_lock:
                getattr(mav.mav, name)(*args)
        except Exception:
            pass

    def request_streams(self):
        rate_us = int(1e6 / max(1, self.cfg["link"]["stream_rate_hz"]))
        wanted = [
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, rate_us),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate_us),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200000),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 1000000),
        ]
        for msg_id, interval in wanted:
            self._send("command_long_send",
                       self.mav.target_system, self.mav.target_component,
                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                       msg_id, interval, 0, 0, 0, 0, 0)
            time.sleep(0.05)

    def statustext(self, severity, text):
        self._send("statustext_send", severity, text.encode()[:50])

    def heartbeat(self):
        """Announce ourselves as an onboard computer on the vehicle's sysid.
        ArduPilot only routes to channels it has learned a route on."""
        self._send("heartbeat_send",
                   mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                   mavutil.mavlink.MAV_STATE_ACTIVE)

    def addressed_to_us(self, msg):
        link = self.cfg["link"]
        return (getattr(msg, "target_system", None) == link["source_system"] and
                getattr(msg, "target_component", None) == link["source_component"])

    def mav_loop(self):
        self.mav_connect()
        while not self.stop.is_set():
            try:
                msg = self.mav.recv_match(blocking=True, timeout=1.0)
            except Exception as exc:
                log(f"recv failed: {exc}; reconnecting")
                self.mav_connect()
                continue
            if msg is None:
                continue
            t = boottime()
            kind = msg.get_type()

            if kind == "GLOBAL_POSITION_INT":
                self.buf.add_pos(t, msg.lat / 1e7, msg.lon / 1e7,
                                 msg.alt / 1000.0, msg.relative_alt / 1000.0)
                self.maybe_fallback_trigger(msg)
            elif kind == "GPS_RAW_INT":
                self.buf.add_gps(t, msg.lat / 1e7, msg.lon / 1e7,
                                 msg.alt / 1000.0, msg.fix_type,
                                 msg.satellites_visible,
                                 (msg.eph / 100.0) if msg.eph < 65535 else float("nan"))
            elif kind == "ATTITUDE":
                self.buf.add_att(t, msg.roll, msg.pitch, msg.yaw)
            elif kind == "SYSTEM_TIME":
                self.handle_system_time(msg, t)
            elif kind == "HEARTBEAT":
                self.handle_heartbeat(msg)
            elif kind == "CAMERA_FEEDBACK":
                self.last_feedback = t
                self.enqueue_capture(t, msg.img_idx)
            elif kind == "COMMAND_LONG" and \
                    msg.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
                self.enqueue_capture(t, None)
                if self.addressed_to_us(msg):
                    # QGC's manual trigger waits for this before it stops spinning.
                    self._send("command_ack_send", msg.command,
                               mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def handle_system_time(self, msg, t):
        if msg.time_unix_usec <= 0:
            return
        unix = msg.time_unix_usec / 1e6
        self.gps_epoch_offset = unix - t
        if not self.cfg["system"]["set_clock_from_gps"] or self.clock_set:
            return
        if abs(time.time() - unix) <= 3.0 or os.geteuid() != 0:
            self.clock_set = True      # nothing to do, or nothing we can do
            return
        try:
            time.clock_settime(time.CLOCK_REALTIME, unix)
            subprocess.run(["fake-hwclock", "save"], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            log(f"clock stepped from GPS -> {datetime.now(timezone.utc)}")
            self.clock_set = True
        except Exception as exc:
            log(f"clock step failed: {exc}")   # retried on the next SYSTEM_TIME

    def handle_heartbeat(self, msg):
        # Only the autopilot's own heartbeat carries the armed flag.
        if msg.get_srcSystem() != self.mav.target_system or \
                msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            return
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed == self.armed:
            return
        self.armed = armed
        if armed:
            self.on_arm()
        else:
            self.on_disarm()

    def on_arm(self):
        log("ARMED")
        self.img_idx = 0
        self.dropped = 0
        self.last_fallback_pos = None
        if free_mb(self.cfg["storage"]["root"]) < self.cfg["storage"]["min_free_mb"]:
            self.statustext(2, "CAM DISK FULL - not capturing")
            log("refusing session: disk below threshold")
            return                     # no session -> enqueue_capture ignores triggers
        with self.session_lock:
            if self.session is None:
                self.session = FlightSession(self.cfg["storage"]["root"])
        threading.Thread(target=self.lock_exposure, daemon=True).start()

    def on_disarm(self):
        log("DISARMED")
        self.close_session()
        try:
            self.picam.set_controls({"AeEnable": True, "AwbEnable": True})
        except Exception:
            pass

    def close_session(self, timeout=5.0):
        """Detach the session and send it down the pipeline behind its frames.
        Returns immediately; the writer closes it. Nothing here blocks the
        MAVLink thread for long."""
        with self.session_lock:
            session, self.session = self.session, None
        if session is None:
            return
        try:
            self.capture_q.put((CLOSE, session), timeout=timeout)
        except queue.Full:
            log("capture queue stuck - closing session directly")
            session.close()

    def maybe_fallback_trigger(self, msg):
        """Only used if the FC never sends CAMERA_FEEDBACK."""
        dist = self.cfg["mission"]["fallback_trigger_dist_m"]
        if dist <= 0 or not self.armed:
            return
        if boottime() - self.last_feedback < self.cfg["mission"]["fallback_after_s"]:
            return
        lat, lon = msg.lat / 1e7, msg.lon / 1e7
        if self.last_fallback_pos is None:
            self.last_fallback_pos = (lat, lon)
            return
        dlat = (lat - self.last_fallback_pos[0]) * 111320.0
        dlon = (lon - self.last_fallback_pos[1]) * 111320.0 * math.cos(math.radians(lat))
        if math.hypot(dlat, dlon) >= dist:
            self.last_fallback_pos = (lat, lon)
            self.enqueue_capture(boottime(), None)

    def enqueue_capture(self, t_trigger, fc_idx):
        with self.session_lock:
            session = self.session
        if session is None or not self.cam_ready:
            return
        try:
            self.capture_q.put_nowait((t_trigger, fc_idx, session))
        except queue.Full:
            self.dropped += 1
            log("capture queue full - frame dropped")

    # -- capture / write -------------------------------------------------
    def capture_loop(self):
        # Keep draining after stop so a SIGTERM mid-flight loses nothing queued.
        while not (self.stop.is_set() and self.capture_q.empty()):
            try:
                item = self.capture_q.get(timeout=1.0)
            except queue.Empty:
                continue
            if item[0] is CLOSE:
                self.write_q.put(item)     # ordering matters more than latency here
                continue
            t_trigger, fc_idx, session = item
            try:
                request = self.picam.capture_request()
            except Exception as exc:
                log(f"capture failed: {exc}")
                self.statustext(3, "CAM capture error")
                continue
            try:
                meta = request.get_metadata()
                array = request.make_array("main")
            finally:
                request.release()

            sensor_ts = meta.get("SensorTimestamp")
            if sensor_ts:
                t_expose = sensor_ts / 1e9
                if self.cfg["camera"]["exposure_midpoint"]:
                    t_expose += meta.get("ExposureTime", 0) / 2e6
            else:
                t_expose = t_trigger
            t_expose += self.cfg["camera"]["extra_latency_ms"] / 1000.0

            # Filenames come from our own counter, always. The FC's img_idx
            # is recorded in the CSV for cross-referencing its dataflash log,
            # but mixing the two sources would let os.replace() overwrite.
            self.img_idx += 1
            try:
                self.write_q.put_nowait((array, meta, t_expose, self.img_idx, fc_idx, session))
            except queue.Full:
                self.dropped += 1
                log("write queue full - frame dropped")

    def encode_jpeg(self, array):
        """RGB HxWx3 uint8 -> JPEG bytes. simplejpeg avoids PIL's extra copy."""
        q = int(self.cfg["camera"]["jpeg_quality"])
        if simplejpeg is not None:
            return simplejpeg.encode_jpeg(array, quality=q, colorspace="RGB",
                                          fastdct=True)
        import io
        from PIL import Image
        blob = io.BytesIO()
        Image.fromarray(array).save(blob, format="JPEG", quality=q, optimize=False)
        return blob.getvalue()

    def write_loop(self):
        import io
        while not (self.stop.is_set() and self.write_q.empty()):
            try:
                item = self.write_q.get(timeout=1.0)
            except queue.Empty:
                continue
            if item[0] is CLOSE:
                item[1].close()
                os.sync()
                continue
            array, meta, t_expose, idx, fc_idx, session = item

            fix = self.buf.sample(t_expose)
            if fix is None:
                log("no telemetry for frame - writing without geotag")
                fix = dict(NO_FIX)

            utc_dt = None
            if self.gps_epoch_offset is not None:
                utc_dt = datetime.fromtimestamp(t_expose + self.gps_epoch_offset,
                                                tz=timezone.utc)

            name = f"NEB_{idx:05d}.jpg"
            path = session.dir / name
            try:
                t0 = time.monotonic()
                data = self.encode_jpeg(array)
                if math.isfinite(fix["lat"]):
                    # piexif.insert() writes to a sink; it never returns bytes.
                    sink = io.BytesIO()
                    piexif.insert(build_exif(fix, utc_dt, meta), data, sink)
                    data = insert_xmp(sink.getvalue(), build_xmp(fix))
                tmp = path.with_suffix(".tmp")
                with open(tmp, "wb") as fh:
                    fh.write(data)
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, path)
                session.count += 1
                session.write_row([
                    name,
                    utc_dt.isoformat() if utc_dt else "",
                    f"{fix['lat']:.8f}", f"{fix['lon']:.8f}",
                    f"{fix['alt_ellip']:.3f}", f"{fix['alt_amsl']:.3f}",
                    f"{fix['alt_rel']:.3f}",
                    f"{fix['yaw']:.2f}", f"{fix['pitch']:.2f}", f"{fix['roll']:.2f}",
                    fix["fix"], fix["sats"], f"{fix['eph']:.2f}",
                    meta.get("ExposureTime", ""), f"{meta.get('AnalogueGain', 0):.2f}",
                    "" if fc_idx is None else fc_idx,
                ])
                # The README asks you to read this number before planning a
                # mission: it is the real per-frame write cost on this card.
                log(f"{name} {len(data) / 1e6:.1f} MB in {time.monotonic() - t0:.2f} s")
            except Exception as exc:
                log(f"write failed for {name}: {exc}")
                self.statustext(3, "CAM write error")

    # -- status ----------------------------------------------------------
    def status_loop(self):
        """1 Hz: watchdog + MAVLink heartbeat. Every status_period_s: the one
        line the operator reads in QGC before takeoff."""
        period = self.cfg["system"]["status_period_s"]
        last_status = -period
        last_text = None
        while not self.stop.is_set():
            self.notify.ping()
            self.heartbeat()
            now = time.monotonic()
            if now - last_status >= period:
                last_status = now
                mb = free_mb(self.cfg["storage"]["root"])
                shots = int(mb / 4.5)   # ~4.5 MB per 12 MP frame
                with self.session_lock:
                    session = self.session
                if session:
                    state, n = "REC", session.count
                elif not self.cam_ready:
                    state, n = "INIT", 0
                elif not self.linked:
                    state, n = "NOLINK", 0
                else:
                    state, n = "READY", 0
                text = f"CAM {state} {n}f {shots} left"
                if self.dropped:
                    text += f" DROP{self.dropped}"
                self.notify.status(text)
                sev = 4 if (self.dropped or state not in ("READY", "REC") or mb < 512) else 6
                self.statustext(sev, text)
                if text != last_text:
                    log(text)
                    last_text = text
            self.stop.wait(1.0)

    # -- lifecycle -------------------------------------------------------
    def run(self):
        Path(self.cfg["storage"]["root"]).mkdir(parents=True, exist_ok=True)
        self.camera_start()

        self.workers = [
            threading.Thread(target=self.capture_loop, name="capture", daemon=True),
            threading.Thread(target=self.write_loop, name="write", daemon=True),
        ]
        others = [
            threading.Thread(target=self.mav_loop, name="mavlink", daemon=True),
            threading.Thread(target=self.status_loop, name="status", daemon=True),
        ]
        for th in self.workers + others:
            th.start()
        # READY as soon as the camera is up. The FC link is mav_loop's problem;
        # a Pi that boots before the FC must not be killed for it.
        self.notify.ready()
        while not self.stop.is_set():
            time.sleep(0.5)
        self.shutdown()

    def shutdown(self, drain_s=15.0):
        log("shutting down")
        self.close_session(timeout=2.0)
        # Workers keep draining after stop; give them a bounded window
        # (systemd TimeoutStopSec is 90 s by default).
        deadline = time.monotonic() + drain_s
        for th in getattr(self, "workers", []):
            th.join(max(0.0, deadline - time.monotonic()))
            if th.is_alive():
                log(f"{th.name} still busy at shutdown - queued frames may be lost")
        try:
            self.picam.stop()
        except Exception:
            pass
        os.sync()


def main():
    cfg = load_config()
    svc = NebulaCam(cfg)

    def handler(signum, _frame):
        log(f"signal {signum}")
        svc.stop.set()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)
    svc.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        log(f"FATAL: {exc}")
        time.sleep(2)
        sys.exit(1)   # systemd restarts us
