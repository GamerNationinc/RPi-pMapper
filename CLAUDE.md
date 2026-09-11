# RPi-pMapper (nebula-cam)

Photogrammetry capture pod: Raspberry Pi Zero 2 W + Camera Module 3 (IMX708),
MAVLink to an ArduPilot flight controller (MicoAir743-AIO, TELEM2), writing
geotagged JPEGs + a CSV sidecar for Metashape / Pix4D / ODM. The README is the
operator manual; this file is for working on the code.

## Layout

- `nebula_cam.py` — the whole service, single file, no package. Threads:
  `mav_loop` (telemetry in, arm/disarm, trigger events) → `capture_loop`
  (grab frame + SensorTimestamp) → `write_loop` (encode, EXIF/XMP, CSV) plus
  `status_loop` (1 Hz: watchdog ping, MAVLink heartbeat, JSON snapshot;
  every `status_period_s`: board health + STATUSTEXT). Optional side
  threads: `UdpBridge` (serial↔UDP 14550 for a GCS on WiFi) and a tiny
  HTTP status server. Neither may block or fail the camera.
- `nebula-top` — curses TUI over the status snapshot (file on the Pi, or
  `http://host:8080` from a laptop). `render()` is pure and testable;
  keep display logic there, not in the curses loop.
- `nebula-wifi` + `nebula-wifi.service` — oneshot at boot: turns `[wifi]`
  in the TOML into NetworkManager profiles, optional fallback AP. Not in
  nebula-cam's dependency chain, by design.
- `install.sh` — idempotent Pi provisioning, run once then image the card.
  Also generates `/usr/local/bin/nebula-update` and `nebula-lock`.
- `nebula-cam.service` — `Type=notify`, `Restart=always`, `WatchdogSec=20`.
- `nebula-cam.toml` — config; deployed copy lives at
  `/boot/firmware/nebula-cam.toml` so it is editable from an SD reader.
  Sections: `[link] [camera] [storage] [mission] [system] [telemetry]
  [status]` are read by the service (`DEFAULTS` in `nebula_cam.py` must
  stay in sync); `[wifi]` is read only by `nebula-wifi`.
- `tests/` — `test_geotag.py` and `test_pipeline.py`, hardware stubbed,
  runnable on Windows. See `tests/README.md`. Run both before committing.
- `install.sh` installs: `nebula_cam.py` → `/opt/nebula-cam/`,
  `nebula-top`/`nebula-wifi` → `/usr/local/bin/`, both `.service` files,
  the TOML to the boot partition (first time only). It is the single
  source of truth: `nebula-update` on the Pi does `git pull` then re-runs
  it, so it must stay idempotent and must work **offline** (apt update may
  fail; everything else must already be there).

## Status model

One state (`INIT/NOLINK/READY/REC` = S0–S3) plus flags (`HBLOST NOGPS
NOCLOCK DROP DISKLOW CAMERR WRITEERR UNDERVOLT THROTTLE SLOW`), computed in
`NebulaCam.state()` / `flags()` and published identically to STATUSTEXT,
`/run/nebula-cam/status.json`, the HTTP page and `nebula-top`. Add a new
condition in exactly those two methods and document it in README §9 and
`FLAG_HELP` in `nebula-top`; never invent a second vocabulary.

## Design rules (do not break these)

- The flight controller owns the trigger (`CAM1_TRIGG_DIST`). We only react
  to `CAMERA_FEEDBACK`. The fallback distance trigger is off by default.
- Position is interpolated at libcamera's `SensorTimestamp`
  (CLOCK_BOOTTIME), not at the moment we saw the trigger. Everything in
  `StateBuffer` is stamped with `boottime()`; never mix in `time.time()`.
- Nothing on the trigger path touches the SD card. Capture and write are
  separate bounded queues; a full queue drops and counts, it never blocks.
  `WRITE_QUEUE_FRAMES = 3` is the memory budget (35.8 MB per raw frame,
  512 MB board, no swap) — don't raise it without measuring.
- Session close is an ordered `CLOSE` sentinel through both queues, so every
  captured frame lands before the CSV closes. Don't add sleeps to
  `on_disarm`; the MAVLink thread must never block.
- All MAVLink TX goes through `_send()` under `tx_lock`, which also mirrors
  to the UDP bridge. Never call `mav.mav.*_send` directly.
- The status snapshot goes to `/run` (tmpfs) — never write status to the
  card.
- Root filesystem is read-only in the field (overlayfs). Only `/data` is
  writable. Don't add anything that writes elsewhere at runtime.
- No network dependency anywhere in the boot or capture path. WiFi carries
  the UDP telemetry bridge, the status page and offload — all optional,
  all must degrade to nothing when WiFi is absent. The FC's own radio is
  the primary control link; the bridge is a second path, never the only one.
- EXIF `GPSAltitude` is WGS-84 ellipsoidal (from `GPS_RAW_INT`). AMSL and
  AGL go in the CSV/XMP. Don't swap conventions.
- Unhandled errors should exit; systemd restarts us. Don't add retry loops
  that can hang silently.

## Hardware / platform facts

- MAVLink 2 (`MAVLINK20=1` is set before pymavlink import). We are
  sysid 1 / compid 191 (`MAV_COMP_ID_ONBOARD_COMPUTER`), a component of
  the vehicle, not a second vehicle.
- UART is the PL011 at `/dev/ttyAMA0` (Bluetooth disabled). 921600 baud.
- picamera2 raises on any control libcamera doesn't advertise — check
  `picam.camera_controls` before setting anything new.
- Frame format is `BGR888`, which libcamera names little-endian: it is
  R,G,B in memory. Encode with `simplejpeg` (`colorspace="RGB"`), PIL is
  the fallback. Do not reintroduce a channel flip.
- Target OS: Raspberry Pi OS Lite 64-bit Bookworm, Python 3.11
  (`tomllib` built in; `tomli` fallback only for older).
- Zero 2 W is slow: ~1 s per 12 MP JPEG write. Anything added to
  `write_loop` costs mission cruise speed.

## Developing on Windows

- The repo is edited on Windows and run on the Pi. `.gitattributes` forces
  LF; if you generate files with Python `write_text`, strip CRs before
  committing (`sed -i 's/\r$//'`).
- Hardware modules (`picamera2`, `libcamera`, `pymavlink`, `simplejpeg`)
  aren't installable here; `tests/` stubs them in `sys.modules` and shims
  the Linux-only bits (`CLOCK_BOOTTIME`, `os.sync`, `statvfs`,
  `O_DIRECTORY`). Needs `piexif pillow numpy`. Before committing:

      python -m py_compile nebula_cam.py nebula-top nebula-wifi
      bash -n install.sh
      python tests/test_geotag.py && python tests/test_pipeline.py

- Bash heredocs in this environment mangle backslashes (`\U` in Windows
  paths breaks Python). Write patch scripts with the Write tool and run
  them, or use `sed` by line number.
- Nothing here can exercise capture, the serial port, NetworkManager or
  curses on a real terminal. State clearly what was tested locally vs.
  what needs the Pi.

## Deploying

    git clone https://github.com/GamerNationinc/RPi-pMapper.git && cd RPi-pMapper
    sudo ./install.sh && sudo reboot
    nebula-top                       # expect S2 READY; S1 NOLINK = no FC heartbeat
    journalctl -u nebula-cam -f      # "camera up", "linked to system 1"

From a laptop on the same WiFi: `http://nebula-cam.local:8080/` or
`nebula-top http://nebula-cam.local:8080`. QGC autoconnects on UDP 14550.

Updating a locked Pi: `sudo nebula-update` (may reboot twice), then
`sudo nebula-lock`. Forgetting the lock leaves root writable in the field.
