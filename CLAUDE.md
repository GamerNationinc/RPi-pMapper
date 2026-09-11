# RPi-pMapper (nebula-cam)

Photogrammetry capture pod: Raspberry Pi Zero 2 W + Camera Module 3 (IMX708),
MAVLink to an ArduPilot flight controller (MicoAir743-AIO, TELEM2), writing
geotagged JPEGs + a CSV sidecar for Metashape / Pix4D / ODM. The README is the
operator manual; this file is for working on the code.

## Layout

- `nebula_cam.py` — the whole service, single file, no package. Threads:
  `mav_loop` (telemetry in, arm/disarm, trigger events) → `capture_loop`
  (grab frame + SensorTimestamp) → `write_loop` (encode, EXIF/XMP, CSV) plus
  `status_loop` (systemd watchdog ping, MAVLink heartbeat + STATUSTEXT).
- `install.sh` — idempotent Pi provisioning, run once then image the card.
  Also generates `/usr/local/bin/nebula-update` and `nebula-lock`.
- `nebula-cam.service` — `Type=notify`, `Restart=always`, `WatchdogSec=20`.
- `nebula-cam.toml` — config; deployed copy lives at
  `/boot/firmware/nebula-cam.toml` so it is editable from an SD reader.
  `DEFAULTS` in `nebula_cam.py` must stay in sync with it.

## Design rules (do not break these)

- The flight controller owns the trigger (`CAM1_TRIGG_DIST`). We only react
  to `CAMERA_FEEDBACK`. The fallback distance trigger is off by default.
- Position is interpolated at libcamera's `SensorTimestamp`
  (CLOCK_BOOTTIME), not at the moment we saw the trigger. Everything in
  `StateBuffer` is stamped with `boottime()`; never mix in `time.time()`.
- Nothing on the trigger path touches the SD card. Capture and write are
  separate bounded queues; a full queue drops and counts, it never blocks.
- Root filesystem is read-only in the field (overlayfs). Only `/data` is
  writable. Don't add anything that writes elsewhere at runtime.
- No network dependency anywhere in the boot or capture path. WiFi is an
  offload convenience only.
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
  aren't installable here. To test the pure-Python parts (`StateBuffer`,
  `build_exif`, `build_xmp`, `insert_xmp`), stub those modules in
  `sys.modules` before `import nebula_cam`; `piexif` and `pillow` are all
  that's needed. Always run `python -m py_compile nebula_cam.py` and
  `bash -n install.sh` before committing.
- Nothing here can exercise capture or the serial link. State clearly what
  was tested locally vs. what needs the Pi.

## Deploying

    git clone https://github.com/GamerNationinc/RPi-pMapper.git && cd RPi-pMapper
    sudo ./install.sh && sudo reboot
    journalctl -u nebula-cam -f      # expect "camera up", "linked to system 1"

Updating a locked Pi: `sudo nebula-update` (may reboot twice), then
`sudo nebula-lock`. Forgetting the lock leaves root writable in the field.
