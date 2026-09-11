# RPi-pMapper (nebula-cam)

Photogrammetry capture pod for Nebula. Pi Zero 2 W + Camera Module 3 (IMX708),
MAVLink to a MicoAir743-AIO over TELEM2, geotagged JPEGs straight out of the
box into Metashape / Pix4D / ODM.

---

## 1. Wiring

| Pi Zero 2 W | MicoAir743 TELEM2 | Note |
|---|---|---|
| GPIO14 (pin 8) TX | RX | |
| GPIO15 (pin 10) RX | TX | |
| GND (pin 6) | GND | Must be common |
| 5V (pin 2/4) | — | **Do not** power from the FC BEC |

Both ends are 3.3 V logic — no level shifter. Give the Pi its own regulator:
a Zero 2 W plus IMX708 peaks around 1.2 A on boot and browns out on a small
FC BEC, and a brownout during a card write is exactly how SD cards die.

Camera Module 3 needs the **22-pin to 15-pin** mini-CSI ribbon for the Zero.
The standard Pi 4 cable will not fit.

Mass budget: Pi Zero 2 W 11 g, IMX708 4 g, ribbon + mount + regulator ~12 g.
Call it 30 g on a sub-250 g airframe.

---

## 2. ArduPilot parameters

```
SERIAL2_PROTOCOL   2        # MAVLink2
SERIAL2_BAUD       921
SERIAL2_OPTIONS    0

CAM1_TYPE          1        # Servo. Emits CAMERA_FEEDBACK on every trigger,
                            # whether or not the Pi is alive.
CAM1_DURATION      10       # 1.0 s pulse
CAM1_TRIGG_DIST    7.7      # metres - see the table below
CAM1_MIN_INTERVAL  1200     # ms floor; protects against overspeed triggering
CAM1_RELAY_ON      1
```

Assign a spare servo output to `RCx_OPTION`/`SERVOx_FUNCTION = 10` (CameraTrigger)
so the trigger is real rather than notional. Nothing needs to be plugged into
it — we only care that the FC generates the event and the feedback message.

`SR2_*` stream rates are set by the Pi at connect via `SET_MESSAGE_INTERVAL`,
so you don't need to touch them. The Pi speaks MAVLink 2 as system 1 /
component 191 (`MAV_COMP_ID_ONBOARD_COMPUTER`) and sends a heartbeat every
`status_period_s`, so it shows up in QGC as a component of the vehicle, not
as a second vehicle.

**Optional, later:** run a wire from a Pi GPIO back to a FC input configured as
`CAM1_FEEDBACK_PIN`. That lets the flight controller stamp the *true* shutter
moment in its own dataflash log, which gives you a second, independent geotag
source for post-processing. Worth doing before you fly anything you'd have to
re-fly.

---

## 3. Mission planning numbers (IMX708 @ 4608 x 2592)

Sensor 6.45 x 3.63 mm, 1.4 µm pixels, f = 4.74 mm.

GSD (cm/px) ≈ altitude (m) × 0.0295

| AGL | GSD | Footprint (w × h) | Trigger dist @80% fwd | Line spacing @70% side |
|---|---|---|---|---|
| 30 m | 0.9 cm | 41 × 23 m | 4.6 m | 12.2 m |
| 50 m | 1.5 cm | 68 × 38 m | 7.7 m | 20.4 m |
| 80 m | 2.4 cm | 109 × 61 m | 12.3 m | 32.7 m |
| 120 m | 3.5 cm | 163 × 92 m | 18.4 m | 49.0 m |

Assumes the long sensor axis is mounted **across track**. Rotate the mount 90°
and the trigger distance and line spacing swap.

Cruise speed is capped by write throughput, not the airframe. At 50 m and
7.7 m spacing, 5 m/s means a frame every 1.5 s. Measure your actual per-frame
write time in the journal before committing to a big block (every frame logs
`NEB_00042.jpg 4.3 MB in 0.87 s`); if it exceeds ~1.2 s, drop to 2304 x 1296
(GSD doubles) or slow down.

**Rolling shutter:** the IMX708 reads out progressively. At 5 m/s the top and
bottom of a frame are captured ~20 ms apart, which is ~10 cm of along-track
skew. Fine for 3 cm GSD corridor work; it is the limiting factor if you ever
want sub-centimetre products, and the reason to move to a global-shutter
sensor when you do.

---

## 4. Latency calibration (do this once)

The service tags each photo using libcamera's `SensorTimestamp`, so most of
the trigger-to-exposure lag is already handled. `extra_latency_ms` corrects
whatever residual remains.

**Bench method:**
1. Put the drone on the bench with GPS lock, props off, and arm it.
2. Display a GPS-disciplined millisecond clock on a phone or laptop screen.
3. Trigger 20 frames manually (`MAV_CMD_DO_DIGICAM_CONTROL` from QGC).
4. Compare the clock visible in each image against the `utc_time` column
   in `geotags.csv`.
5. Median difference, in ms, goes into `extra_latency_ms`.

**Flight method (better, and validates the whole chain):** fly one straight
line over a painted cross or a surveyed target at your normal mission speed,
in both directions. Process both. A systematic along-track offset that flips
sign with direction is uncorrected latency; half the total separation divided
by ground speed is your error in seconds.

At 5 m/s, every 20 ms of uncorrected latency is 10 cm of position error —
comparable to your GSD, so this is worth an afternoon.

---

## 5. Output

Per flight, `/data/flights/flight_YYYYMMDD_HHMMSSZ/`:

- `NEB_00001.jpg` … — EXIF GPS (WGS-84 ellipsoidal altitude), GPS image
  direction, exposure and gain; XMP with camera yaw/pitch/roll in both Pix4D
  and DJI schemas.
- `geotags.csv` — filename, UTC, lat, lon, three altitudes, attitude, fix
  type, satellite count, EPH. Flushed and fsync'd after every row, so a
  power cut costs you at most the frame in flight.

Metashape and Pix4D read the EXIF and XMP directly — import the folder and go.
ODM does too, but feed it the CSV as a GCP/geo file if you want the attitude
priors used.

**Altitude convention:** EXIF `GPSAltitude` is WGS-84 **ellipsoidal**, taken
from `GPS_RAW_INT`, not the EKF's AMSL. Both are in the CSV. Do not mix them
between flights — geoid separation in Ontario is roughly -36 m, and quietly
swapping conventions mid-project shifts a block vertically by that amount.

---

## 6. Why it survives the field

| Failure | Mitigation |
|---|---|
| Power yanked mid-write | Read-only root via overlayfs; `/data` is the only writable volume |
| SD corruption on boot | `fsck.repair=yes`; nothing critical on the writable partition |
| Kernel/board hang | BCM hardware watchdog, 15 s |
| Python hang | systemd `WatchdogSec=20`, fed by the status thread |
| Service crash | `Restart=always`, `StartLimitIntervalSec=0` — it never gives up |
| WiFi not present | No network unit in the boot path; both wait-online units masked |
| No RTC → 1970 timestamps | Clock stepped from MAVLink `SYSTEM_TIME` on first fix, plus `fake-hwclock` |
| Card full | Session refused below 512 MB, shouted over MAVLink |
| Camera dead / cable loose | `STATUSTEXT` heartbeat is absent or reports an error — visible in QGC before takeoff |
| Frames arriving faster than we write | Bounded queues, drop counter reported in the status line |

The pre-flight check is one line in the QGC message panel:

```
CAM READY 0f 1420 left
```

No message, no takeoff.

---

## 7. Steam Deck ground station

The Pi joins the Deck's hotspot as a client and advertises `nebula-cam.local`
over mDNS. It is never a dependency — the camera works with the Deck switched
off.

```bash
# on the Deck, desktop mode
nmcli device wifi hotspot ifname wlan0 ssid NEBULA-GCS password <yours>
```

Two things to plan around:

- The Deck cannot host a hotspot and be on WiFi at the same time. Offload is
  an offline activity, or you bring a USB-C ethernet dongle.
- Zero 2 W WiFi tops out around 2–3 MB/s in practice. A 300-frame flight is
  ~1.4 GB, so budget ~10 minutes. For a survey day, pull the card or move
  `/data` to a USB-OTG stick.

Updating a locked Pi:

```bash
ssh pi@nebula-cam.local
sudo nebula-update    # unlocks, pulls, reboots (may reboot twice)
sudo nebula-lock      # re-enable read-only root - do not forget this
```

---

## 8. Build order

1. Flash Raspberry Pi OS Lite 64-bit (Bookworm), set hostname `nebula-cam`,
   preconfigure the Deck's SSID in Imager.
2. `git clone https://github.com/GamerNationinc/RPi-pMapper.git && cd RPi-pMapper`
   then `sudo ./install.sh`, reboot.
3. Confirm the FC link and a `STATUSTEXT` heartbeat in QGC.
4. Set the ArduPilot params above.
5. Bench-trigger 20 frames, check EXIF, run the latency calibration.
6. Create the `/data` partition, add it to `/etc/fstab`.
7. `sudo nebula-lock`.
8. Power down, image the card as `nebula-cam-v1.img`.

Steps 1–7 happen once, ever. After that a new pod is: flash the image, edit
one TOML file on the boot partition if anything differs, fly.
