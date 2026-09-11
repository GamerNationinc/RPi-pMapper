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
| WiFi not present | No network unit in the boot path; both wait-online units masked; `nebula-wifi` is a separate oneshot |
| No RTC → 1970 timestamps | Clock stepped from MAVLink `SYSTEM_TIME` on first fix, plus `fake-hwclock` |
| Card full | Session refused below 512 MB, shouted over MAVLink |
| Camera dead / cable loose | `STATUSTEXT` heartbeat is absent or reports an error — visible in QGC before takeoff |
| Frames arriving faster than we write | Bounded queues, drop counter reported in the status line |

The pre-flight check is one line in the QGC message panel:

```
CAM READY 0f 1420 left
```

No message, no takeoff. Section 9 lists every state and flag, and
`nebula-top` shows all of it live.

---

## 7. Ground station WiFi (Steam Deck, laptop, or the Pi's own AP)

The Pi joins whichever known hotspot is in range and advertises
`nebula-cam.local` over mDNS. WiFi is never a dependency — the camera works
with every ground device switched off. All of it is configured in the `[wifi]`
section of `/boot/firmware/nebula-cam.toml` and applied at boot by
`nebula-wifi.service`:

```toml
[wifi]
band = "bg"                       # 2.4 GHz only: range beats throughput
networks = [
  { ssid = "NEBULA-GCS",    psk = "...", priority = 20 },   # Steam Deck
  { ssid = "NEBULA-LAPTOP", psk = "...", priority = 10 },   # laptop
]
fallback_ap = true                # nothing in range after 45 s -> Pi hosts NEBULA-CAM
fallback_ap_psk = "nebulacam"
```

Highest priority in range wins. Edit the file from any SD card reader, no SSH
needed. The passwords are readable by anyone holding the card, so use hotspot
passwords you don't care about.

**Steam Deck** (desktop mode):

```bash
nmcli device wifi hotspot ifname wlan0 ssid NEBULA-GCS password <yours>
```

**Laptop** — turn on the OS hotspot and put its SSID/password in the TOML:

- Windows: *Settings → Network & internet → Mobile hotspot*, set **Band = 2.4 GHz**
  (some laptops default to 5 GHz, which the Pi will not see with `band = "bg"`).
  Clients get `192.168.137.x`. `nebula-cam.local` resolves on Windows 10 1903+;
  if it doesn't, the hotspot panel lists connected devices with their IPs.
- macOS: *System Settings → General → Sharing → Internet Sharing* to Wi-Fi.
- Linux: same `nmcli … hotspot` line as the Deck.

**No hotspot at all:** after `fallback_after_s` the Pi hosts `NEBULA-CAM`
itself at `10.42.0.1`. Join it from anything and open `http://10.42.0.1:8080/`.
The AP profile never autoconnects, so a real network always wins next boot.

Things to plan around:

- Neither the Deck nor a laptop can host a hotspot and be on WiFi at the same
  time. Offload is an offline activity, or you bring a USB ethernet dongle.
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

## 8. WiFi as a telemetry link, and how far it reaches

With `[telemetry] udp_enabled = true` the service bridges the FC's serial
stream to UDP 14550 on the WiFi. QGC on the laptop or Deck autoconnects with
no setup; Mission Planner: *UDP, port 14550, Connect*. Everything the FC
sends on TELEM2 (at the 10 Hz rates the Pi requests) plus the Pi's own
`STATUSTEXT` lines arrive, and anything the GCS sends goes straight to the FC
— parameters, mission upload, mode changes, `DO_DIGICAM_CONTROL`.

Two rules:

1. **This is a second path, not the primary one.** If the drone is flown on a
   real telemetry radio (SiK 915 MHz on TELEM1), keep it. The WiFi link adds a
   dependency on the Pi, which is exactly what section 6 spent its effort
   avoiding for the *camera*; for *control* of the aircraft that dependency
   is not acceptable.
2. WiFi is fine for **telemetry** (a few kB/s, tolerant of packet loss) and
   hopeless for **video**. Do not try.

### What 100 acres looks like

100 acres ≈ 405 000 m² ≈ a 636 m square. Farthest point from the GCS:

| GCS position | Max slant range |
|---|---|
| centre of the block | ~450 m |
| middle of one edge | ~710 m |
| a corner | ~900 m |

Stand in the middle of the long edge if you can. Every metre of ground
antenna height also matters: at 900 m the first Fresnel zone is ~5 m across,
so an antenna at head height over a crop is already half-blocked.

### Ground-side adapter

Keep the airframe side stock. The Zero 2 W's on-board radio (~17 dBm, PCB
antenna) is 0 g extra and the mass budget in section 1 has no room for a USB
adapter (40–60 g). Put all the gain on the ground instead; antenna gain helps
both directions equally.

| Ground setup | Realistic telemetry range (LOS, 2.4 GHz) |
|---|---|
| Laptop / Deck internal WiFi | 150–300 m — covers the block only from the centre, marginally |
| **Alfa AWUS036ACHM** (MT7610U, in-kernel `mt76x0u`, RP-SMA, up to 23 dBm) with its stock 5 dBi omni | 400–600 m |
| Same Alfa + **14 dBi 2.4 GHz panel** on a photo tripod, pointed at the block | 800–1200 m |
| **Ubiquiti NanoStation Loco M2** as the field AP (built-in 8 dBi panel, 23 dBm, outdoor, PoE from a 24 V battery), laptop on ethernet | 1–2 km, most robust; also the only option that leaves the laptop's WiFi free |

The AWUS036ACHM is the pick because it needs no driver on Linux/Deck (kernel
5.x+), Alfa ships a Windows driver, and it has a real antenna connector. The
AWUS036ACM (MT7612U) is the 5 GHz-capable sibling; irrelevant here, 5 GHz is
the wrong band for range.

Link budget for the sceptical, Pi → ground at 900 m: free-space loss at
2.437 GHz is 99 dB. 17 dBm TX − 1 dBi PCB antenna − 99 dB + 14 dBi panel
− 1 dB cable = **−70 dBm** at the Alfa, whose 1 Mbps sensitivity is about
−93 dBm: 23 dB of margin on paper, call it 8–10 dB after airframe shadowing
and fading. Workable for telemetry. With the laptop's own 2 dBi antenna the
same sum is −81 dBm and the margin evaporates in the real world.

### Getting the most out of it

- **Lock 2.4 GHz** (`band = "bg"`, already the default) and 20 MHz channels.
  Pick channel 1, 6 or 11, whichever is quiet where you fly.
- **Windows hotspot cannot be told which adapter to use.** The simplest way to
  get the Alfa's antenna into the link on a Windows laptop is the other way
  round: let the Pi host `NEBULA-CAM` (fallback AP, or set it as the only
  network) and have the Alfa *join* it. Gain is gain regardless of who is
  the AP. On Linux/Deck you can host the hotspot on the Alfa directly:
  `nmcli device wifi hotspot ifname wlx… ssid NEBULA-GCS password …`.
- **Point the panel** at the survey block and raise it: 2–3 m on a light
  mast beats anything else you can do for the money.
- **Airframe:** the Zero 2 W's antenna is the trace at the board edge next
  to the camera connector. Mount that edge outboard, away from carbon fibre,
  the battery and the GPS mast. Carbon is a near-perfect shield.
- **Power saving off** on the Pi (installer does this; `nebula-wifi` sets it
  on every profile it creates) — power save is what makes a link that
  *looks* fine drop packets every few seconds.
- **Watch it live:** `nebula-top` shows RSSI, the UDP peers and byte counts.
  Below about −80 dBm expect loss; below −87 dBm it is gone.
- **Regulatory (Canada, RSS-247):** 2.4 GHz EIRP limit is 36 dBm for
  point-to-multipoint. 20 dBm + 14 dBi = 34 dBm is legal; do not stack a
  1 W amplifier on top of the panel.

If you need telemetry you can bet an aircraft on at 900 m, the answer is not
more WiFi — it is a 915 MHz SiK radio (Holybro SiK 500 mW: ~2 km; RFD900x:
tens of km) on TELEM1, with the WiFi kept for the status page and offload.

---

## 9. Knowing what it is doing: status codes and `nebula-top`

The service publishes one **state** and any number of **flags**, everywhere
at once: the `STATUSTEXT` line in QGC, a JSON snapshot on tmpfs, an HTTP
page, `systemctl status`, and the journal (on change only).

| State | Code | Meaning |
|---|---|---|
| `INIT`   | S0 | camera starting |
| `NOLINK` | S1 | camera up, no FC heartbeat yet — check TELEM2 wiring/params |
| `READY`  | S2 | linked, disarmed. **This is the pre-flight line.** |
| `REC`    | S3 | armed, session open, capturing |

| Flag | Meaning | Action |
|---|---|---|
| `HBLOST`   | FC heartbeat older than 3 s | link fault mid-flight; geotags degrade |
| `NOGPS`    | GPS fix < 3D | normal indoors; **no takeoff** outdoors |
| `NOCLOCK`  | no `SYSTEM_TIME` from FC yet | UTC in EXIF/CSV will be blank until it arrives |
| `DROP`     | frames dropped this session | slow down / drop resolution |
| `DISKLOW`  | below `min_free_mb` | session refused on arm; clear the card |
| `CAMERR`   | a capture failed | ribbon / camera fault |
| `WRITEERR` | a JPEG write failed | card fault; check journal |
| `UNDERVOLT`| board under-voltage (`vcgencmd`) | **fix power now** — this kills cards |
| `THROTTLE` | CPU thermally throttled | shade / airflow; write times will grow |
| `SLOW`     | average write > 1.2 s | mission speed must come down |

In QGC the line reads e.g. `CAM REC 42f 1300 left DROP,SLOW`. Any flag
raises the severity so the message panel colours it.

### `nebula-top`

A btop-style live view. On the Pi it reads `/run/nebula-cam/status.json`;
from a laptop it reads the HTTP endpoint:

```bash
nebula-top                                   # over ssh, on the Pi
nebula-top http://nebula-cam.local:8080      # from the laptop / Deck
nebula-top --once                            # one frame, no curses (scripts)
```

It shows: FC link (heartbeat age, msg/s, fix/sats/EPH, GPS clock), camera
(exposure/gain lock, last capture age, errors), session (frames, dropped,
last/avg write time, MB written), queue fill bars, storage, board (CPU temp,
load, free memory, throttle flags), WiFi (SSID, RSSI, IP, UDP peers) and the
last dozen log lines. Red = act now, yellow = watch, green = fine. `q` quits,
`l` hides the log.

The same data on a phone: `http://nebula-cam.local:8080/` (or
`http://10.42.0.1:8080/` on the fallback AP). Raw JSON at `/status.json`
if you want to script against it.

---

## 10. Build order

1. Flash Raspberry Pi OS Lite 64-bit (Bookworm), set hostname `nebula-cam`,
   preconfigure the Deck's SSID in Imager.
2. `git clone https://github.com/GamerNationinc/RPi-pMapper.git && cd RPi-pMapper`
   then `sudo ./install.sh`, reboot.
3. `nebula-top` shows `S2 READY` (or `S1 NOLINK` if TELEM2 isn't right yet).
4. Set the ArduPilot params above.
5. Bench-trigger 20 frames, check EXIF, run the latency calibration.
6. Create the `/data` partition, add it to `/etc/fstab`.
7. `sudo nebula-lock`.
8. Power down, image the card as `nebula-cam-v1.img`.

Steps 1–7 happen once, ever. After that a new pod is: flash the image, edit
one TOML file on the boot partition if anything differs, fly.
