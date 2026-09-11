#!/usr/bin/env bash
# nebula-cam provisioning - run once on a fresh Raspberry Pi OS Lite (Bookworm,
# 64-bit) install, then image the card. Every subsequent Pi is flash-and-fly.
#
#   sudo ./install.sh
#
# Idempotent: safe to re-run, and nebula-update re-runs it after every git
# pull, so it must also work offline (packages already installed).

set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOOT=/boot/firmware
[[ -d $BOOT ]] || BOOT=/boot
CONFIG=$BOOT/config.txt
CMDLINE=$BOOT/cmdline.txt
REPO_URL=https://github.com/GamerNationinc/RPi-pMapper.git

say() { echo -e "\n\033[1;32m==>\033[0m $*"; }

add_line() {  # add_line <file> <line>   (exact-match, so re-runs don't duplicate)
  grep -qxF -- "$2" "$1" 2>/dev/null || echo "$2" >> "$1"
}

# ---------------------------------------------------------------- packages
say "installing packages"
# Offline re-runs are normal (nebula-update in the field): a failed update
# is fine as long as everything below is already installed.
apt-get update || echo "  apt-get update failed - offline? continuing with what's installed"
# lxml/future via apt so pip never has to compile them on a Zero 2 W.
apt-get install -y --no-install-recommends \
  python3-picamera2 python3-simplejpeg python3-pip python3-numpy python3-pil \
  python3-lxml python3-future python3-tomli \
  fake-hwclock avahi-daemon network-manager rsync git iw
# mavnative is a C extension pymavlink tries to build; Lite has no compiler
# and we don't need it. The pure-Python parser is plenty at 10 Hz.
DISABLE_MAVNATIVE=True pip3 install --break-system-packages pymavlink piexif

# ---------------------------------------------------------------- boot config
say "hardening boot configuration"
if ! grep -q "^# --- nebula-cam ---" "$CONFIG"; then
  printf '\n# --- nebula-cam ---\n' >> "$CONFIG"
fi
add_line "$CONFIG" "camera_auto_detect=1"
# Free the real PL011 UART for MAVLink. mini-UART baud drifts with core clock.
add_line "$CONFIG" "dtoverlay=disable-bt"
add_line "$CONFIG" "enable_uart=1"
# Hardware watchdog: a hung board reboots itself.
add_line "$CONFIG" "dtparam=watchdog=on"
# Shave boot time and current draw.
add_line "$CONFIG" "boot_delay=0"
add_line "$CONFIG" "disable_splash=1"
# Bookworm ships dtparam=audio=on; edit it rather than appending a second one.
if grep -q "^dtparam=audio=on" "$CONFIG"; then
  sed -i 's/^dtparam=audio=on/dtparam=audio=off/' "$CONFIG"
else
  add_line "$CONFIG" "dtparam=audio=off"
fi

# Serial console would fight us for the UART. raspi-config knows every
# spelling (serial0 / ttyS0 / ttyAMA0); the sed is belt-and-braces.
raspi-config nonint do_serial_cons 1 2>/dev/null || true   # 1 = console OFF
raspi-config nonint do_serial_hw 0   2>/dev/null || true   # 0 = hardware ON
systemctl disable --now serial-getty@ttyAMA0.service serial-getty@ttyS0.service \
  serial-getty@serial0.service 2>/dev/null || true
sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//' "$CMDLINE"
grep -q "fsck.repair=yes" "$CMDLINE" || sed -i '1 s|$| fsck.repair=yes|' "$CMDLINE"
grep -q "logo.nologo"     "$CMDLINE" || sed -i '1 s|$| logo.nologo|' "$CMDLINE"

# ---------------------------------------------------------------- watchdog
say "enabling systemd watchdogs"
sed -i 's/^#\?RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf
sed -i 's/^#\?RebootWatchdogSec=.*/RebootWatchdogSec=60/'   /etc/systemd/system.conf

# ---------------------------------------------------------------- sd card wear
say "keeping the card quiet"
# Journal in RAM, capped. Nothing in the journal is worth a card write.
mkdir -p /etc/systemd/journald.conf.d
printf '[Journal]\nStorage=volatile\nRuntimeMaxUse=32M\n' > /etc/systemd/journald.conf.d/nebula.conf
# No swap: it only accelerates SD wear and can stall the writer thread.
systemctl disable --now dphys-swapfile.service 2>/dev/null || true
[[ -f /var/swap ]] && { dphys-swapfile uninstall 2>/dev/null || rm -f /var/swap; }
# Background jobs that write to root and eat the Zero's CPU at boot.
systemctl disable --now apt-daily.timer apt-daily-upgrade.timer man-db.timer 2>/dev/null || true

# ---------------------------------------------------------------- no waiting
say "removing boot-time network blocking"
systemctl mask systemd-networkd-wait-online.service 2>/dev/null || true
systemctl mask NetworkManager-wait-online.service 2>/dev/null || true
systemctl disable --now bluetooth.service hciuart.service 2>/dev/null || true
systemctl disable --now triggerhappy.service 2>/dev/null || true

# WiFi power saving makes the ground link drop packets every few seconds.
# nebula-wifi also sets it per profile; this catches Imager's profile.
mkdir -p /etc/NetworkManager/conf.d
printf '[connection]\nwifi.powersave = 2\n' > /etc/NetworkManager/conf.d/nebula-wifi-powersave.conf
systemctl reload NetworkManager 2>/dev/null || true

# ---------------------------------------------------------------- data volume
say "preparing /data"
if ! grep -q " /data " /etc/fstab; then
  cat <<'EOF'

  ------------------------------------------------------------------
  NOTE: create a dedicated partition for photos before enabling the
  read-only root, then add it to /etc/fstab, e.g.

    /dev/mmcblk0p3  /data  ext4  defaults,noatime,nofail  0  2

  Falling back to a directory on the root filesystem for now, which
  will NOT survive the overlay being enabled.
  ------------------------------------------------------------------
EOF
fi
mkdir -p /data/flights
chmod 755 /data /data/flights

# ---------------------------------------------------------------- the service
say "installing nebula-cam"
mkdir -p /opt/nebula-cam
install -m 755 "$SRC/nebula_cam.py"       /opt/nebula-cam/nebula_cam.py
install -m 644 "$SRC/nebula-cam.service"  /etc/systemd/system/nebula-cam.service
install -m 755 "$SRC/nebula-top"          /usr/local/bin/nebula-top
install -m 755 "$SRC/nebula-wifi"         /usr/local/bin/nebula-wifi
install -m 644 "$SRC/nebula-wifi.service" /etc/systemd/system/nebula-wifi.service
# The config is the operator's; never overwrite an existing one.
[[ -f $BOOT/nebula-cam.toml ]] || install -m 644 "$SRC/nebula-cam.toml" "$BOOT/nebula-cam.toml"

# Keep a git checkout for nebula-update to pull from. If we were run from a
# clone, point at it; otherwise fetch one. root pulling a user-owned checkout
# needs safe.directory or git refuses ("dubious ownership").
if [[ -d $SRC/.git ]]; then
  ln -sfn "$SRC" /opt/nebula-cam-src
elif [[ ! -d /opt/nebula-cam-src/.git ]]; then
  git clone --depth 1 "$REPO_URL" /opt/nebula-cam-src || \
    echo "  (clone failed - offline? nebula-update will not work until it exists)"
fi
for d in /opt/nebula-cam-src "$(readlink -f /opt/nebula-cam-src 2>/dev/null || true)"; do
  [[ -n $d ]] && git config --system --add safe.directory "$d" 2>/dev/null || true
done

# Update helper. The overlay makes a plain git pull evaporate, so it unlocks
# first; then it re-runs this script, which is the single source of truth for
# what gets installed where.
cat > /usr/local/bin/nebula-update <<'EOF'
#!/usr/bin/env bash
set -e
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
if findmnt -no FSTYPE / | grep -q overlay; then
  echo "root is read-only; disabling overlay and rebooting."
  echo "run nebula-update again after the reboot, then: sudo nebula-lock"
  raspi-config nonint disable_overlayfs
  reboot
fi
cd /opt/nebula-cam-src
git pull --ff-only
bash ./install.sh
systemctl restart nebula-wifi.service nebula-cam.service
echo
echo "updated to $(git rev-parse --short HEAD). remember: sudo nebula-lock"
EOF
chmod 755 /usr/local/bin/nebula-update

cat > /usr/local/bin/nebula-lock <<'EOF'
#!/usr/bin/env bash
set -e
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
if ! grep -q " /data " /etc/fstab; then
  echo "WARNING: /data is not a separate partition in /etc/fstab."
  echo "         With the overlay on, photos will be written to RAM and lost."
  read -r -p "lock anyway? [y/N] " a; [[ $a == y ]] || exit 1
fi
raspi-config nonint enable_overlayfs
echo "read-only root enabled. rebooting."
reboot
EOF
chmod 755 /usr/local/bin/nebula-lock

systemctl daemon-reload
systemctl enable nebula-cam.service nebula-wifi.service

say "done"
cat <<'EOF'

Next steps:
  1. reboot and confirm:  systemctl status nebula-cam
  2. verify the FC link:  nebula-top   (expect S2 READY; S1 NOLINK = no FC heartbeat)
     or from a laptop on the same WiFi:  http://nebula-cam.local:8080/
  3. edit /boot/firmware/nebula-cam.toml: hotspot SSIDs/passwords in [wifi]
  4. set the ArduPilot params listed in README.md
  5. run the latency calibration in README.md, write the result into the TOML
  6. create the /data partition, add it to fstab
  7. sudo nebula-lock        <- read-only root, do this LAST
  8. shut down, pull the card, dd it to nebula-cam-vX.img

EOF
