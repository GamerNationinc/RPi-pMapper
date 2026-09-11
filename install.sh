#!/usr/bin/env bash
# nebula-cam provisioning - run once on a fresh Raspberry Pi OS Lite (Bookworm,
# 64-bit) install, then image the card. Every subsequent Pi is flash-and-fly.
#
#   sudo ./install.sh
#
# Idempotent: safe to re-run.

set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOOT=/boot/firmware
[[ -d $BOOT ]] || BOOT=/boot
CONFIG=$BOOT/config.txt
CMDLINE=$BOOT/cmdline.txt

say() { echo -e "\n\033[1;32m==>\033[0m $*"; }

add_line() {  # add_line <file> <line>
  grep -qxF "$2" "$1" 2>/dev/null || echo "$2" >> "$1"
}

# ---------------------------------------------------------------- packages
say "installing packages"
apt-get update
# lxml/future via apt so pip never has to compile them on a Zero 2 W.
apt-get install -y --no-install-recommends \
  python3-picamera2 python3-simplejpeg python3-pip python3-numpy python3-pil \
  python3-lxml python3-future python3-tomli \
  fake-hwclock avahi-daemon rsync git iw
pip3 install --break-system-packages pymavlink piexif

# ---------------------------------------------------------------- boot config
say "hardening boot configuration"
add_line "$CONFIG" ""
add_line "$CONFIG" "# --- nebula-cam ---"
add_line "$CONFIG" "camera_auto_detect=1"
# Free the real PL011 UART for MAVLink. mini-UART baud drifts with core clock.
add_line "$CONFIG" "dtoverlay=disable-bt"
add_line "$CONFIG" "enable_uart=1"
# Hardware watchdog: a hung board reboots itself.
add_line "$CONFIG" "dtparam=watchdog=on"
# Shave boot time and current draw.
add_line "$CONFIG" "boot_delay=0"
add_line "$CONFIG" "disable_splash=1"
add_line "$CONFIG" "dtparam=audio=off"

# WiFi power saving makes the Deck link drop; there is no dt overlay for
# this, it is a NetworkManager setting.
mkdir -p /etc/NetworkManager/conf.d
printf '[connection]\nwifi.powersave = 2\n' > /etc/NetworkManager/conf.d/nebula-wifi-powersave.conf

# Serial console would fight us for ttyAMA0.
systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
sed -i 's/console=serial0,[0-9]*[ ]*//' "$CMDLINE"
grep -q "fsck.repair=yes" "$CMDLINE" || sed -i '1 s|$| fsck.repair=yes|' "$CMDLINE"
grep -q "logo.nologo"     "$CMDLINE" || sed -i '1 s|$| logo.nologo|' "$CMDLINE"

# ---------------------------------------------------------------- watchdog
say "enabling systemd watchdogs"
sed -i 's/^#\?RuntimeWatchdogSec=.*/RuntimeWatchdogSec=15/' /etc/systemd/system.conf
sed -i 's/^#\?RebootWatchdogSec=.*/RebootWatchdogSec=60/'   /etc/systemd/system.conf

# ---------------------------------------------------------------- no waiting
say "removing boot-time network blocking"
systemctl mask systemd-networkd-wait-online.service 2>/dev/null || true
systemctl mask NetworkManager-wait-online.service 2>/dev/null || true
systemctl disable --now bluetooth.service hciuart.service 2>/dev/null || true
systemctl disable --now triggerhappy.service 2>/dev/null || true
# No swap: it only accelerates SD wear and can stall the writer thread.
systemctl disable --now dphys-swapfile.service 2>/dev/null || true

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
install -m 755 "$SRC/nebula_cam.py"     /opt/nebula-cam/nebula_cam.py
install -m 644 "$SRC/nebula-cam.service" /etc/systemd/system/nebula-cam.service
install -m 755 "$SRC/nebula-top"          /usr/local/bin/nebula-top
install -m 755 "$SRC/nebula-wifi"         /usr/local/bin/nebula-wifi
install -m 644 "$SRC/nebula-wifi.service" /etc/systemd/system/nebula-wifi.service
[[ -f $BOOT/nebula-cam.toml ]] || install -m 644 "$SRC/nebula-cam.toml" "$BOOT/nebula-cam.toml"

# Keep a git checkout for nebula-update to pull from. If we were run from a
# clone, point at it; otherwise fetch one.
REPO_URL=https://github.com/GamerNationinc/RPi-pMapper.git
if [[ -d $SRC/.git ]]; then
  ln -sfn "$SRC" /opt/nebula-cam-src
elif [[ ! -d /opt/nebula-cam-src/.git ]]; then
  git clone --depth 1 "$REPO_URL" /opt/nebula-cam-src || \
    echo "  (clone failed - offline? nebula-update will not work until it exists)"
fi

# Update helper - needed because the overlay makes a plain git pull evaporate.
cat > /usr/local/bin/nebula-update <<'EOF'
#!/usr/bin/env bash
set -e
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
RO=$(findmnt -no OPTIONS / | grep -c '\bro\b' || true)
if [[ $RO -gt 0 ]]; then
  echo "root is read-only; disabling overlay and rebooting."
  echo "run nebula-update again after the reboot, then run: sudo nebula-lock"
  raspi-config nonint disable_overlayfs
  reboot
fi
cd /opt/nebula-cam-src && git pull
install -m 755 nebula_cam.py /opt/nebula-cam/nebula_cam.py
install -m 644 nebula-cam.service /etc/systemd/system/nebula-cam.service
install -m 755 nebula-top /usr/local/bin/nebula-top
install -m 755 nebula-wifi /usr/local/bin/nebula-wifi
install -m 644 nebula-wifi.service /etc/systemd/system/nebula-wifi.service
systemctl daemon-reload && systemctl restart nebula-cam
echo "updated. remember: sudo nebula-lock"
EOF
chmod 755 /usr/local/bin/nebula-update

cat > /usr/local/bin/nebula-lock <<'EOF'
#!/usr/bin/env bash
set -e
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
raspi-config nonint enable_overlayfs
echo "read-only root enabled. rebooting."
reboot
EOF
chmod 755 /usr/local/bin/nebula-lock

systemctl daemon-reload
systemctl enable nebula-cam.service
systemctl enable nebula-wifi.service

say "done"
cat <<'EOF'

Next steps:
  1. reboot and confirm:  systemctl status nebula-cam
  2. verify the FC link:  nebula-top   (expect S2 READY; S1 NOLINK = no FC heartbeat)
     or from a laptop on the same WiFi:  http://nebula-cam.local:8080/
  3. set the ArduPilot params listed in README.md
  4. run the latency calibration in README.md, write the result into
     /boot/firmware/nebula-cam.toml
  5. create the /data partition, add it to fstab
  6. sudo nebula-lock        <- read-only root, do this LAST
  7. shut down, pull the card, dd it to nebula-cam-vX.img

EOF
