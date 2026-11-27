#!/usr/bin/env bash
# Pair and configure an Xbox Series X|S (USB-C / Model 1914 BLE) controller
# directly on Raspberry Pi Zero W (Raspberry Pi OS Lite, Kernel 6.18).
set -euo pipefail

echo "=== HUE-DINI: Xbox Series X|S Bluetooth & evdev Setup ==="

echo "[1/4] Installing BlueZ and Python evdev dependencies..."
sudo apt-get update
sudo apt-get install -y bluetooth bluez python3-evdev

echo "[2/4] Adding user '$USER' to 'input' group (passwordless /dev/input/event* access)..."
sudo usermod -aG input "$USER"

echo "[3/4] Configuring /etc/bluetooth/main.conf for BLE HOGP auto-reconnect..."
BT_CONF="/etc/bluetooth/main.conf"
if [ -f "$BT_CONF" ]; then
  sudo cp "$BT_CONF" "${BT_CONF}.bak.$(date +%s)"
fi
sudo python3 - <<'PY'
from pathlib import Path
p = Path("/etc/bluetooth/main.conf")
text = p.read_text(encoding="utf-8") if p.exists() else "[General]\n"
updates = {
    "FastConnectable": "true",
    "Privacy": "device",
    "JustWorksRepairing": "always",
}
lines = text.splitlines()
out = []
in_general = False
seen = set()
for line in lines:
    s = line.strip()
    if s.startswith("[") and s.endswith("]"):
        if in_general:
            for k, v in updates.items():
                if k not in seen:
                    out.append(f"{k} = {v}")
        in_general = (s.lower() == "[general]")
        out.append(line)
        continue
    if in_general and "=" in s and not s.startswith("#"):
        key = s.split("=", 1)[0].strip()
        if key in updates:
            out.append(f"{key} = {updates[key]}")
            seen.add(key)
            continue
    out.append(line)
if in_general:
    for k, v in updates.items():
        if k not in seen:
            out.append(f"{k} = {v}")
p.write_text("\n".join(out) + "\n", encoding="utf-8")
PY

sudo systemctl restart bluetooth

cat <<'EOF'

[4/4] Interactive Pairing Instructions:
  1. Put your Xbox Series X|S controller into pairing mode
     (turn it on with the Xbox button, then hold the top Pair button for 3s until the Xbox logo blinks fast).
  2. Run:
       bluetoothctl
     Inside bluetoothctl, run:
       power on
       agent on
       default-agent
       scan on
     Wait until you see "Xbox Wireless Controller" and its MAC address (XX:XX:XX:XX:XX:XX), then run:
       pair XX:XX:XX:XX:XX:XX
       trust XX:XX:XX:XX:XX:XX
       connect XX:XX:XX:XX:XX:XX
       scan off
       quit

  3. Log out and log back in (or reboot) once so the 'input' group membership takes effect.
  4. Wired USB-C fallback: plugging the controller into a spare USB-A port on the
     MakerSpot USB Hub HAT works immediately with the exact same Pi controller code.
EOF
