#!/usr/bin/env bash
# Tries each I2S overlay variant at runtime, keeps the first one that exposes a working "mic4" capture card,
# and makes it permanent in config.txt.   Usage: bash tools/overlay_try.sh
DIR="$(cd "$(dirname "$0")/.." && pwd)"
OV=/boot/firmware/overlays; [ -d "$OV" ] || OV=/boot/overlays
CFG=/boot/firmware/config.txt; [ -f "$CFG" ] || CFG=/boot/config.txt
command -v dtc >/dev/null || sudo apt-get install -y device-tree-compiler
grep -q '^dtparam=i2s=on' "$CFG" || echo 'dtparam=i2s=on' | sudo tee -a "$CFG" >/dev/null
sudo sed -i '/^dtoverlay=i2s-4mic/d' "$CFG"          # avoid double loading
for v in i2s-4mic i2s-4mic-dmic i2s-4mic-spdif; do
  echo; echo "=== variant: $v"
  sudo dtoverlay -r "$v" 2>/dev/null
  if ! dtc -@ -H epapr -Wno-unit_address_vs_reg -I dts -O dtb -o /tmp/$v.dtbo "$DIR/boot/$v.dts" 2>/tmp/dtc.err; then
     echo "dtc compile ERROR:"; cat /tmp/dtc.err; continue; fi
  sudo cp /tmp/$v.dtbo "$OV/"
  if ! sudo dtoverlay -v "$v" 2>&1 | tail -n 6; then :; fi
  sleep 1
  if ! arecord -l | grep -q mic4; then
     echo "-> no 'mic4' card. Kernel messages:"; dmesg | tail -n 25 | grep -i -E "i2s|asoc|sound|mic4|codec|pinctrl|dai|error|fail"; sudo dtoverlay -r "$v" 2>/dev/null; continue; fi
  echo "-> card 'mic4' registered. Testing capture channel counts..."
  for ch in 8 4 2; do
    if arecord -D hw:CARD=mic4,DEV=0 -c $ch -r 16000 -f S32_LE -d 1 -t raw /dev/null 2>/tmp/arec.err; then
      echo "   $ch channels: OK"; GOOD=$ch; break; else echo "   $ch channels: failed ($(head -n1 /tmp/arec.err))"; fi
  done
  if [ -n "$GOOD" ]; then
    echo "dtoverlay=$v" | sudo tee -a "$CFG" >/dev/null
    echo; echo "SUCCESS with $v ($GOOD channels). Saved to $CFG."
    echo "Set in config.json:  \"alsa\": {\"device\": \"hw:CARD=mic4,DEV=0\", \"channels\": $GOOD, ...}"
    echo "Next: python3 tools/hw_check.py -D hw:CARD=mic4,DEV=0 -c $GOOD"
    exit 0
  fi
  sudo dtoverlay -r "$v" 2>/dev/null
done
echo; echo "No variant worked. Send me: 'sudo dtoverlay -v i2s-4mic', 'dmesg | grep -i -E \"i2s|asoc|sound|rp1\"', 'uname -r', 'pinctrl get 18-26'"
exit 1
