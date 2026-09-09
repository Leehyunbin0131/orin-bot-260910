#!/usr/bin/env bash
# Grant plugdev access to this D435if model; no network or service restart.
set -euo pipefail

rules_file=/etc/udev/rules.d/99-orinbot-realsense-rsusb.rules
rule='SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", GROUP="plugdev", MODE="0660"'
if [[ ${1:-} == --help ]]; then
    echo 'sudo bash ~/ros2_ws/tools/setup_rsusb_usb_access.sh'
    echo "Installs $rules_file and refreshes permissions for USB 8086:0b3a."
    exit 0
fi
[[ $# == 0 ]] || { echo 'Unexpected argument; use --help.' >&2; exit 2; }
[[ $EUID == 0 ]] || {
    echo 'Run: sudo bash ~/ros2_ws/tools/setup_rsusb_usb_access.sh' >&2
    exit 1
}
getent group plugdev >/dev/null
if [[ -e "$rules_file" && $(cat "$rules_file") != "$rule" ]]; then
    echo "A different rule already exists at $rules_file; inspect it before replacing." >&2
    exit 1
fi
umask 022
printf '%s\n' "$rule" > "$rules_file"
chmod 644 "$rules_file"
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=usb \
    --attr-match=idVendor=8086 --attr-match=idProduct=0b3a
udevadm settle --timeout=10
echo 'D435if USB access configured for members of plugdev.'
echo 'Next: bash ~/ros2_ws/tools/with_rsusb.sh rs-enumerate-devices -s'
