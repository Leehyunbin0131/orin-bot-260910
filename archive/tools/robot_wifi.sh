#!/usr/bin/env bash
# Standalone NetworkManager helper for this Orin. No motor commands.
set -euo pipefail
export LC_ALL=C

wifi_device=wlP1p1s0
profile=orinbot-outdoor
profile_file=/etc/NetworkManager/system-connections/orinbot-outdoor.nmconnection
helper=/usr/local/sbin/orinbot-wifi
state_dir=/var/lib/orinbot-wifi
unit=orinbot-wifi-switch

fail() { echo "$*" >&2; exit 1; }
active_uuid() { nmcli -g GENERAL.CON-UUID device show "$wifi_device"; }
profile_uuid() { nmcli -g connection.uuid connection show "$profile"; }

show_info() {
    echo 'Mac Wi-Fi: ORINBOT'
    printf 'Wi-Fi password: '
    nmcli --show-secrets -g 802-11-wireless-security.psk connection show "$profile"
    echo 'SSH: ssh hyunlee@10.42.0.1'
}

restore_indoor() {
    echo 'Restoring the previous indoor Wi-Fi connection.'
    nmcli connection modify "$profile" connection.autoconnect no || true
    nmcli --wait 30 connection up uuid "$(cat "$state_dir/indoor.uuid")" ifname "$wifi_device"
}

start_worker() {
    if systemctl is-active --quiet "$unit.service"; then
        fail 'A Wi-Fi switch is already running; check: journalctl -u orinbot-wifi-switch -n 30'
    fi
    systemctl reset-failed "$unit.service" 2>/dev/null || true
    systemd-run --quiet --collect --unit="$unit" "$helper" "$1"
}

case "${1:-help}" in
    help|--help|-h)
        echo 'sudo bash ~/ros2_ws/tools/robot_wifi.sh setup  # install; keep current Wi-Fi'
        echo 'sudo orinbot-wifi outdoor                    # start hotspot; reconnect from Mac'
        echo 'sudo orinbot-wifi indoor                     # return to saved indoor Wi-Fi'
        echo 'sudo orinbot-wifi info                       # show Wi-Fi password and SSH address'
        exit 0
        ;;
esac

[[ $EUID == 0 ]] || fail 'Run this command with sudo in your SSH terminal.'
for dependency in nmcli systemd-run ss dnsmasq; do
    command -v "$dependency" >/dev/null || fail "Required command missing: $dependency"
done

case "${1:-}" in
    setup)
        [[ $(nmcli -g WIFI-PROPERTIES.AP device show "$wifi_device") == yes ]] \
            || fail 'The Wi-Fi adapter does not report hotspot support.'
        systemctl is-active --quiet ssh || fail 'Start the SSH service before setup.'
        install -d -m 700 "$state_dir"
        if [[ ! -f "$profile_file" ]]; then
            if nmcli -g connection.uuid connection show "$profile" >/dev/null 2>&1; then
                fail 'A different profile already uses the name orinbot-outdoor; inspect it first.'
            fi
            umask 077
            wifi_password=$(od -An -N12 -tx1 /dev/urandom | tr -d ' \n')
            wifi_uuid=$(cat /proc/sys/kernel/random/uuid)
            temp_profile=$(mktemp /etc/NetworkManager/system-connections/.orinbot-XXXXXX)
            trap 'rm -f "$temp_profile"' EXIT
            cat > "$temp_profile" <<EOF
[connection]
id=$profile
uuid=$wifi_uuid
type=wifi
interface-name=$wifi_device
autoconnect=false
autoconnect-priority=100

[wifi]
mode=ap
ssid=ORINBOT
band=bg
channel=6
powersave=2

[wifi-security]
key-mgmt=wpa-psk
proto=rsn;
pairwise=ccmp;
group=ccmp;
psk=$wifi_password

[ipv4]
method=shared
address1=10.42.0.1/24
never-default=true

[ipv6]
method=disabled
EOF
            mv "$temp_profile" "$profile_file"
            trap - EXIT
        fi
        nmcli connection load "$profile_file"
        if [[ $(readlink -f "$0") != "$helper" ]]; then
            install -m 755 "$0" "$helper"
        fi
        echo 'Setup complete. The current Wi-Fi connection has not been switched.'
        show_info
        echo 'Save the password on your Mac, then run: sudo orinbot-wifi outdoor'
        ;;
    outdoor)
        [[ -x "$helper" && -f "$profile_file" ]] || fail 'Run setup first.'
        current_uuid=$(active_uuid)
        if [[ $current_uuid == "$(profile_uuid)" ]]; then
            show_info
            exit 0
        fi
        [[ $current_uuid =~ ^[0-9a-fA-F-]{36}$ ]] || fail 'Connect to indoor Wi-Fi first so rollback is available.'
        # Capture the working connection before the worker can change it.
        install -d -m 700 "$state_dir"
        printf '%s\n' "$current_uuid" > "$state_dir/indoor.uuid"
        show_info
        echo 'Switching in 3 seconds. On your Mac join ORINBOT and SSH to 10.42.0.1.'
        echo 'If no SSH connection arrives within 3 minutes, indoor Wi-Fi will be restored.'
        start_worker __outdoor
        ;;
    __outdoor)
        # systemd keeps this running even when the original SSH connection drops.
        sleep 3
        trap restore_indoor EXIT
        trap 'exit 1' HUP INT TERM
        nmcli --wait 30 connection up "$profile" ifname "$wifi_device"
        wifi_deadline=$((SECONDS + 180))
        while (( SECONDS < wifi_deadline )); do
            if ss -Hnt4 state established '( sport = :22 )' | awk '$3 == "10.42.0.1:22" {found=1} END {exit !found}'; then
                nmcli connection modify "$profile" connection.autoconnect yes
                trap - EXIT HUP INT TERM
                echo 'SSH detected on hotspot. Hotspot mode is now saved for future boots.'
                exit 0
            fi
            sleep 2
        done
        echo 'No SSH connection detected on the hotspot within 3 minutes.' >&2
        exit 1
        ;;
    indoor)
        [[ -s "$state_dir/indoor.uuid" ]] || fail 'No previous indoor Wi-Fi connection has been saved.'
        echo 'Returning to indoor Wi-Fi in 3 seconds. Rejoin indoor Wi-Fi on your Mac.'
        start_worker __indoor
        ;;
    __indoor)
        sleep 3
        if ! restore_indoor; then
            echo 'Indoor Wi-Fi unavailable; returning to the hotspot.' >&2
            nmcli connection modify "$profile" connection.autoconnect yes
            nmcli --wait 30 connection up "$profile" ifname "$wifi_device"
            exit 1
        fi
        ;;
    info)
        show_info
        ;;
    *) fail 'Unknown command. Use: setup, outdoor, indoor, info' ;;
esac
