#!/bin/bash
# set -x

# Hostname
echo "Name: $(cat /etc/hostname)"

# Wi-Fi Line
wifi=$(iwgetid -r)
if [ "$wifi" ]; then
	echo Wi-Fi: $wifi
fi

# just assume that the battery is at /tmp/batteryState, printed by the mirte_master_check script
# way faster than using ros2 topic echo
percentage=$(
	tail -2 /tmp/batteryState | head -1
) || true

if [ "$(echo $percentage | wc -c)" -gt 1 ]; then
	percentage=$(echo "$percentage" | awk '{print $NF}')
	soc=$(echo "$percentage * 100" | bc)
	printf "SOC: %.0f%%\n" "$soc"
fi

# Show only active IP4 addresses
ips=$(ip -4 -o addr show scope global | while read -r line; do dev=$(echo "$line" | awk '{print $2}'); ip link show "$dev" | grep -q "LOWER_UP" && echo "$line"; done | awk '{print $4}' | cut -d/ -f1)
echo "IPs: $ips"

# if not sure about date, then show uptime, otherwise show date
if [ "$(timedatectl | grep "synchronized: yes" | wc -l)" -eq 1 ]; then
	echo "Time: $(date +"%H:%M:%S")"
else
	echo "Uptime: $(uptime | sed 's/^.* up \+\(.\+\), \+[0-9] user.*$/\1/')"
fi
