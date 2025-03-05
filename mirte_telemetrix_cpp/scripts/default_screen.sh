#!/bin/bash
# set -x
echo "IPs: $(hostname -I)"

# Hostname
echo "Hn:$(cat /etc/hostname)"

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

# Time
echo "Time: $(date +"%H:%M:%S")"
