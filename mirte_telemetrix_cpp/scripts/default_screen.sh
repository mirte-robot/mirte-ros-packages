#!/bin/bash
# set -x
echo "IPs: "$(hostname -I)

# Hostname
echo "Hn:"$(cat /etc/hostname)

# Wi-Fi Line
wifi=$(iwgetid -r)
if [ "$wifi" ]; then
	echo Wi-Fi: $wifi
fi
# echo $MIRTE_USE_MULTIROBOT
# topic="/io/power/power_watcher"
# if [ "$MIRTE_USE_MULTIROBOT" = 'true' ]; then
# 	mirte_space=$(cat /etc/hostname | tr '[:upper:]' '[:lower:]' | tr '-' '_')
# 	topic="/$mirte_space$topic"
# fi
percentage=$(
	tail -2 /tmp/batteryState | head -1
) || true

# State of Charge
if [ "$(echo $percentage | wc -c)" -gt 1 ]; then
		# echo "percentage"
		percentage=$(echo "$percentage" | awk '{print $NF}')

	soc=$(echo "$percentage * 100" | bc)
	printf "SOC: %.0f%%\n" "$soc"

fi

# Time
echo "Time: "$(date +"%H:%M:%S")
