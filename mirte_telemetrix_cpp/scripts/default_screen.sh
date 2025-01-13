#!/bin/bash
echo "IPs: "$(hostname -I)

# Hostname
echo "Hn:"$(cat /etc/hostname)

# Wi-Fi Line
wifi=$(iwgetid -r)
if [ "$wifi" ]; then
	echo Wi-Fi: $wifi
fi

mirte_space=$(cat /etc/hostname | tr '[:upper:]' '[:lower:]' | tr '-' '_')
# State of Charge
if [ $(ros2 topic list | grep "/$mirte_space/io/power/power_watcher$") ]; then
	soc=$(ros2 topic echo /$mirte_space/io/power/power_watcher sensor_msgs/msg/BatteryState --once --field percentage --no-lost-messages --csv --spin-time 0.000001 2>/dev/null)
	soc=$(echo "$soc * 100" | bc)
	printf "SOC: %.0f%%\n" "$soc"

fi

# Time
echo "Time: "$(date +"%H:%M:%S")
