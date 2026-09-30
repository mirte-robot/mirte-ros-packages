#!/bin/bash
# set -x

# if /home/mirte/.mirte_settings.sh exist, source it
if [ -f /home/mirte/.mirte_settings.sh ]; then
	source /home/mirte/.mirte_settings.sh
fi

counter=$1

# Hostname
echo "Name: $(cat /etc/hostname)"

# counter mod 2 pages
counter=$((counter % 2))

if [ "$counter" -eq 0 ]; then
	# if ros_domain_ID is set, show it
	if [ -n "$ROS_DOMAIN_ID" ]; then
		echo "ROS Domain ID: $ROS_DOMAIN_ID"
	fi
elif [ "$counter" -eq 1 ]; then

	# Wi-Fi Line
	wifi=$(iwgetid -r)
	if [ "$wifi" ]; then
		echo Wi-Fi: $wifi
	fi
	# show cpu percentage
	# get cpu load from uptime, better than top, as there are multiple implementations.
	cpu=$(uptime | awk -F 'load average:' '{ print $2 }' | cut -d, -f1 | awk '{print $1 * 100}')
	# get core count
	cores=$(nproc)
	# divide by core count
	cpu=$(echo "$cpu / $cores" | bc)
	echo "CPU: $cpu%"
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

# Show time if sure about time (and always show uptime
if [ "$(timedatectl | grep "synchronized: yes" | wc -l)" -eq 1 ]; then
	echo "Time: $(date +"%H:%M:%S")"
fi
echo "Uptime: $(uptime | sed 's/^.* up \+\(.\+\), \+[0-9] user.*$/\1/')"

# Show only active IP4 addresses (as last, since there might be more)
ips=$(ip -4 -o addr show scope global | while read -r line; do
	dev=$(echo "$line" | awk '{print $2}')
	ip link show "$dev" | grep -q "LOWER_UP" && echo "$line"
done | awk '{print $4}' | cut -d/ -f1)
echo "IPs: $ips"
