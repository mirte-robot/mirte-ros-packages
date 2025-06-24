# MIRTE Fastdds discovery server
Use this for easier fastdds discovery server setup when using a MIRTE robot.

## Instructions
- On the robot, change the ~/.mirte_settings.sh, change the ```export MIRTE_FASTDDS=false``` line to ```export MIRTE_FASTDDS=true```. This indicates that you're going to use fastdds server and that it should be the main router. Restart the robot or only the ROS system ( ```sudo systemctl restart mirte-ros``` ).
- On your own devices, clone this repo, install rosdep dependencies ( ```rosdep install --from-paths src --ignore-src -y```) and build it.
- Before sourcing the workspace, run ```export MIRTE_FASTDDS=<mirte_ip>``` to let this package know you want to use the FastDDS discovery server and where to find the server.
- Source your workspace. During sourcing, you should see a line about using FastDDS.

## Disabling
- change the MIRTE_FASTDDS environment variable back to false on your robot.
- ```unset MIRTE_FASTDDS``` on your own devices or ```export MIRTE_FASTDDS=```.
- After sourcing the workspace, the Zenoh settings are reverted.
When using a different RMW, remove this package, as it might change the ```ROS_DISCOVERY_SERVER``` & ```FASTRTPS_DEFAULT_PROFILES_FILE``` variables.
