# MIRTE Fastdds discovery server
Use this for easier fastdds discovery server setup when using a MIRTE robot.

## Instructions
- On the robot, change the ~/.mirte_settings.sh, change the ```export MIRTE_FASTDDS=false``` line to ```export MIRTE_FASTDDS=true```. This indicates that you're going to use fastdds server and that it should be the main router. Restart the robot or only the ROS system ( ```sudo systemctl restart mirte-ros``` ).
- On your own devices, clone this repo, install rosdep dependencies ( ```rosdep install --from-paths src --ignore-src -y```) and build it.
- Before sourcing the workspace, run ```export MIRTE_FASTDDS=<mirte_ip>``` to let this package know you want to use Zenoh and where to find the main router.
- Source your workspace. During sourcing, you should see a line about using Zenoh.
<!-- - You might be notified of a missing router. You can start it with ```ros2 run rmw_zenoh_cpp rmw_zenohd```. It also needs the same ```export MIRTE_ZENOH=...``` and a sourced workspace. -->
<!-- 
## Multirobot
When using multiple robots, devote one robot/device as your main router. Keep that one on ```MIRTE_ZENOH=true``` and change all others to use the IP of the main device. -->

## Disabling MIRTE-zenoh
- change the MIRTE_ZENOH environment variable back to false on your robot.
- ```unset MIRTE_ZENOH``` on your own devices and stop the Zenoh router.
- After sourcing the workspace, the Zenoh settings are reverted.
When using a different RMW, remove this package, as it might change the ```RMW_IMPLEMENTATION``` & ```ZENOH_ROUTER_CONFIG_URI``` variables.
