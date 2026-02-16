# Will work in the Future But broken until newer ROS Version

## Currently Broken/Not functioning as intended
This section describes features and behaviors which do not yet work on ROS Humble, but which will work on newer versions.

### Subloggers / Parser logging
Subloggers do not go to ROS out untill ROS Iron when no corresponding node handle is available. ([ros2/rclcpp#](https://github.com/ros2/rclcpp/pull/1717))

## Will Break
This section describes things that will change when upgrading from ROS Humble.

### QoS objects
In ROS Humble, most `rclcpp::Node::create_*` take a `rmw_qos_profile_t` object instead of a `rclcpp::QoS` object this changes in.
This will be deprecated in ROS Iron, with a warning and removed in ROS Kilted.
To migrate just remove the `.get_rmw_qos_profile()` from the function signatures.
