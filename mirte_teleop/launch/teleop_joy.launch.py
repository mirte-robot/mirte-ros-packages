from launch import LaunchDescription
from launch_ros.actions import Node


# works for ps4 controller
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {"dev": "/dev/input/js0", "deadzone": 0.1, "autorepeat_rate": 20.0}
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_joy_node",
                parameters=[
                    {
                        "axis_linear.x": 1,
                        "axis_angular.yaw": 0,
                        "axis_linear.y": 3,
                        "scale_linear.x": 1.0,
                        "scale_linear.y": 1.0,
                        "scale_angular.yaw": 4.0,
                        # 'scale_angular': 1.0
                    }
                ],
                remappings=[("/cmd_vel", "/mirte_base_controller/cmd_vel")],
            ),
        ]
    )
