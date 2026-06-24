from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# works for ps4 controller
# if other controller, maybe change the included launch file to teleop_joy.launch.py
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mirte_teleop",
                executable="mirte_master_arm.py",
                name="mirte_master_arm",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("mirte_teleop"),
                        "launch",
                        "teleop_joy_ps4.launch.py",
                    )
                )
            ),
        ]
    )
