# Due to the different implementation of launchfiles
# (https://github.com/ros2/teleop_twist_keyboard/issues/21)
# one is not able to run teleop_twist_keyboard from
# a launchfile. The xterm workaround is considered to be
# too much of a workaround on a headless machine.
# Solution: Start the node the hard way from a python
# subprocess.
# TODO: Get the lib location in a more generic way

import sys, subprocess
import platform

hostname = platform.node().replace("-", "_").lower()

# FIXME: This differs between the Pioneer and the Master...
# OPT-TODO: Set it to publish stamped directly
# Currently parameters (turn/speed) are not implemented in ROS2
sys.exit(
    subprocess.call(
        [
            "/opt/ros/humble/lib/teleop_twist_keyboard/teleop_twist_keyboard",
            "--ros-args",
            "-r",
            f"cmd_vel:=/mirte_base_controller/cmd_vel_unstamped",
        ]
    )
)

# from launch import LaunchDescription
# from launch_ros.actions import Node
#
#
# def generate_launch_description():
#    ld = LaunchDescription()
#
#    # Need to add prefix: https://github.com/ros2/teleop_twist_keyboard/issues/21
#    node=Node(
#        package = 'teleop_twist_keyboard',
#        executable = 'teleop_twist_keyboard',
#        remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
#        parameters = [
#           { 'speed': 0.55 },
#           { 'turn':  0.05 }
#        ],
#        output = 'screen',
#        prefix='export DISPLAY=:0; xterm -e'
#    )
#
#    ld.add_action(node)
#    return ld
