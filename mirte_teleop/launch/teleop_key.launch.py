# Due to the different implementation of launchfiles
# (https://github.com/ros2/teleop_twist_keyboard/issues/21)
# one is not able to run teleop_twist_keyboard from
# a launchfile. The xterm workaround is considered to be
# too much of a workaround on a headless machine.
# Solution: Start the node the hard way from a python
# subprocess.
# TODO: Get the lib location in a more generic way


import os
import sys, subprocess
import platform

from launch import LaunchDescription


from ament_index_python import get_package_prefix


# if argv contains this filename, then run the teleop_twist_keyboard node, otherwise just return the launch description
if len(sys.argv) > 0 and sys.argv.count(os.path.basename(__file__)) > 0:
    # FIXME: This differs between the Pioneer and the Master...
    # OPT-TODO: Set it to publish stamped directly
    # Currently parameters (turn/speed) are not implemented in ROS2
    sys.exit(
        subprocess.call(
            [
                os.path.join(
                    get_package_prefix("teleop_twist_keyboard"),
                    "lib",
                    "teleop_twist_keyboard",
                    "teleop_twist_keyboard",
                ),
                "--ros-args",
                "-r",
                f"cmd_vel:=/mirte_base_controller/cmd_vel",
                # "-p",
                # "speed:=0.55",
                # "-p",
                # "turn:=0.05",
            ]
        )
    )
else:
    # print with yellow color
    print("\033[93m" + "Not running teleop_twist_keyboard node as teleop_key.launch.py cannot be included. Run directly instead." + "\033[0m")

def generate_launch_description():
    return LaunchDescription(
        []
           
    )