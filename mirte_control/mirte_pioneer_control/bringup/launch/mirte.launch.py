# Based on:
#
# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    frame_prefix_launch_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value="",
        description="An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.",
    )

    # TODO: Add possible namespacing
    # TODO: Use prefixes? for urdf Yes?

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mirte_control"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": robot_description_content,
        "frame_prefix": LaunchConfiguration("frame_prefix"),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mirte_control"),
            "config",
            "mirte_diff_drive_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[("~/robot_description", "robot_description")],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[robot_controller_spawner],
    #         )
    #     )
    # )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        namespace="diff_drive_controller",
        remappings=[
            ("cmd_vel_out", "cmd_vel"),
            ("cmd_vel_in", "cmd_vel_unstamped"),
        ],
        parameters=[
            {
                "frame_id": (
                    LaunchConfiguration("frame_prefix"),
                    TextSubstitution(text="base_link"),
                )
            }
        ],
    )

    nodes = [
        frame_prefix_launch_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        twist_stamper,
    ]

    return LaunchDescription(nodes)
