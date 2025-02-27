# Copyright 2023 ros2_control Development Team
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description="An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.",
        )
    ]

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mirte_base_control"),
                    "urdf",
                    "base.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content,
        "frame_prefix": LaunchConfiguration("frame_prefix"),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mirte_base_control"),
            "config",
            "mirte_base_control.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(robot_controllers, allow_substs=True)],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            ("~/tf_odometry", "/tf"),
        ],
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
        arguments=[
            "pid_wheels_controller",
            "mirte_base_controller",
        ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    # # Delay start of joint_state_broadcaster after `robot_controller`
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        namespace="mirte_base_controller",
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
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        twist_stamper,
    ]

    return LaunchDescription(declared_arguments + nodes)
