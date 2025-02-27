import platform

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "machine_namespace",
                default_value=TextSubstitution(
                    text=platform.node().replace("-", "_").lower()
                ),
                description="The namespace containing all Robot specific ROS communication",
            ),
            DeclareLaunchArgument(
                "hardware_namespace",
                default_value="io",
                description="The namespace for the Telemetrix Node and the hardware peripherals",
            ),
        ],
    )

    machine_namespace = LaunchConfiguration("machine_namespace")
    hardware_namespace = LaunchConfiguration("hardware_namespace")
    frame_prefix = ""  # LaunchConfiguration( # No frame prefixes as that does not work with moveit/nav2 and the odom topic must be prefixed instead of the frames.
    #     "_frame_prefix", default=[machine_namespace, "/"]
    # )

    minimal_master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "launch",
                        "minimal_master.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "hardware_namespace": hardware_namespace,
            "frame_prefix": frame_prefix,
        }.items(),
    )

    # Instead of this, we could add a conditional to the launch argument declarations
    # to only launch when the condition is not set. By means of LaunchConfigurationEquals
    ld.add_action(
        GroupAction(
            [minimal_master_launch],
            launch_configurations={
                arg.name: LaunchConfiguration(arg.name)
                for arg in ld.get_launch_arguments()
            },
            forwarding=False,
        )
    )
    return ld
