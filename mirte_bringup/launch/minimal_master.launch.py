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
                default_value="",
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
    start_controller_manager = LaunchConfiguration(
        "_start_controller_manager", default="false"
    )
    start_state_publishers = LaunchConfiguration(
        "_start_state_publishers", default="false"
    )

    telemetrix = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_telemetrix_cpp"),
                        "launch",
                        "telemetrix.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config_path": PathJoinSubstitution(
                [
                    FindPackageShare("mirte_bringup"),
                    "telemetrix_config",
                    "mirte_master_config.yaml",
                ]
            ),
            "hardware_namespace": hardware_namespace,
            "frame_prefix": frame_prefix,
        }.items(),
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "launch",
                        "ros2_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"frame_prefix": frame_prefix}.items(),
    )

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "launch",
                        "state_publishers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"frame_prefix": frame_prefix}.items(),
    )

    mecanum_drive_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_base_control"),
                        "launch",
                        "mirte_base.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "frame_prefix": frame_prefix,
            "start_controller_manager": start_controller_manager,
            "start_state_publishers": start_state_publishers,
        }.items(),
    )

    arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_master_arm_control"),
                        "launch",
                        "mirte_master_arm_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "frame_prefix": frame_prefix,
            "start_controller_manager": start_controller_manager,
            "start_state_publishers": start_state_publishers,
        }.items(),
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        parameters=[{"default_transport": "theora", "port": 8181}],
        output="screen",
    )

    depth_cam = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("astra_camera"),
                        "launch",
                        "astra_pro_plus.launch.xml",
                    ]
                )
            ]
        )
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1_launch.py"]
                )
            ]
        )
    )

    # Instead of this, we could add a conditional to the launch argument declarations
    # to only launch when the condition is not set. By means of LaunchConfigurationEquals
    ld.add_action(
        GroupAction(
            [
                PushRosNamespace(machine_namespace),
                telemetrix,
                ros2_control,
                state_publishers,
                web_video_server,
                lidar,
                depth_cam,
                arm_control,
                mecanum_drive_control,
            ],
            launch_configurations={
                arg.name: LaunchConfiguration(arg.name)
                for arg in ld.get_launch_arguments()
            },
            forwarding=False,
        )
    )
    return ld
