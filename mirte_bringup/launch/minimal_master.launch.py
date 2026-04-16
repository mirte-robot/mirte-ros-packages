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
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from pathlib import Path

ticks = 1321
invert_motors = False


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
            DeclareLaunchArgument(
                "use_base_pid_control",
                default_value="true",
                description="Use speed PID control for the wheels, you might need to change the gains in mirte_master_base_control/bringup/config/mirte_base_cotnrol.yaml",
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
    use_base_pid_control = LaunchConfiguration(
        "use_base_pid_control",
    )
    ticks_arg = LaunchConfiguration("ticks", default=str(ticks))
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
        launch_arguments=(
            {
                "config_path": PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "telemetrix_config",
                        "mirte_master_config.yaml",
                    ]
                ),
                "hardware_namespace": hardware_namespace,
                "frame_prefix": frame_prefix,
            }
            | (
                {
                    "overlay_config_path": PathJoinSubstitution(
                        [
                            FindPackageShare("mirte_bringup"),
                            "telemetrix_config/overlay",
                            "200rpm_wheels.yaml",
                        ]
                    )
                }
                if invert_motors
                else {}
            )
        ).items(),
    )
    ros2_control = GroupAction(
        actions=[
            SetRemap(
                dst="/mirte_base_controller/cmd_vel",
                src="/mirte_base_controller/cmd_vel_unstamped",
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "launch",
                        "ros2_control.launch.py",
                    ]
                ),
                launch_arguments={
                    "frame_prefix": frame_prefix,
                    "use_base_pid_control": use_base_pid_control,
                }.items(),
            ),
            # IncludeLaunchDescription(
            #     [
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("mirte_bringup"),
            #                 "launch",
            #                 "ros2_control.launch.py",
            #             ]
            #         )
            #     ]
            # # ),
            # # remappings=[{"/mirte_base_control/cmd_vel", "/mirte_base_control/cmd_vel_unstamped"}],
            # launch_arguments={
            #     "frame_prefix": frame_prefix,
            #     "use_base_pid_control": use_base_pid_control,
            # }.items(),
            # )
        ]
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
        launch_arguments={"frame_prefix": frame_prefix, "ticks": ticks_arg}.items(),
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
            "use_pid_control": use_base_pid_control,
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
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_bringup"),
                        "launch",
                        "gripper_camera.launch.py",
                    ]
                )
            ]
        )
    )
    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        parameters=[
            {
                "default_transport": "theora",
                "port": 8181,
                "default_stream_type": "ros_compressed",
                "address": "localhost",  # Nginx will proxy it on /ros-video/
            }
        ],
        output="screen",
    )

    # use lsusb to detect which depth cam is connected
    astra_devices = [device.name for device in Path("/dev").glob("astra*")]
    print(astra_devices)
    if "astradepth" in astra_devices and "astrauvc" in astra_devices:
        depth_cam = GroupAction(
            actions=[
                # SetRemap(src="/camera/color/image_raw", dst="/camera/color/_image_raw"),
                # SetRemap(src="/camera/depth/image_raw", dst="/camera/depth/_image_raw"),
                # SetRemap(src="/camera/ir/image_raw", dst="/camera/ir/_image_raw"),
                IncludeLaunchDescription(
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
                    ),
                ),
            ]
        )
    elif "astra_mini_pro" in astra_devices:
        depth_cam = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("orbbec_camera"),
                                    "launch",
                                    "astra.launch.py",  # for the astra mini S pro
                                ]
                            )
                        ]
                    ),
                ),
            ]
        )
    else:
        print("No compatible depth camera found, not launching any depth camera node.")
        depth_cam = None
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "auto_standby": "true",
        }.items(),
    )
    rosbridge = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            ]
        ),
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
                cameras,
                web_video_server,
                lidar,
                *(
                    [depth_cam] if depth_cam is not None else []
                ),  # if no cam is found, just not start it as it is unknown which one to start.
                arm_control,
                mecanum_drive_control,
                rosbridge,
            ],
            launch_configurations={
                arg.name: LaunchConfiguration(arg.name)
                for arg in ld.get_launch_arguments()
            },
            forwarding=False,
        )
    )
    return ld
