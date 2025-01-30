import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEBUG = True
DEBUGGER = True


def generate_launch_description():
    telemetrix_ros_arguments = (
        [
            "--log-level",
            f"{platform.node().replace('-','_').lower()}.io.telemetrix:=debug",
        ]
        if DEBUG
        else []
    )
    prefix = ["gdbserver localhost:8012"] if DEBUGGER else []
    # run:
    # gdb -ex "file '$(ros2 pkg prefix mirte_telemetrix_cpp)/bin/mirte_telemetrix_cpp_node'" -ex "target remote localhost:8012" -ex c
    launch_arguments: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution(
                (
                    FindPackageShare("mirte_telemetrix_cpp"),
                    "config",
                    "mirte_user_config.yaml",
                )
            ),
        ),
        DeclareLaunchArgument(
            "hardware_namespace",
            default_value="io",
            description="The namespace for the Telemetrix Node and the hardware peripherals",
        ),
        DeclareLaunchArgument(
            "frame_prefix", default_value="", description="The TF2 frame prefix"
        ),
    ]

    ld = LaunchDescription(launch_arguments)

    node = Node(
        package="mirte_telemetrix_cpp",
        name="telemetrix",
        executable="mirte_telemetrix_cpp_node",
        parameters=[
            LaunchConfiguration("config_path"),
            {"frame_prefix": LaunchConfiguration("frame_prefix")},
        ],
        prefix=prefix,
        output="screen",
        emulate_tty=True,
        namespace=LaunchConfiguration("hardware_namespace"),
        respawn=True,
        respawn_delay=5,
        ros_arguments=telemetrix_ros_arguments,
        # TODO: Not avialable yet in humble (avialable starting from jazzy)
        # respawn_max_retries=10,
        # Makes Node required
        # on_exit=Shutdown(
        #     reason="The telemetrix node died repeatedly, try some trouble-shooting steps"
        # ),
    )

    ld.add_action(node)
    return ld
