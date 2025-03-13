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
    PythonExpression,
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
        ),
        DeclareLaunchArgument(
            "use_base_pid_control",
            default_value="true",
            description="Use speed PID control for the wheels, you might need to change the gains in mirte_master_base_control/bringup/config/mirte_base_cotnrol.yaml",
        ),
    ]
    use_base_pid_control = LaunchConfiguration("use_base_pid_control")
    arm_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("mirte_master_arm_control"),
            "config",
            "mirte_master_arm_control.yaml",
        ]
    )

    base_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("mirte_base_control"),
            "config",
            PythonExpression(
                [
                    '"mirte_base_control.yaml" if "',
                    use_base_pid_control,
                    '".lower() in ("yes", "true", "t", "1") else "mirte_base_control_no_pid.yaml"',
                ]
            ),
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(base_controller_yaml, allow_substs=True),
            ParameterFile(arm_controller_yaml, allow_substs=True),
        ],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            ("~/tf_odometry", "/tf"),
        ],
    )

    nodes = [
        control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
