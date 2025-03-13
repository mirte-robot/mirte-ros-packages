from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    start_controller_manager = LaunchConfiguration("start_controller_manager")
    start_state_publishers = LaunchConfiguration("start_state_publishers")

    declared_arguments = [
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description="An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.",
        ),
        DeclareLaunchArgument(
            "start_ros2_control",
            default_value="true",
            description="A boolean whether this launchfile needs to start the ros2 controller manager. Defaults to true.",
        ),
        DeclareLaunchArgument(
            "start_state_publishers",
            default_value="true",
            description="A boolean whether this launchfile needs to start the state publisher and joint boardcaster. Defaults to true.",
        ),
        DeclareLaunchArgument(
            "use_pid_control",
            default_value="true",
            description="Use speed PID control for the wheels, you might need to change the gains in mirte_master_base_control/bringup/config/mirte_base_cotnrol.yaml",
        ),
    ]
    use_pid_control = LaunchConfiguration("use_pid_control")

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
            PythonExpression(['"mirte_base_control.yaml" if "', use_pid_control,
                                           '".lower() in ("yes", "true", "t", "1") else "mirte_base_control_no_pid.yaml"'])
        ],
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
        condition=IfCondition(start_controller_manager),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(start_state_publishers),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(start_state_publishers),
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pid_wheels_controller",
            "mirte_base_controller",
        ], 
        # TODO: In later versions, there is an IfElseSubstitution class, then we only need one without the condition
        condition=IfCondition(use_pid_control),
    )
    robot_controller_spawner_no_pid = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mirte_base_controller",
        ], 
        condition=UnlessCondition(use_pid_control),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        robot_controller_spawner_no_pid,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
