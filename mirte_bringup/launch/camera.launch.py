from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    video_devices = [
        device
        for device in Path("/dev").glob("video*")
        if (Path("/sys/class/video4linux") / device.name / "name").read_text().strip()
        != "cedrus"
    ]
    if len(video_devices) < 1:
        return LaunchDescription()
    video_device = video_devices[0]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value=TextSubstitution(text=str(video_device)),
                description="The Linux video device of the camera",
                choices=list(str(device) for device in Path("/dev").glob("video*")),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="webcam",
                parameters=[
                    {
                        "pixel_format": "yuyv",
                        "video_device": LaunchConfiguration("video_device"),
                    }
                ],
            ),
            # The video web server is broken untill https://github.com/RobotWebTools/web_video_server/pull/136 is merged,
            # since the port parameter cannot be changed and port is already used (Now on the forked branch)
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                parameters=[{"default_transport": "theora", "port": 8181}],
            ),
        ]
    )
