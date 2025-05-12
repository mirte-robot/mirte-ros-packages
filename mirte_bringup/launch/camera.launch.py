from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import subprocess
import re


def generate_launch_description():
    video_regex = re.compile(r"video\d+")
    video_devices = [
        device
        for device in Path("/dev").glob("video*")
        if video_regex.match(device.name)
        and (Path("/sys/class/video4linux") / device.name / "name").read_text().strip()
        != "cedrus"
    ]
    if len(video_devices) < 1:
        return LaunchDescription()
    video_device = video_devices[0]
    nodes = [
        DeclareLaunchArgument(
            "video_device",
            default_value=TextSubstitution(text=str(video_device)),
            description="The Linux video device of the camera",
            choices=list(str(device) for device in Path("/dev").glob("video*")),
        ),
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            parameters=[
                {
                    "default_transport": "theora",
                    "port": 8181,
                    "default_stream_type": "ros_compressed",
                    "address": "localhost",  # Nginx will proxy it on /ros-video/
                }
            ],
        ),
    ]
    for device in video_devices:
        cmd = f'v4l2-ctl --device={device} --all | grep "Format Video Capture"'
        out = subprocess.run(
            ["bash", "-c", cmd],
            check=False,
            capture_output=True,
        )
        if out.returncode != 0:
            continue
        print(device, out.stdout.decode())
        device_name = device.name
        nodes.append(
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=device_name,
                namespace=device_name,
                parameters=[
                    {
                        "pixel_format": "yuyv2rgb",
                        "video_device": str(device),
                    }
                ],
            )
        )
    return LaunchDescription(nodes)
