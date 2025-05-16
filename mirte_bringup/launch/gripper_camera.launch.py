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
    # video_device = video_devices[0]
    gripper_cams = ["HD Camera: HD Camera", "USB 2.0 PC Cam"]
    video_devices = [
        device
        for device in video_devices
        if (Path("/sys/class/video4linux") / device.name / "name").read_text().strip()
        in gripper_cams
    ]
    nodes = []
    gripper_count = 0
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
                name=(
                    f"gripper_camera_{gripper_count}"
                    if gripper_count > 0
                    else "gripper_camera"
                ),
                namespace=(
                    f"gripper_camera_{gripper_count}"
                    if gripper_count > 0
                    else "gripper_camera"
                ),
                remappings=[
                    ("image_raw", f"_image_raw"),  # discourage image_raw by hiding it.
                ],
                parameters=[
                    {
                        "pixel_format": "yuyv2rgb",
                        "video_device": str(device),
                    }
                ],
            )
        )
        gripper_count += 1
    return LaunchDescription(nodes)
