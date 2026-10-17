# Mirte teleop

Sample teleoperation launch and scripts for MIRTE pioneer and master.


## Teleop Key:
This is an ugly hack to get input from the user, the launch file doesn't actually launch. Don't include this one in other launch files!


# Arm teleop

To teleop the arm, the script uses the /io/servo/... services. You'll need to disable the ros2_arm_control, either with the /enable_ros2_arm_control service or disable it by setting `enable_default` to `false` in `mirte_bringup/config/control/mirte_master_arm_control.yaml:53`.

## Teleop joy mm arm:
Default settings are for an PS4 controller directly connected to the robot.
Change includes to use xbox controller (probably)
- drive:
  - left stick for forward and rotate
  - arrow buttons for move left right
  - need to hold L1 for it to move.
- Arm:
  - right stick to move the arm. 
  - Rotation is left-right
  - Up down is done with up down of the stick. If the shoulder_lift servo is at the lowest position, the elbow and wrist start to move.
  - Open gripper with X, close with O.
- Shutdown robot with options button for 2 seconds.

## Teleop joy ps4:
PS4 controller works differently internally (/dev/jsX) than before, so joy-linux is required to read those inputs.
Uses L1 (left button) as enable for cmd_vel.


## Mirte master arm script:
Controls the arm as described above (mm arm).



# Connecting bluetooth controller directly to robot
Make sure that bluez, teleop-twist-joy, joy and joy-linux are installed on the robot:
```bash
sudo apt install bluez bluetooth ros-humble-teleop-twist-joy ros-humble-joy ros-humble-joy-linux
```

Connect bluetooth controller:
```bash
sudo bluetoothctl
        scan on
        # hold and press share and PS button till fast blink
        # find MAC of controller:
        devices
        pair <MAC>
        connect <MAC>
        trust <MAC>
        exit
```
The controller should now have a blue bar and there should be a `/dev/input/js0` file.


Sometimes you'll need to restart the bluetooth service after it to auto-connect and show up as `/dev/input/jsX`

```bash
sudo systemctl restart bluetooth.service
```

## Kernel issue
If after a reboot the `/dev/input/jsX` file doesn't appear, then it might be that the kernel is missing the driver for it. (Known issue)

```bash
sudo apt-mark unhold "*" # normally we don't want to upgrade kernel packages
sudo apt update
sudo apt upgrade -y # will update kernel packages
sudo reboot
```

After rebooting and pressing the Playstation button, it now should appear.


## Autostart on boot
Add 
```python
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("mirte_teleop"),
                                    "launch",
                                    "teleop_joy_mm_arm.launch.py",
                                ]
                            )
                        ]
                    )
                )
```
to `src/mirte-ros-packages/mirte_bringup/launch/minimal_master.launch.py`, line 300.
