# Mirte teleop

Sample teleoperation launch and scripts for MIRTE pioneer and master.


## Teleop Key:
This is an ugly hack to get input from the user, the launch file doesn't actually launch. Don't include this one in other launch files!


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
        connect <MAC>
        trust <MAC>
        exit
```
The controller should now have a blue bar and there should be a `/dev/js0` file.


Sometimes you'll need to restart the bluetooth service after it to auto-connect and show up as `/dev/jsX`

```bash
sudo systemctl restart bluetooth.service
```



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