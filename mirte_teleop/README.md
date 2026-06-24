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