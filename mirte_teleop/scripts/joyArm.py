#!/usr/bin/env python3.8

import traceback

import rospy
from mirte_msgs.srv import SetServoAngle
from sensor_msgs.msg import Joy
from mirte_msgs.msg import ServoPosition

curr_rot = -1
curr_shoulder = -1
set_rot = -1
set_sh = -1
set_el = -1
curr_el = -1
curr_gr = -1


def callback_joy(data):
    if data.buttons[5]:
        rot = data.axes[6]
        updown = data.axes[7]
        move_arm(rot, updown)
    if data.buttons[0]:
        move_gr(0)
    if data.buttons[1]:
        move_gr(1)


def cb_rot(data):
    global curr_rot
    curr_rot = data.angle


def cb_sh(data):
    global curr_shoulder
    curr_shoulder = data.angle


def cb_el(data):
    global curr_el
    curr_el = data.angle


def cb_gr(data):
    global curr_gr
    curr_gr = data.angle


def move_gr(op):
    set_gr(curr_gr + ((2 * op) - 1) * 0.1)


def move_arm(rot, updown):
    global curr_rot, curr_shoulder
    try:
        print("movearm", rot, updown, curr_el)
        if curr_rot == -1:
            return
        # if curr_shoulder == -1:
        #     return
        # print("movear", rot, updown, curr_rot, rot*0.05)
        # print("rot:", curr_rot + rot * 0.05)
        set_rot(curr_rot + rot * -0.15)
        set_sh(curr_shoulder + updown * 0.1)
        set_el(curr_el + updown * -0.1)
        # print("rotated")
    except Exception:
        print("ex")
        print(traceback.format_exc())


def talker():
    global set_rot, set_sh, set_el, set_gr
    rospy.init_node("arm_control", anonymous=True)
    rospy.Subscriber("/joy", Joy, callback_joy, queue_size=1)
    rospy.Subscriber("/mirte/servos/servoRot/position", ServoPosition, cb_rot)
    rospy.Subscriber("/mirte/servos/servoShoulder/position", ServoPosition, cb_sh)
    rospy.Subscriber("/mirte/servos/servoElbow/position", ServoPosition, cb_el)
    rospy.Subscriber("/mirte/servos/servoGripper/position", ServoPosition, cb_gr)

    set_rot = rospy.ServiceProxy("/mirte/set_servoRot_servo_angle", SetServoAngle)
    set_sh = rospy.ServiceProxy("/mirte/set_servoShoulder_servo_angle", SetServoAngle)
    set_el = rospy.ServiceProxy("/mirte/set_servoElbow_servo_angle", SetServoAngle)
    set_gr = rospy.ServiceProxy("/mirte/set_servoGripper_servo_angle", SetServoAngle)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
