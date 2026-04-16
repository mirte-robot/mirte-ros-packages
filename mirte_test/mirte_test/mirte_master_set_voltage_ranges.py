#!/usr/bin/env python3

voltage_min = 10.5
voltage_max = 14

ranges = {
    2: {
        "min": 3400,
        "max": 21000,
        "home": 12000,
        "name": "shoulder_pan",
        "position": 0,
    },
    3: {
        "min": 2832,
        "max": 20000,
        "home": 11450,
        "name": "shoulder_lift",
        "position": 0,
    },
    4: {"min": 120, "max": 21000, "home": 11750, "name": "elbow", "position": 0},
    5: {"min": 1128, "max": 21672, "home": 12200, "name": "wrist", "position": 0},
    # 6: {"min": 6168, "max": 14224, "home": 10524, "name": "gripper", "position": 0},
}

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from mirte_msgs.msg import ServoPosition
from mirte_msgs.srv import (
    GetServoOffset,
    SetServoOffset,
    SetServoAngle,
    SetServoVoltageRange,
)
import time
import sys


class CalibrateNode(Node):

    def __init__(self):
        super().__init__("calibrate_node")
        self.calibrate()
        # stop the node
        self.destroy_node()
        # raise KeyboardInterrupt
        sys.exit(0)

    def enable_arm_control(self, enable):
        enable_service = "/enable_arm_control"
        enable_client = self.create_client(SetBool, enable_service)
        if not enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service %s is not callable" % enable_service)
        else:
            enable_req = SetBool.Request()
            enable_req.data = enable
            fut = enable_client.call_async(enable_req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error(
                    "Service %s failed, will continue with calibration" % enable_client
                )

    def calibrate(self):
        # check existence of all servos and that they have a position
        while len(self.get_topic_names_and_types()) < 3:
            # sleep 1s
            time.sleep(1)
        # print(all_topics)
        self.enable_arm_control(False)

        for servo_id in ranges:
            servo_data = ranges[servo_id]

            topic = "/io/servo/hiwonder/%s/_set_voltage_range" % servo_data["name"]
            set_pos = self.create_client(SetServoVoltageRange, topic)
            req = SetServoVoltageRange.Request()
            req.low = float(voltage_min)
            req.high = float(voltage_max)
            fut = set_pos.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            print(fut.result())
        print("done setting voltages!")
        self.enable_arm_control(True)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CalibrateNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
