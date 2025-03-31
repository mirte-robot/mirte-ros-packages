
# ids = [2, 3, 4, 5, 6]

# TODO check with the documentation and the config files
ranges = {
    2: {"min": 3400, "max": 21000, "home": 12000, "name": "shoulder_pan", "position": 0},
    3: {"min": 2832, "max": 20000, "home": 11450, "name": "shoulder_lift", "position": 0},
    4: {"min": 120, "max": 21000, "home": 11750, "name": "elbow", "position": 0},
    5: {"min": 1128, "max": 21672, "home": 12200, "name": "wrist", "position": 0},
    6: {"min": 6168, "max": 14224, "home": 10524, "name": "gripper", "position": 0},
}

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from mirte_msgs.msg import ServoPosition
from mirte_msgs.srv import GetServoOffset, SetServoOffset, SetServoAngle
import time
import sys

class CalibrateNode(Node):

    def __init__(self):
        super().__init__('calibrate_node')
        self.calibrate()
        # stop the node
        self.destroy_node()
        # raise KeyboardInterrupt
        sys.exit(0)
    def callback(self, msg, servo_id):
        # print("callback", msg, servo_id)
        if self.on:
            global ranges
            ranges[servo_id]["position"] = msg.raw
            # print("servo_id", servo_id, "position", msg.raw)

    def calibrate(self):
        # check existence of all servos and that they have a position
        while len(self.get_topic_names_and_types()) < 3:
            # sleep 1s
            time.sleep(1)
        # print(all_topics)
        time.sleep(2)
        self.on = True
        all_topics = [topic for (topic, type) in self.get_topic_names_and_types()]
        # print(all_topics)
        self.subscribers = []
        for servo_id in ranges:
            print(servo_id)
            # check if /io/servo/hiwonder/{id}/position exists
            topic = "/io/servo/hiwonder/%s/position" % ranges[servo_id]["name"]
            # print(topic)
            if topic not in all_topics:
                self.get_logger().error("Servo %d does not exist" % servo_id)
                return
            print("subscribiing to topic", topic, "for servo", servo_id)

            self.subscribers.append(self.create_subscription(ServoPosition, topic, lambda msg,  s=servo_id: self.callback(msg, s), 1))

            # set offset to 0
            # topic = "/io/servo/hiwonder/%s/_offset" % ranges[servo_id]["name"]
            topic = "/io/servo/hiwonder/%s/_set_offset" % ranges[servo_id]["name"]
            set_pos = self.create_client(SetServoOffset, topic)
            req = SetServoOffset.Request()
            req.centidegrees = 0
            fut = set_pos.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
        for servo_id in ranges:
            servo_data = ranges[servo_id]
            topic = "/io/servo/hiwonder/%s/set_angle" % servo_data["name"]
            set_pos = self.create_client(SetServoAngle, topic)
            req = SetServoAngle.Request()
            req.angle =0.0
            req.degrees = False
            fut = set_pos.call_async(req)
            out = rclpy.spin_until_future_complete(self, fut)
            # time.sleep(1)
        print("servos do exist")
        print("disabling servos")
        enable_client = self.create_client(SetBool, "/io/servo/hiwonder/enable_all_servos")
        req = SetBool.Request()
        req.data = False
        fut = enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        # tell user to move
        print("Move arm to home position")
        try:
            input("Press Enter to continue...")
            rclpy.spin_once(self)
        except KeyboardInterrupt:
            print("Exiting")
            return
        print("reading positions...")
        # spin for 2 sec
        start_time  = time.time()
        while time.time() - start_time < 2:
            rclpy.spin_once(self)
            time.sleep(0.1)
            
        print("done reading")

        # get offset and current position
        self.on = False # no more reading curr position to not influence it from movement.

        for servo_id in ranges:
            servo_data = ranges[servo_id]
            topic = "/io/servo/hiwonder/%s/_offset" % servo_data["name"]
            curr_pos = self.create_client(GetServoOffset, topic)
            req = GetServoOffset.Request()
            fut = curr_pos.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            curr_offset = fut.result().centidegrees
            diff = servo_data["position"] - servo_data["home"]
            totalDiff = diff + curr_offset
            print("diff of", servo_id, " diff: ", diff, " totaldiff: ", totalDiff, " home: ", servo_data["home"], " currpos: ", servo_data["position"],  " curr_off: ",curr_offset)
            if abs(totalDiff) > 200:
                print("setting offset:", totalDiff)
                if abs(totalDiff/24) > 125:
                    print("servo too much off, need to fix the servo")
                    continue
                topic = "/io/servo/hiwonder/%s/_set_offset" % servo_data["name"]
                set_pos = self.create_client(SetServoOffset, topic)
                req = SetServoOffset.Request()
                req.centidegrees = totalDiff
                fut = set_pos.call_async(req)
                rclpy.spin_until_future_complete(self, fut)
            
            # time.sleep(2)
        for servo_id in ranges:
            servo_data = ranges[servo_id]
            topic = "/io/servo/hiwonder/%s/set_angle" % servo_data["name"]
            set_pos = self.create_client(SetServoAngle, topic)
            req = SetServoAngle.Request()
            req.angle =0.0
            req.degrees = False
            fut = set_pos.call_async(req)
            out = rclpy.spin_until_future_complete(self, fut)
            # print("out", out)
            # time.sleep(1)
            # print(curr_offset)
        # calculate new offset
        # save offset to servos
        print("done calibrating, should be in the home position!")



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CalibrateNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
