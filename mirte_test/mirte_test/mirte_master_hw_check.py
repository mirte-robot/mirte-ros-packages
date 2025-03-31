import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from mirte_msgs.msg import ServoPosition, Encoder
from mirte_msgs.srv import GetServoOffset, SetServoOffset, SetServoAngle, SetMotorSpeed
import time
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import math


class HWNode(Node):

    def __init__(self):
        super().__init__("check_node")
        self.check()
        # stop the node
        self.destroy_node()
        # raise KeyboardInterrupt
        sys.exit(0)

    def check_wheels(self):
        ok = True
        # check service existence
        motors = ["front_left", "front_right", "rear_left", "rear_right"]
        for motor in motors:
            print("testing motor", motor)
            service = "/io/motor/%s/set_speed" % motor
            # check if service exists
            if service not in self.all_services:
                self.get_logger().error("Service %s does not exist" % service)
                self.ok = False
                continue
            # check if service is callable
            client = self.create_client(SetMotorSpeed, service)
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Service %s is not callable" % service)
                self.ok = False
                continue

            encoder_topic = "/io/encoder/%s" % motor
            # check if topic exists
            if encoder_topic not in self.all_topics:
                self.get_logger().error("Topic %s does not exist" % encoder_topic)
                self.ok = False
                continue

            last_encoder = None

            def update_encoder(msg, m):
                nonlocal last_encoder
                if m != motor:

                    # print("update encoder", msg, m, motor)
                    return
                # print("update encoder", msg)
                last_encoder = msg.value

            self.create_subscription(
                Encoder,
                encoder_topic,
                lambda msg, m=motor: update_encoder(msg, m),
                1,
            )

            start_time = time.time()
            while last_encoder is None and time.time() - start_time < 5:
                time.sleep(0.1)
                rclpy.spin_once(self)
            if last_encoder is None:
                self.get_logger().error("Encoder %s is not publishing" % encoder_topic)
                self.ok = False
                continue
            start_enc = last_encoder
            # set speed to 30
            req = SetMotorSpeed.Request()
            req.speed = 30
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error("Service %s failed" % service)
                self.ok = False
                continue
            # check if encoder is updating
            start_time = time.time()
            while abs(start_enc - last_encoder) < 100 and time.time() - start_time < 5:
                time.sleep(0.1)
                rclpy.spin_once(self)

            req.speed = 0
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error("Service %s failed" % service)
                self.ok = False
                continue
            if abs(start_enc - last_encoder) < 100:
                self.get_logger().error("Encoder %s is not updating" % encoder_topic)
                self.ok = False
                continue

        # check encoder existence and correct
        last_odom = None

        def update_odom(msg):
            nonlocal last_odom
            last_odom = msg

        self.create_subscription(
            Odometry,
            "/mirte_base_controller/odom",
            update_odom,
            1,
        )

        def diff_odom(last_odom, msg):
            if last_odom is None or msg is None:
                return 0
            # check if odom is updating
            return pow(
                pow((last_odom.pose.pose.position.x - msg.pose.pose.position.x), 2)
                + pow((last_odom.pose.pose.position.y - msg.pose.pose.position.y), 2),
                0.5,
            )

        start_time = time.time()
        while last_odom is None and time.time() - start_time < 5:
            time.sleep(0.1)
            rclpy.spin_once(self)
        if last_odom is None:
            self.get_logger().error("Odom is not publishing")
            self.ok = False
            return
        start_odom = last_odom
        # publish to /mirte_base_controller/cmd_vel
        cmd_vel_topic = "/mirte_base_controller/cmd_vel"
        # check if topic exists
        if cmd_vel_topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % cmd_vel_topic)
            self.ok = False
            return
        # check if topic is callable
        client = self.create_publisher(Twist, cmd_vel_topic, 1)
        if not client:
            self.get_logger().error("Topic %s is not callable" % cmd_vel_topic)
            self.ok = False
            return
        # publish to /mirte_base_controller/cmd_vel
        msg = Twist()
        msg.linear.x = 1.0
        client.publish(msg)
        # check if odom is updating
        start_time = time.time()
        while diff_odom(last_odom, start_odom) < 10 and time.time() - start_time < 5:
            client.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self)
        print("diff_odom", diff_odom(last_odom, start_odom))
        print("time diff", time.time() - start_time)
        if diff_odom(last_odom, start_odom) < 3:  # 1m/s, 5s, so at least 4m
            self.get_logger().error("Odom is not updating")
            self.ok = False
            return

        # check multiple speed (done by cmd_vel)

    def check_sonars(self):
        sonars = ["front_left", "rear_left", "front_right", "rear_right"]
        min_vals = [100, 100, 100, 100]
        max_vals = [0, 0, 0, 0]

        def update_sonar(msg, s):
            nonlocal min_vals, max_vals
            if math.isfinite(msg.range):
                # print("update sonar", msg, s)
                min_vals[s] = min(min_vals[s], msg.range)
                max_vals[s] = max(max_vals[s], msg.range)
                # print("update sonar", msg, s, min_vals, max_vals)
            # print("update sonar", msg, s)

        for i, sonar in enumerate(sonars):
            sonar_topic = "/io/distance/%s" % sonar
            # check if topic exists
            if sonar_topic not in self.all_topics:
                self.get_logger().error("Topic %s does not exist" % sonar_topic)
                self.ok = False
                continue
            # check if topic is callable
            client = self.create_subscription(
                Range,
                sonar_topic,
                lambda msg, s=i: update_sonar(msg, s),
                1,
            )
        start_time = time.time()
        # print("sonar_vals", sonar_vals)
        while (
            any([x == 100 for x in min_vals])
            or any([abs(x - y) < 0.1 for (x, y) in zip(min_vals, max_vals)])
        ) and time.time() - start_time < 15:
            time.sleep(0.1)
            # print("sonar_vals", sonar_vals)
            rclpy.spin_once(self, timeout_sec=0.1)
            print(
                "waiting on ",
                [
                    sonar
                    for (x, y, sonar) in zip(min_vals, max_vals, sonars)
                    if abs(x - y) < 0.1 or x == 100
                ],
            )
        # print("sonar_vals", sonar_vals)
        if any([x == 0 for x in max_vals]):
            self.get_logger().error("Sonars are not publishing")
            print("sonar_vals", max_vals)
            print("sonar direction", sonars)
            self.ok = False
            return
        if any([abs(x - y) < 0.1 for (x, y) in zip(min_vals, max_vals)]):
            self.get_logger().error("Sonars are not updating")
            print("sonar_vals", [x for x in zip(min_vals, max_vals)])
            print("sonar direction", sonars)
            self.ok = False
            return
        print("sonars okay!")

    def check(self):
        time.sleep(2)
        self.ok = True
        self.all_services = [name for (name, _) in self.get_service_names_and_types()]
        self.all_topics = [name for (name, _) in self.get_topic_names_and_types()]
        # check wheels
        # self.check_wheels()
        # check odom
        # check sonars
        self.check_sonars()
        # # check oled
        # self.check_oled()
        # # check ina
        # self.check_ina()
        # # check servos
        # self.check_servos()
        # # check camera
        # self.check_camera()
        # # check lidar
        # self.check_lidar()
        # # check imu optionally
        # self.check_imu()
        # report MAC
        if self.ok:
            self.get_logger().info("All checks passed")

        else:
            self.get_logger().error("Some checks failed")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = HWNode()

    rclpy.spin(minimal_publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
