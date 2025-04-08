import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from mirte_msgs.msg import ServoPosition, Encoder
from mirte_msgs.srv import (
    GetServoOffset,
    SetServoOffset,
    SetServoAngle,
    SetMotorSpeed,
    SetOLEDText,
)
import time
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, BatteryState, PointCloud2, LaserScan, Imu
import math
import subprocess
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
                fut = client.call_async(req)  # force over the PID controller.
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
            print("last_odom", last_odom)
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
        if diff_odom(last_odom, start_odom) < 2:  # 1m/s, 5s, so at least 4m
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

    def check_oled(self):
        oled_service = "/io/oled/oled/set_text"
        # check if service exists
        if oled_service not in self.all_services:
            self.get_logger().error("Service %s does not exist" % oled_service)
            self.ok = False
            return
        # check if service is callable
        client = self.create_client(SetOLEDText, oled_service)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service %s is not callable" % oled_service)
            self.ok = False
            return

        req = SetOLEDText.Request()
        req.text = "Hello World"
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error("Service %s failed" % oled_service)
            self.ok = False
            return
        if fut.result().status == False:
            self.get_logger().error("Service %s failed" % oled_service)
            self.ok = False
            return

        time.sleep(2)
        req.text = "Hello World 2"
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error("Service %s failed" % oled_service)
            self.ok = False
            return
        if fut.result().status == False:
            self.get_logger().error("Service %s failed" % oled_service)
            self.ok = False
            return
        print("oled okay!")

    def check_ina(self):
        # check topic existence
        # check topic publishing and voltages and current is okay
        ina_topic = "/io/power/power_watcher"
        # check if topic exists
        if ina_topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % ina_topic)
            self.ok = False
            return
        curr = None
        volt = None

        def update_ina(msg):
            nonlocal curr, volt
            if math.isfinite(msg.current) and math.isfinite(msg.voltage):
                curr = msg.current
                volt = msg.voltage
                # print("update ina", msg)
                return
            # self.get_logger().error("INA is not publishing")
            # self.ok = False
            return

        # check if topic is callable
        client = self.create_subscription(
            BatteryState, ina_topic, lambda msg: update_ina(msg), 1
        )
        start_time = time.time()
        while time.time() - start_time < 5:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

        if curr is None or volt is None:
            self.get_logger().error("INA is not publishing")
            self.ok = False
            return

        if curr < 0.1 or curr > 10:
            self.get_logger().error("INA current is not okay")
            self.ok = False
            return
        if volt < 10 or volt > 15:
            self.get_logger().error("INA voltage is not okay")
            self.ok = False
            return
        print("ina okay!")

    def check_camera(self):
        camera_topic = "/camera/depth/points"
        # check if topic exists
        if camera_topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % camera_topic)
            self.ok = False
            return
        # check if topic is callable
        received = False

        def update_camera(msg):
            self.get_logger().info("Camera is publishing")
            nonlocal received
            received = True
            return

        client = self.create_subscription(
            PointCloud2, camera_topic, lambda msg: update_camera(msg), 1
        )

        start_time = time.time()
        while time.time() - start_time < 10 and not received:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        if not received:
            self.get_logger().error("Camera is not publishing")
            self.ok = False
            return
        print("camera okay!")

    def check_lidar(self):
        lidar_topic = "/scan"
        # check if topic exists
        if lidar_topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % lidar_topic)
            self.ok = False
            return
        # check if topic is callable
        received = False

        def update_lidar(msg):
            nonlocal received
            received = True
            return

        client = self.create_subscription(
            LaserScan, lidar_topic, lambda msg: update_lidar(msg), 1
        )

        start_time = time.time()
        while time.time() - start_time < 5 and not received:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        if not received:
            self.get_logger().error("Lidar is not publishing")
            self.ok = False
            return
        print("lidar okay!")

    def check_imu(self):
        topic = "/io/imu/movement/data"
        # check if topic exists
        if topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % topic)
            self.ok = False
            return
        # check if topic is callable
        received = False
        lastmsg = None

        def update_imu(msg):
            nonlocal received, lastmsg
            if (
                msg.linear_acceleration.x == 0
                and msg.linear_acceleration.y == 0
                and msg.linear_acceleration.z == 0
            ):  # if sensor is not connected, then it publishes 0.
                return
            if lastmsg is None:
                lastmsg = msg
                return
            if (
                lastmsg is not None
                and lastmsg.linear_acceleration.x == msg.linear_acceleration.x
                and lastmsg.linear_acceleration.y == msg.linear_acceleration.y
                and lastmsg.linear_acceleration.z == msg.linear_acceleration.z
            ):
                # data should always be changing
                print("imu data is not changing")
                return
            received = True
            return

        client = self.create_subscription(Imu, topic, lambda msg: update_imu(msg), 1)
        start_time = time.time()
        while time.time() - start_time < 5 and not received:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        if not received:
            self.get_logger().error("IMU is not publishing")
            self.ok = False
            return
        print("imu okay!")

    def check_servos(self):
        # check if servos are connected
        servos = ["shoulder_pan", "shoulder_lift", "elbow", "wrist", "gripper"]
        # check if service exists
        # call /enable_arm_control
        enable_service = "/enable_arm_control"
        # check if service exists
        if enable_service not in self.all_services:
            self.get_logger().error("Service %s does not exist" % enable_service)
            self.ok = False
            return
        # check if service is callable
        enable_client = self.create_client(SetBool, enable_service)
        if not enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service %s is not callable" % enable_service)
            self.ok = False
            return
        # set to true
        enable_req = SetBool.Request()
        enable_req.data = False
        fut = enable_client.call_async(enable_req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error("Service %s failed" % enable_service)
            self.ok = False
            return

        for servo in servos:
            print("testing servo", servo)
            service = "/io/servo/hiwonder/%s/set_angle" % servo
            # check if service exists
            if service not in self.all_services:
                self.get_logger().error("Service %s does not exist" % service)
                self.ok = False
                continue
            # check if service is callable
            client = self.create_client(SetServoAngle, service)
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Service %s is not callable" % service)
                self.ok = False
                continue
            # set to 0 degrees
            req = SetServoAngle.Request()
            req.angle = 0.0
            req.degrees = False
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error("Service %s failed" % service)
                self.ok = False
                continue
            print("set to 0 degrees", fut.result())

            time.sleep(0.5)
            # set to 0.2 rad
            req.angle = 0.2
            req.degrees = False
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error("Service %s failed" % service)
                self.ok = False
                continue
            print("set to 0.5 degrees", fut.result())
            time.sleep(0.5)
            # set to 0 degrees
            req.angle = -0.1
            req.degrees = False
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is None:
                self.get_logger().error("Service %s failed" % service)
                self.ok = False
                continue
            print("set to -0.1 degrees", fut.result())

            # check if servo is updating
            topic = "/io/servo/hiwonder/%s/position" % servo
            # check if topic exists
            if topic not in self.all_topics:
                self.get_logger().error("Topic %s does not exist" % topic)
                self.ok = False
                continue
            # check if topic is callable
            last_position = None

            def update_position(msg, s):
                nonlocal last_position
                if s != servo:
                    return
                if msg.raw == 0:
                    return
                last_position = msg.angle
                # print("update position", msg, s)

            self.create_subscription(
                ServoPosition,
                topic,
                lambda msg, s=servo: update_position(msg, s),
                1,
            )
            start_time = time.time()
            while last_position is None and time.time() - start_time < 5:
                time.sleep(0.1)
                rclpy.spin_once(self)
            if last_position is None:
                self.get_logger().error("Servo %s is not publishing" % topic)
                self.ok = False
                continue
            print("last_position", last_position)
            if abs(last_position - -0.1) > 0.1:
                self.get_logger().error("Servo %s is not updating" % topic)
                self.ok = False
                continue

        enable_req.data = True
        fut = enable_client.call_async(enable_req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error("Service %s failed" % service)
            self.ok = False
            return

        # publish to /mirte_master_arm_controller/joint_trajectory
        arm_cmd_topic = "/mirte_master_arm_controller/joint_trajectory"
        # check if topic exists
        if arm_cmd_topic not in self.all_topics:
            self.get_logger().error("Topic %s does not exist" % arm_cmd_topic)
            self.ok = False
            return
        # check if topic is callable
        client = self.create_publisher(JointTrajectory, arm_cmd_topic, 1)
        if not client:
            self.get_logger().error("Topic %s is not callable" % arm_cmd_topic)
            self.ok = False
            return
        # ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint'], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start:{ sec: 3, nanosec: 0}}]}"

        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]
        traj.points = []
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        # publish to /mirte_master_arm_controller/joint_trajectory
        count = client.get_subscription_count()
        print("count", count)
        client.publish(traj)
        rclpy.spin_once(self)
        client.wait_for_all_acked()  # TODO: doesnt work all the time
        # check if arm is moving

        print("done servos")

    def report_mac(self):
        # print MAC address of WiFi
        command = "ip link show wlan0 | grep ether | awk '{print $2}'"
        # run command
        mac = subprocess.check_output(command, shell=True).decode("utf-8").strip()
        self.get_logger().info("MAC address: %s" % mac)

    def check(self):
        time.sleep(2)
        self.ok = True
        self.all_services = [name for (name, _) in self.get_service_names_and_types()]
        self.all_topics = [name for (name, _) in self.get_topic_names_and_types()]
        # check wheels
        self.check_wheels()
        # # check odom
        # # check sonars
        self.check_sonars()
        # # # check oled
        self.check_oled()
        # # # check ina
        self.check_ina()
        # # check servos
        self.check_servos()
        # # check camera
        self.check_camera()
        # # check lidar
        self.check_lidar()
        # # check imu optionally
        self.check_imu()
        # report MAC
        self.report_mac()
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
