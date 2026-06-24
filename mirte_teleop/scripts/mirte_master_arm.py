#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from mirte_msgs.srv import SetServoAngle
from control_msgs.action import GripperCommand


class MirteMasterArm(Node):
    def __init__(self):
        super().__init__("mirte_master_arm")

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 1)

        self.shoulder_client = self.create_client(
            SetServoAngle, "/io/servo/hiwonder/shoulder_lift/set_angle"
        )
        self.shoulder_pan_client = self.create_client(
            SetServoAngle, "/io/servo/hiwonder/shoulder_pan/set_angle"
        )
        self.elbow_client = self.create_client(
            SetServoAngle, "/io/servo/hiwonder/elbow/set_angle"
        )
        self.wrist_client = self.create_client(
            SetServoAngle, "/io/servo/hiwonder/wrist/set_angle"
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, "/mirte_master_gripper_controller/gripper_cmd"
        )
        # Right stick: axis 3 = right horizontal, axis 4 = right vertical (may vary by controller)
        self.right_axis_horizontal = 5  # rotation
        self.right_axis_vertical = 2  # lift

        self.shoulder_angle = 0.0
        self.shoulder_pan_angle = 0.0

        self.deadzone = 0.1
        self.step = 0.050  # rad per callback

        self.axes = []
        self.buttons = []
        self.timer = self.create_timer(0.1, self.joy_calc)  # 10 Hz
        self.shutdown_timer = None  # Timer for shutdown button press duration
        self.get_logger().info("MirteMasterArm node started, listening to /joy")

    def joy_callback(self, msg: Joy):
        self.axes = msg.axes
        self.buttons = msg.buttons

    def joy_calc(self):
        self.calc_gripper()
        self.check_shutdown()
        axes = self.axes
        # print(f"Received Joy message: {msg}")
        if len(axes) <= max(self.right_axis_horizontal, self.right_axis_vertical):
            self.get_logger().warn("Not enough axes in Joy message")
            return

        horiz = axes[self.right_axis_horizontal]
        vert = axes[self.right_axis_vertical]

        shoulder_changed = False
        shoulder_pan_changed = False

        if abs(horiz) > self.deadzone:
            self.shoulder_angle += horiz * -self.step
            self.shoulder_angle = max(-6.0, min(1.77, self.shoulder_angle))
            shoulder_changed = True

        if abs(vert) > self.deadzone:
            self.shoulder_pan_angle += vert * self.step
            self.shoulder_pan_angle = max(-1.7, min(1.7, self.shoulder_pan_angle))
            shoulder_pan_changed = True

        if shoulder_changed:
            self.calc_shoulder_angles(self.shoulder_angle)
            self.send_servo_request(
                self.shoulder_client, self.shoulder_angle, "shoulder"
            )
            self.send_servo_request(self.elbow_client, self.elbow_angle, "elbow")
            self.send_servo_request(self.wrist_client, self.wrist_angle, "wrist")

        if shoulder_pan_changed:
            self.send_servo_request(
                self.shoulder_pan_client, self.shoulder_pan_angle, "shoulder_pan"
            )
        # print(f"Shoulder angle: {self.shoulder_angle:.2f}, Shoulder pan angle: {self.shoulder_pan_angle:.2f}")

    def send_gripper_request(self, angle):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = angle
        goal_msg.command.max_effort = 10.0

        self.gripper_action_client.wait_for_server()
        print(f"Sending gripper command: {angle}")
        return self.gripper_action_client.send_goal_async(goal_msg)

    def calc_gripper(self):
        if len(self.buttons) < 2:
            self.get_logger().warn("Not enough buttons in Joy message")
            return
        # print(f"Gripper button state: {self.buttons}")
        open_button = self.buttons[2]  # Assuming button 2 is for opening the gripper
        close_button = self.buttons[1]  # Assuming button 1 is for closing the gripper

        if open_button and not close_button:
            future = self.send_gripper_request(-0.5)  # Open gripper
            future.add_done_callback(self.goal_response_callback)
        elif close_button and not open_button:
            future = self.send_gripper_request(0.6)  # Close gripper
            future.add_done_callback(self.goal_response_callback)

        #     rclpy.spin_until_future_complete(self, future)

    def check_shutdown(self):
        # if options button is pressed for 2 seconds, shutdown the robot
        print(f"Buttons: {self.buttons}")
        if len(self.buttons) < 10:
            self.get_logger().warn("Not enough buttons in Joy message")
            return

        if self.buttons[9] == 1:  # Assuming button 9 is the options button
            if self.shutdown_timer is None:
                self.shutdown_timer = self.create_timer(2.0, self.shutdown_robot)
        else:
            if self.shutdown_timer is not None:
                self.shutdown_timer.cancel()
                self.shutdown_timer = None  # Reset the timer if the button is released
            self.shutdown_timer = None  # Reset the timer if the button is released

    def shutdown_robot(self):
        # check if node is running on robot, check if /home/mirte exists, if so, shutdown the robot
        import os

        if os.path.exists("/home/mirte"):
            self.get_logger().info("Shutting down the robot...")
            os.system("sudo shutdown now")
        else:
            self.get_logger().info("Not running on robot, shutting down ROS...")
            self.destroy_node()
            raise SystemExit("Shutting down ROS...")
        rclpy.shutdown()

    def send_servo_request(self, client, angle, name):
        if not client.service_is_ready():
            self.get_logger().warn(f"{name} service not available")
            return

        req = SetServoAngle.Request()
        req.angle = angle
        req.degrees = False
        future = client.call_async(req)
        future.add_done_callback(lambda f: self.service_response_callback(f, name))

    def calc_shoulder_angles(self, shoulder_angle):
        # if shoulder angle is less than -1.66, start moving the elbow to go down.
        if shoulder_angle < -1.66:
            self.elbow_angle = (
                shoulder_angle + 1.66
            ) * 1.0  # simple linear mapping for demonstration

        else:
            self.elbow_angle = 0.0  # (shoulder_angle + 1.66) * 0.5  # simple linear mapping for demonstration

        self.wrist_angle = (
            -self.elbow_angle
        )  # simple inverse relationship for demonstration

    def service_response_callback(self, future, name):
        try:
            out = future.result()
        except Exception as e:
            self.get_logger().error(f"{name} service call failed: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = MirteMasterArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
