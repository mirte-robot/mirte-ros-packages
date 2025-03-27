
ids = [2, 3, 4, 5, 6]

# TODO check with the documentation and the config files
ranges = {
    2: {"min": 3400, "max": 21000, "home": 12000},
    3: {"min": 2832, "max": 20000, "home": 11450},
    4: {"min": 120, "max": 21000, "home": 11750},
    5: {"min": 1128, "max": 21672, "home": 12200},
    6: {"min": 6168, "max": 14224, "home": 10524},
}

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CalibrateNode(Node):

    def __init__(self):
        super().__init__('calibrate_node')
        self.calibrate()

    def calibrate(self):
        # check existence of all servos and that they have a position
        for(id in ids):
            
        # disable all servos
        # tell user to move
        # get offset and current position
        # calculate new offset
        # save offset to servos



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1




def main():
    rclpy.init(args=args)

    minimal_publisher = CalibrateNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
