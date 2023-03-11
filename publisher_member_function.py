import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import os
cwd = os.getcwd()
file_path = os.path.dirname(__file__)
os.chdir(file_path)
from control import control
os.chdir(cwd)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        example = [{"Label": "Yellow", "Zpos": None, "Ypos": 1.66, "Xpos": -1.16, "Time": None},  # A
                   {"Label": "Blue", "Zpos": None, "Ypos": 1, "Xpos": 1, "Time": None},  # B
                   {"Label": "Yellow", "Zpos": None, "Ypos": 3.2, "Xpos": -0.14, "Time": None},  # C
                   {"Label": "Blue", "Zpos": None, "Ypos": 2.68, "Xpos": 1.64, "Time": None},  # D
                   {"Label": "Yellow", "Zpos": None, "Ypos": 4.62, "Xpos": 1.26, "Time": None},  # E
                   {"Label": "Blue", "Zpos": None, "Ypos": 3.42, "Xpos": 2.88, "Time": None},  # F
                   {"Label": "Yellow", "Zpos": None, "Ypos": 4.98, "Xpos": 3.16, "Time": None}]  # G
        theta = control(example)
        msg = Float32()
        msg.data = theta
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
