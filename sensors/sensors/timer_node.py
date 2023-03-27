import rclpy
import datetime

from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        self.set_params()
        self.get_params()
        self.timer = self.create_timer(self.hz, self.timer_callback)

    def set_params(self):
        self.declare_parameter('hz', 1)

    def get_params(self):
        self.hz = self.get_parameter('hz').get_parameter_value().integer_value

    def timer_callback(self):
        self.get_logger().info(str(datetime.datetime.now()))

def main(args=None):
    rclpy.init(args=args)
    timer_node = TimerNode()
    rclpy.spin(timer_node)
    timer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
