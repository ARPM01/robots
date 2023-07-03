import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(String, 'cmd_input', self.cmd_input_callback, 10)
        self.subscription
        self.get_logger().info('TurtleBot controller node initialized.')

    def cmd_input_callback(self, msg):
        twist_msg = Twist()
        if msg.data == 'forward':
            twist_msg.linear.x = 0.2  # Adjust the linear velocity as desired
        elif msg.data == 'left':
            twist_msg.angular.z = 0.5  # Adjust the angular velocity as desired
        elif msg.data == 'right':
            twist_msg.angular.z = -0.5  # Adjust the angular velocity as desired
        elif msg.data == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().warn('Invalid command: {}'.format(msg.data))
            return

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()

    while True:
        command = input('Enter a command (forward, left, right, stop): ')
        if command:
            msg = String()
            msg.data = command
            turtlebot_controller.cmd_input_callback(msg)
        else:
            break

    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
