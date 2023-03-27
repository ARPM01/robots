import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class SensorsSubscriber(Node):
        def __init__(self):
            super().__init__('subscriber_node')
            self.scan_ranges = []

            #self.init_scan_state = False

            self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile=qos_profile_sensor_data)
            self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.laserscan_callback, qos_profile=qos_profile_sensor_data)
            self.imu_count = 0

        def timer_callback(self):
            self.get_logger().info(str(datetime.datetime.now()))

        def imu_callback(self, msg = Imu()):
            self.get_logger().info("Orientation X: %s, Y: %s, Z: %s \n" % (msg.orientation.x, msg.orientation.y, msg.orientation.z), throttle_duration_sec = 5)
            self.get_logger().info("Angular Velocity X: %s, Y: %s, Z: %s \n" %  (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z), throttle_duration_sec = 5)
            self.get_logger().info("Linear Acceleration X: %s, Y: %s, Z: %s \n" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z), throttle_duration_sec = 5)
        def battery_callback(self, msg = BatteryState()):
            self.get_logger().info("Battery Voltage: %s | Temperature: %s \n" % (msg.voltage, msg.temperature), throttle_duration_sec = 5)

        def laserscan_callback(self, msg = LaserScan()):
            filtered = [i for i in msg.ranges if i > 0]
            self.get_logger().info("Min Scan Range: %s | Max Scan Range: %s \n" % (min(filtered), max(filtered)), throttle_duration_sec = 5)

def main(args=None):

    rclpy.init(args=args)

    subscriber_node = SensorsSubscriber()

    rclpy.spin(subscriber_node)

    subscriber_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
