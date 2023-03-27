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
		super().__init__('sensors_subscriber')

		self.set_params()
		self.get_params()
		self.scan_ranges = []
		self.init_scan_state = False

		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		self.create_subcscription(Imu, 'imu', self.imu_callback, qos_profile=qos_profile_sensor_data)
		self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
		self_create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.imu_count = 0


