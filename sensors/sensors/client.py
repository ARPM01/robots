import threading
import cv2
import numpy as np
import sys
from collections import deque
import socket
import time
import pickle

chunksize = 1024

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.twist_counter = 0
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_data = []
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=qos_profile_sensor_data)
        self.get_logger().info('TurtleBot controller node initialized.')

    def lidar_callback(self, msg):
            self.lidar_data = [i for i in msg.ranges if i > 0]
            #self.get_logger().info("Min Scan Range: %s | Max Scan Range: %s \n" % (min(filtered), max(filtered)), throttle_duration_sec = 5)

    def move_towards_object(self, bounding_boxes, object_names, target_object):
        twist_msg = Twist()
        found_target = False
        reached = False

        for bbox, name in zip(bounding_boxes, object_names):
            if name == target_object:
                found_target = True
                x, y, width, height = bbox

                # Calculate the center of the bounding box
                box_center_x = x + width / 2
                box_center_y = y + height / 2

                # Get the image center (assumed to be the robot's current position)
                image_width = 640  # Adjust according to your image size
                image_height = 480  # Adjust according to your image size
                image_center_x = image_width / 2
                image_center_y = image_height / 2

                # Calculate the difference between the box center and image center
                delta_x = box_center_x - image_center_x
                delta_y = box_center_y - image_center_y

                # Adjust the linear and angular velocities based on the box position
                linear_velocity = 0.15  # Adjust as desired
                angular_velocity = 0.4  # Adjust as desired

                # Move the robot towards the object based on the box position
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = -angular_velocity * delta_x / image_width

                self.publisher.publish(twist_msg)
                # Terminate if within distance_threshold
                distance_threshold = 0.30
                if self.lidar_data:
                    min_distance = min(self.lidar_data)

                    if min_distance < distance_threshold and 0.30*640*480 <= width*height:
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        self.publisher.publish(twist_msg)
                        print('\nFound: ', target_object, 'LIDAR reading: ', min_distance)
                        reached = True
                break

        if not found_target: # Twist until target is found
            self.twist_counter += 1
            twist_msg.angular.z = 1.0
            self.publisher.publish(twist_msg)
            time.sleep(0.15)
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)

        if self.twist_counter >= 5: # Reposition
            print(f'NO {target_object} FOUND. Moving forward.')
            twist_msg.linear.x = 0.15
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)
            self.twist_counter = 0

        return reached


def split_bytes(byte_sequence):
    global chunksize
    output, s = [], len(byte_sequence)

    last_length = s % chunksize
    s_length = len(byte_sequence)
    last_sequence = byte_sequence[s_length - last_length:]  # get the last segment
    it_limit = s_length - last_length

    for i in range(0, it_limit, chunksize):
        output.append(byte_sequence[i:i + chunksize])

    if last_length > 0:
        output.append(last_sequence)

    return output

def receive_bounding_boxes(connection):
    bounding_boxes_size = int.from_bytes(connection.recv(4), 'big')
    bounding_boxes_data = b''
    while len(bounding_boxes_data) < bounding_boxes_size:
        data = connection.recv(chunksize)
        if not data:
            break
        bounding_boxes_data += data

    names, bounding_boxes = pickle.loads(bounding_boxes_data)
    return names, bounding_boxes

def main():
    print('Connecting to server...')
    pysock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pysock.connect(("10.158.14.215", 35000))

    rclpy.init(args=None)
    turtlebot_controller = TurtlebotController()

    target_object = input("Enter the target object: ")
    reached = False

    # Start the ROS 2 event loop
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(turtlebot_controller)

    while rclpy.ok():
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            print('ERROR: CAMERA NOT FOUND')
            sys.exit()

        camera.set(3, 480)
        camera.set(4, 640)
        status, frame = camera.read()

        if not status:
            continue

        status, encoded = cv2.imencode(".jpg", frame)

        if not status:
            continue

        encoded = encoded.tobytes()
        encoded_size = str(len(encoded))
        encoded_size = ('X' * (8 - len(encoded_size))) + encoded_size

        pysock.send(encoded_size.encode())

        split_data = split_bytes(encoded)
        for data in split_data:
            pysock.send(data)

        bounding_boxes_names, bounding_boxes = receive_bounding_boxes(pysock)
        print("\nReceived bounding boxes:")
        for name, bbox in zip(bounding_boxes_names, bounding_boxes):
            print(f'name: {name} bbox: {bbox}')

        reached = turtlebot_controller.move_towards_object(bounding_boxes, bounding_boxes_names, target_object)
        camera.release()

        if reached:
            break

        executor.spin_once()

    turtlebot_controller.destroy_node()
    rclpy.shutdown()
