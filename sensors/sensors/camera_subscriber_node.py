import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10
        )
        self.subscription
        self.frame_counter = 0

    def image_callback(self, msg):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        filename = f'img_dump/frame_{self.frame_counter}.jpg'
        cv2.imwrite(filename, frame)
        self.frame_counter += 1
        #cv2.imshow('Camera Feed', frame)
        print(f'{filename} received')
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
