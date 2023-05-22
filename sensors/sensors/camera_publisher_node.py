import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1.0 / 30, self.publish_image)  # 30 FPS
        self.bridge = CvBridge()

    def publish_image(self):
        cap = cv2.VideoCapture(0)  # 0 represents the default camera index

        if not cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return

        ret, frame = cap.read()

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher.publish(msg)
            #cv2.imshow('Camera Feed', frame)
            #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
