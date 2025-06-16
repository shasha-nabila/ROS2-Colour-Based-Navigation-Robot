import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_image)
        
        # Create a test image (white background with three colored balls)
        self.test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255  # White background
        
        # Draw three circles representing the RGB balls
        # Red ball
        cv2.circle(self.test_image, (160, 240), 50, (0, 0, 255), -1)  # BGR format
        # Green ball
        cv2.circle(self.test_image, (320, 240), 50, (0, 255, 0), -1)
        # Blue ball
        cv2.circle(self.test_image, (480, 240), 50, (255, 0, 0), -1)

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.test_image, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Publishing test image with RGB balls')

def main():
    rclpy.init()
    publisher = TestImagePublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()