#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.timer = self.create_timer(4.0, self.timer_callback)  # Publish every 2 seconds
        self.bridge = CvBridge()
        
        # Specify the path to your image file
        self.image_path = os.path.join(get_package_share_directory('ur3_test_control'), 'images', 'chesstest3.jpg')  # Replace with actual path
        self.image = self.load_image()
        self.get_logger().info('Image Publisher Node started. Publishing image every 2 seconds.')

    def load_image(self):
        # Load the image from file
        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Image file not found: {self.image_path}')
            return None
        image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)  # Load as BGR
        if image is None:
            self.get_logger().error(f'Failed to load image: {self.image_path}')
        return image

    def timer_callback(self):
        if self.image is not None:
            # Convert the image to a ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            
            # Publish the image
            self.publisher.publish(img_msg)
            self.get_logger().info(f'Published image from {self.image_path}')
        else:
            self.get_logger().warn('No valid image to publish.')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Publisher Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()