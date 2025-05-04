#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import threading
import sys
import termios
import tty
import select

class ImageSaver(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Create CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Store the latest image
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # Create output directory if it doesn't exist
        self.output_dir = "saved_images"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Note: topic name should match exactly
            self.listener_callback,
            10)
        
        self.get_logger().info('Image saver node started. Press "s" to save the current image, "q" to quit.')
        
        # Create a thread for keyboard input
        self.running = True
        self.key_thread = threading.Thread(target=self.key_input_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

    def listener_callback(self, msg):
        """Callback function for receiving image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store the latest image with thread safety
            with self.image_lock:
                self.latest_image = cv_image
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def save_image(self):
        """Save the latest image as JPG"""
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().warn('No image available to save')
                return
            
            # Create a timestamp for the filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.output_dir}/image_{timestamp}.jpg"
            
            # Save the image
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f'Image saved as {filename}')

    def key_input_loop(self):
        """Thread function to handle keyboard input"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 's':
                        self.get_logger().info('Save key pressed')
                        self.save_image()
                    elif key == 'q':
                        self.get_logger().info('Quit key pressed')
                        self.running = False
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    
    image_saver = ImageSaver()
    
    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        image_saver.running = False
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()