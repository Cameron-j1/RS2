import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Note: topic name should match exactly
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_image(self):
        with self.lock:
            return self.latest_image.copy() if self.latest_image is not None else None

# Example usage in a main loop
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(camera_node, timeout_sec=0.1)
            frame = camera_node.get_image()
            if frame is not None:
                cv2.imshow("Camera Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
