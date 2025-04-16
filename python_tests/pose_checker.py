#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from transforms3d.quaternions import mat2quat  # Using transforms3d instead of tf_transformations

class CameraMarkerPublisher(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('camera_marker_publisher')
        
        # Create publisher for the marker
        self.marker_pub = self.create_publisher(Marker, '/camera_marker', 10)
        
        # Camera to end-effector transformation matrix (from hand-eye calibration)
        # Using TSAI method result from your calibration
        self.T_camera_to_ee = np.array([
            [0.95042613, -0.08372958, 0.29946542, 0.00318099],
            [0.03497069, 0.98573678, 0.16462091, 0.08368305],
            [-0.30897772, -0.14598751, 0.93979807, 0.33499275],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        # Current end-effector pose (example - replace with actual pose)
        self.T_ee_to_base = np.array([
            [-0.467, -0.884, -0.003,  0.210],
            [-0.882,  0.466, -0.066, -0.027],
            [ 0.060, -0.028, -0.998,  0.442],
            [ 0.000,  0.000,  0.000,  1.000]
        ])

        # Create a timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def calculate_camera_pose(self):
        """Calculate camera pose based on end-effector pose and hand-eye calibration."""
        # Camera position in base frame: T_ee_to_base * T_camera_to_ee
        T_camera_to_base = self.T_ee_to_base @ self.T_camera_to_ee
        
        # Extract position and orientation
        position = T_camera_to_base[:3, 3]
        rotation_matrix = T_camera_to_base[:3, :3]
        
        # Convert rotation matrix to quaternion using transforms3d
        # Note: mat2quat returns in w, x, y, z format, but we need x, y, z, w
        quat_wxyz = mat2quat(rotation_matrix)
        quaternion = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # Convert to x,y,z,w format
        
        return position, quaternion
    
    def create_camera_marker(self, position, quaternion):
        """Create a marker message representing the camera."""
        marker = Marker()
        marker.header.frame_id = "base_link"  # Change to your robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "camera"
        marker.id = 0
        marker.type = Marker.ARROW  # Using an arrow to represent camera direction
        marker.action = Marker.ADD
        
        # Set the position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Set the orientation
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        
        # Set scale
        marker.scale.x = 0.1  # Arrow length
        marker.scale.y = 0.01  # Arrow width
        marker.scale.z = 0.01  # Arrow height
        
        # Set color (blue camera)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Marker should be persistent
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def timer_callback(self):
        """Callback function for the timer to publish the camera marker."""
        # Calculate current camera pose
        position, quaternion = self.calculate_camera_pose()
        
        # Create and publish the marker
        marker = self.create_camera_marker(position, quaternion)
        self.marker_pub.publish(marker)
        
        # Print current camera position (for debugging)
        self.get_logger().info(f"Publishing camera marker at: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")

    def update_ee_pose(self, new_pose):
        """Update the end-effector pose."""
        self.T_ee_to_base = new_pose
        self.get_logger().info("Updated end-effector pose")

def main(args=None):
    rclpy.init(args=args)
    
    publisher = CameraMarkerPublisher()
    
    # Example of how to update end-effector pose (you could implement a subscriber here)
    # Uncomment the following lines to test with different end-effector poses
    """
    import time
    time.sleep(5)  # Wait 5 seconds
    new_pose = np.array([
        [-0.700, -0.714, 0.000, 0.275],
        [-0.714, 0.700, -0.000, -0.080],
        [-0.000, -0.001, -1.000, 0.090],
        [0.000, 0.000, 0.000, 1.000]
    ])
    publisher.update_ee_pose(new_pose)
    """
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()