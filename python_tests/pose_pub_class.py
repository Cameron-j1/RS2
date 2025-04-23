#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from transforms3d.quaternions import mat2quat
from scipy.spatial.transform import Rotation as R
import threading

class CameraPosePublisher:
    """
    A class for publishing camera pose markers in ROS2 from 4x4 transformation matrices.
    Can be imported and used in other Python scripts.
    """
    def __init__(self, node=None):
        """
        Initialize the Camera Pose Publisher.
        
        Args:
            node: An existing ROS2 node. If None, a new node will be created.
        """
        self.running = True
        # Use provided node or create a new one
        if node is None:
            # Initialize ROS context if it's not already initialized
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('camera_pose_publisher')
            self.should_shutdown = True
            # Create a thread to keep the node spinning
            self.spin_thread = threading.Thread(target=self._spin_node)
            self.spin_thread.daemon = True
            self.spin_thread.start()
        else:
            self.node = node
            self.should_shutdown = False
            self.spin_thread = None
            
        self.publisher = self.node.create_publisher(MarkerArray, '/camera_markers_auto', 10)
        self.timer = None
        self.processed_poses = []
        
        
        # Default Y-rotation of 270 degrees (common for cameras)
        rot_y_270 = R.from_euler('y', 270, degrees=True).as_matrix()
        self.default_rotation = np.eye(4)
        self.default_rotation[:3, :3] = rot_y_270
    
    def _spin_node(self):
        """Thread function to keep the node spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def add_pose(self, transformation_matrix, id, apply_default_rotation=True):
        # print("pose added")
        """
        Add a camera pose using a 4x4 transformation matrix.
        
        Args:
            transformation_matrix: 4x4 transformation matrix
            apply_default_rotation: Whether to apply the default 270-degree Y rotation
        """
        if transformation_matrix.shape != (4, 4):
            raise ValueError("Transformation matrix must be 4x4")
            
        # Apply rotation if needed
        if apply_default_rotation:
            rotated_pose = transformation_matrix @ self.default_rotation
        else:
            rotated_pose = transformation_matrix
        
        # Extract position and quaternion
        position = rotated_pose[:3, 3]
        rotation_matrix = rotated_pose[:3, :3]
        quat_wxyz = mat2quat(rotation_matrix)
        quaternion = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # x, y, z, w
        
        self.processed_poses.append({
            'position': position,
            'quaternion': quaternion,
            'id':int(id),
        })

        print(f"saving id {id}")
        
        # If we're already publishing, publish once immediately to show the new pose
        if self.timer is not None:
            self._publish_markers()
    
    def clear_poses(self):
        """Clear all stored poses."""
        self.processed_poses = []
        # Publish empty marker array to clear existing markers
        self._publish_markers()
    
    def start_publishing(self, frequency=10.0):
        """
        Start publishing markers at the specified frequency (Hz).
        
        Args:
            frequency: Publishing frequency in Hz
        """
        if self.timer is not None:
            self.timer.cancel()
        
        # Create a new timer
        period = 1.0 / frequency
        self.timer = self.node.create_timer(period, self._publish_markers)
        
        # Publish immediately once
        self._publish_markers()
        
        self.get_logger().info(f"Started publishing camera markers at {frequency} Hz")
    
    def stop_publishing(self):
        """Stop publishing markers."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info("Stopped publishing camera markers")
    
    def publish_once(self):
        """Publish markers once."""
        self._publish_markers()
    
    def _publish_markers(self):
        # print("published markers!")

        """Publish all processed poses as markers."""
        marker_array = MarkerArray()
        
        for i, pose_data in enumerate(self.processed_poses):
            # print(f"publishing marker {i}")
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "camera"
            marker.id = pose_data['id']  # Each marker needs a unique ID
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose_data['position'][0]
            marker.pose.position.y = pose_data['position'][1]
            marker.pose.position.z = pose_data['position'][2]
            marker.pose.orientation.x = pose_data['quaternion'][0]
            marker.pose.orientation.y = pose_data['quaternion'][1]
            marker.pose.orientation.z = pose_data['quaternion'][2]
            marker.pose.orientation.w = pose_data['quaternion'][3]
            
            # Set different colors based on marker ID
            colors = [
                (1.0, 0.0, 0.0),  # Red
                (0.0, 1.0, 0.0),  # Green
                (0.0, 0.0, 1.0),  # Blue
                (1.0, 1.0, 0.0),  # Yellow
                (0.0, 1.0, 1.0),  # Cyan
                (1.0, 0.0, 1.0),  # Magenta
            ]
            r, g, b = colors[i % len(colors)]
            
            marker.scale.x = 0.3  # Arrow length
            marker.scale.y = 0.01  # Arrow width
            marker.scale.z = 0.01  # Arrow height
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
        
        self.publisher.publish(marker_array)
    
    def get_logger(self):
        """Get the node's logger."""
        return self.node.get_logger()
    
    def shutdown(self):
        """Shutdown the publisher and ROS node if it was created by this class."""
        self.stop_publishing()
        self.running = False
        
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=1.0)
            
        if self.should_shutdown:
            rclpy.shutdown()


# Example usage
def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create publisher
        publisher = CameraPosePublisher()
        
        # Example: Add pose using 4x4 transformation matrix
        transformation_matrix = np.array([
            [ 0.005, -1.000,  0.001,  0.206],
            [-1.000, -0.005,  0.002,  0.044],
            [-0.002, -0.002, -1.000,  0.401],
            [ 0.0,    0.0,    0.0,    1.0]
        ])
        publisher.add_pose(transformation_matrix, 1)
        
        # Start publishing
        publisher.start_publishing(10.0)  # 10 Hz
        
        # Keep the node running until Ctrl+C
        input("Press Enter to stop...")
        
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        if 'publisher' in locals():
            publisher.shutdown()

if __name__ == "__main__":
    main()