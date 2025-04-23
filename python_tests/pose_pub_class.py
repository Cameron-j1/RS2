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
    
    def add_pose(self, transformation_matrix, id, apply_default_rotation=False):  # Changed default to False
        """
        Add a camera pose using a 4x4 transformation matrix.
        
        Args:
            transformation_matrix: 4x4 transformation matrix
            id: Integer ID for this transform
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
            'id': int(id),
            'rotation_matrix': rotation_matrix  # Store rotation matrix for axis visualization
        })

        print(f"Saved transform with id {id}")
        
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
        
        self.node.get_logger().info(f"Started publishing camera markers at {frequency} Hz")
    
    def stop_publishing(self):
        """Stop publishing markers."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.node.get_logger().info("Stopped publishing camera markers")
    
    def publish_once(self):
        """Publish markers once."""
        self._publish_markers()
    
    def _create_axis_marker(self, pose_data, axis_index, id_offset, color, scale):
        """Create a marker for a specific axis."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "camera_axis"
        marker.id = pose_data['id'] * 10 + id_offset  # Create unique ID
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set starting position
        marker.pose.position.x = pose_data['position'][0]
        marker.pose.position.y = pose_data['position'][1]
        marker.pose.position.z = pose_data['position'][2]
        
        # Get the rotation matrix columns - each column represents an axis direction
        rotation_matrix = pose_data['rotation_matrix']
        
        # Create a quaternion that aligns the arrow with the specific axis
        # For this, we need to create a rotation matrix where the first column (arrow direction)
        # points in the direction of the desired axis from the original rotation
        target_axis = rotation_matrix[:, axis_index]
        
        # Create a rotation matrix for this specific axis
        axis_matrix = np.eye(3)
        
        # Set the X-axis of the arrow to point in our target direction
        axis_matrix[:, 0] = target_axis / np.linalg.norm(target_axis)
        
        # Find perpendicular vectors for Y and Z axes
        # First find a vector not collinear with our target axis
        if np.abs(axis_matrix[0, 0]) < 0.9:
            temp_vec = np.array([1.0, 0.0, 0.0])
        else:
            temp_vec = np.array([0.0, 1.0, 0.0])
            
        # Y-axis is perpendicular to both X-axis and temp vector
        axis_matrix[:, 1] = np.cross(axis_matrix[:, 0], temp_vec)
        axis_matrix[:, 1] = axis_matrix[:, 1] / np.linalg.norm(axis_matrix[:, 1])
        
        # Z-axis is perpendicular to X and Y
        axis_matrix[:, 2] = np.cross(axis_matrix[:, 0], axis_matrix[:, 1])
        
        # Convert to quaternion
        quat_wxyz = mat2quat(axis_matrix)
        
        marker.pose.orientation.x = quat_wxyz[1]
        marker.pose.orientation.y = quat_wxyz[2]
        marker.pose.orientation.z = quat_wxyz[3]
        marker.pose.orientation.w = quat_wxyz[0]
        
        # Scale (length, width, height)
        marker.scale.x = scale[0]  # Length
        marker.scale.y = scale[1]  # Width
        marker.scale.z = scale[2]  # Height
        
        # Color (r, g, b)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Alpha
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def _publish_markers(self):
        """Publish all processed poses as markers with all three axes shown."""
        marker_array = MarkerArray()
        
        for pose_data in self.processed_poses:
            # Define axis colors: X=Red, Y=Green, Z=Blue
            axis_colors = [
                (1.0, 0.0, 0.0),  # X axis - Red
                (0.0, 1.0, 0.0),  # Y axis - Green
                (0.0, 0.0, 1.0),  # Z axis - Blue
            ]
            
            # Create arrows for each axis
            for i, (color, length) in enumerate(zip(axis_colors, [0.1, 0.1, 0.2])):
                marker = self._create_axis_marker(
                    pose_data, 
                    i,           # Axis index (0=X, 1=Y, 2=Z)
                    i+1,         # ID offset
                    color,       # Color for this axis
                    (length, 0.01, 0.01)  # Scale - different length for each axis
                )
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
        publisher.add_pose(transformation_matrix, 1, False)
        
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