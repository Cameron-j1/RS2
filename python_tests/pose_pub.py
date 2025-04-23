#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from transforms3d.quaternions import mat2quat
from scipy.spatial.transform import Rotation as R

class MultiplePosePublisher(Node):
    def __init__(self):
        super().__init__('multiple_pose_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, '/camera_markers', 10)
        
        self.end_effector_poses = [
        #     np.array([
        #     [ 1.000, -0.000,  0.000,  0.000],
        #     [-0.000,  0.000,  1.000,  0.223],
        #     [-0.000, -1.000, -0.000,  0.694],
        #     [ 0.000,  0.000,  0.000,  1.000]
        # ]),
        np.array([
            [ 0.005, -1.000,  0.001,  0.206],
            [-1.000, -0.005,  0.002,  0.044],
            [-0.002, -0.002, -1.000,  0.401],
            [ 0.0,    0.0,    0.0,    1.0]
        ])

        ]

        # Create a 4x4 rotation matrix for 270 degrees around Y
        rot_y_270 = R.from_euler('y', 270, degrees=True).as_matrix()
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rot_y_270

        # cameraTransform = np.array([
        #     [1,0,0,-0.012],
        #     [0,1,0,-0.06],
        #     [0,0,1,0],
        #     [0,0,0,1],
        # ])

        # cameraTransform = np.array([
        #     [1,0,0,+0.01],#-0.012],
        #     [0,1,0,-0.069],#-0.06],
        #     [0,0,1,0.084],
        #     [0,0,0,1],
        # ])

        # self.end_effector_poses.append(self.end_effector_poses[0] @ cameraTransform)
        
        # Apply the rotation to all poses
        self.processed_poses = []
        for pose in self.end_effector_poses:
            # Apply rotation
            rotated_pose = pose @ rotation_matrix_4x4
            
            # Extract position and quaternion
            position = rotated_pose[:3, 3]
            rotation_matrix = rotated_pose[:3, :3]
            quat_wxyz = mat2quat(rotation_matrix)
            quaternion = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # x, y, z, w
            
            self.processed_poses.append({
                'position': position,
                'quaternion': quaternion
            })
        
        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_markers)
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, pose_data in enumerate(self.processed_poses):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "camera"
            marker.id = i  # Each marker needs a unique ID
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose_data['position'][0]
            marker.pose.position.y = pose_data['position'][1]
            marker.pose.position.z = pose_data['position'][2]
            marker.pose.orientation.x = pose_data['quaternion'][0]
            marker.pose.orientation.y = pose_data['quaternion'][1]
            marker.pose.orientation.z = pose_data['quaternion'][2]
            marker.pose.orientation.w = pose_data['quaternion'][3]
            
            # Set different colors based on marker ID for better visualization
            r, g, b = 0.0, 0.0, 0.0
            if i == 0:
                r, g, b = 1.0, 0.0, 0.0  # Red
            elif i == 1:
                r, g, b = 0.0, 1.0, 0.0  # Green
            elif i == 2:
                r, g, b = 0.0, 0.0, 1.0  # Blue
            elif i == 3:
                r, g, b = 1.0, 1.0, 0.0  # Yellow
            else:
                r, g, b = 0.0, 1.0, 1.0  # Cyan
            
            marker.scale.x = 0.3
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
        
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MultiplePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()