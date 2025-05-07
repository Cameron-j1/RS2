import numpy as np
import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from pose_pub_class import CameraPosePublisher as pose_publisher

def transform_calc_rot_matrix(yaw_calc_T1, yaw_calc_T2):
    # Extract X-Y positions from yaw_calc_T1 and yaw_calc_T2
    x1, y1 = yaw_calc_T1[0, 3], yaw_calc_T1[1, 3]
    x2, y2 = yaw_calc_T2[0, 3], yaw_calc_T2[1, 3]

    # Compute yaw angle in radians
    yaw = np.arctan2(y2 - y1, x2 - x1)

    yaw_deg = yaw * (180/math.pi)
    print(f"Z in the global frame: {yaw_deg}")

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R_yaw = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw,  cos_yaw, 0],
        [0,        0,       1]
    ])

    return R_yaw

def mid_transform(T1, T2):
    """Compute midpoint transform with yaw between two poses."""
    midpoint = (T1[:3, 3] + T2[:3, 3]) / 2.0
    rot = transform_calc_rot_matrix(T1, T2)
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = midpoint
    return T

def rotate_z(transform, degrees):
    """Rotate a 4x4 transform around Z axis by specified degrees."""
    radians = np.deg2rad(degrees)
    cos_z = np.cos(radians)
    sin_z = np.sin(radians)
    Rz = np.array([
        [cos_z, -sin_z, 0],
        [sin_z,  cos_z, 0],
        [0,      0,     1]
    ])
    rotated = np.eye(4)
    rotated[:3, :3] = Rz @ transform[:3, :3]  # Rotate the orientation
    rotated[:3, 3] = transform[:3, 3]         # Keep the same position
    return rotated

def main(args=None):
    rclpy.init(args=args)
    publisher = pose_publisher()

    # Define corner transforms
    bottom_left = np.eye(4)
    bottom_left[0][3] = -0.15
    bottom_left[1][3] = 0.2

    bottom_right = np.eye(4)
    bottom_right[0][3] = 0.15
    bottom_right[1][3] = 0.2

    top_left = np.eye(4)
    top_left[0][3] = -0.15
    top_left[1][3] = 0.5

    top_right = np.eye(4)
    top_right[0][3] = 0.15
    top_right[1][3] = 0.5

    # Add corner markers
    publisher.add_pose(bottom_left, 1)
    publisher.add_pose(bottom_right, 2)
    publisher.add_pose(top_left, 3)
    publisher.add_pose(top_right, 4)

    # Create edge transforms
    bottomEdge = mid_transform(bottom_left, bottom_right)
    topEdge = mid_transform(top_left, top_right)
    leftEdge = mid_transform(bottom_left, top_left)
    rightEdge = mid_transform(bottom_right, top_right)

    # Add edge markers
    publisher.add_pose(bottomEdge, 5)
    publisher.add_pose(topEdge, 6)
    publisher.add_pose(rotate_z(leftEdge, 270), 7)
    publisher.add_pose(rotate_z(rightEdge, 270), 8)

    # Publish once
    publisher.publish_once()

    print("Published 8 poses (4 corners, 4 edge midpoints)")
    input("Press Enter to stop...")

    publisher.shutdown()

if __name__ == "__main__":
    main()
