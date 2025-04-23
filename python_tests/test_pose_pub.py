from pose_pub_class import CameraPosePublisher
import rclpy
from rclpy.node import Node
import numpy as np

# Initialize ROS
rclpy.init()
node = Node('my_node')

# Create publisher
publisher = CameraPosePublisher(node)

# Add a camera pose using a 4x4 transformation matrix
transformation_matrix = np.array([
    [ 0.005, -1.000,  0.001,  0.206],
    [-1.000, -0.005,  0.002,  0.044],
    [-0.002, -0.002, -1.000,  0.401],
    [ 0.0,    0.0,    0.0,    1.0]
])
publisher.add_pose(transformation_matrix)

# Publish once
publisher.publish_once()

# Or start continuous publishing
publisher.start_publishing(10.0)  # 10 Hz