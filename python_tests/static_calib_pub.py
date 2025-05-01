import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class ArucoTFPublisher(Node):
    def __init__(self, matrix: np.ndarray):
        super().__init__('aruco_tf_publisher')

        if matrix.shape != (4, 4):
            raise ValueError("Transformation matrix must be 4x4")

        self.translation = matrix[:3, 3]
        self.quaternion = R.from_matrix(matrix[:3, :3]).as_quat()  # [x, y, z, w]

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'aruco_frame'

        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]

        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]
        print('published')
        self.br.sendTransform(t)

def main():
    rclpy.init()

    # 4x4 homogeneous transformation matrix
    matrix = np.array([
        [-0.89587442,  0.25013231,  0.36720955, -0.12574469],
        [ 0.12125400,  0.93274445, -0.33953684, -0.02215785],
        [-0.42744180, -0.25965674, -0.86595143,  0.64774652],
        [0.0, 0.0, 0.0, 1.0]
    ])

    node = ArucoTFPublisher(matrix)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
