import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

class FrameListener(Node):
    def __init__(self, target_frame='tool0', source_frame='base_link'):
        super().__init__('frame_listener_node')

        self.target_frame = target_frame
        self.source_frame = source_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform_matrix(self):
        """
        Returns a 4x4 homogeneous transformation matrix from source_frame to target_frame.
        If unavailable, returns None.
        """
        # print("getting transform")

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                now)

            t = trans.transform.translation
            q = trans.transform.rotation

            # Convert quaternion to rotation matrix
            r = R.from_quat([q.x, q.y, q.z, q.w])
            rot_matrix = r.as_matrix()

            # Build the 4x4 transform
            T = np.eye(4)
            T[0:3, 0:3] = rot_matrix
            T[0:3, 3] = [t.x, t.y, t.z]

            return T

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return None

def main():
    rclpy.init()

    node = FrameListener(target_frame='tool0', source_frame='base_link')

    try:
        while(True):
            rclpy.spin_once(node, timeout_sec=1.0)
    
            T = node.get_transform_matrix()
            if T is not None:
                np.set_printoptions(precision=3, suppress=True)
                print("Transformation matrix (base_link → tool0):")
                print(T)
            else:
                print("Transform not available.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()