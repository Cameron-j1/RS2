import numpy as np
import cv2
import glob
import os
import re
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time

class HandEyeCalibration(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration')
        self.marker_publisher = self.create_publisher(MarkerArray, 'hand_eye_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transforms)
        
        # Store transformations for visualization
        self.aruco_pose = None
        self.ee_pose = None
        self.camera_pose = None
        
    def load_specific_images(self, image_folder_path, pattern="eyeHand*_Color.png"):
        """Load images matching the specific naming pattern from the folder."""
        image_paths = glob.glob(os.path.join(image_folder_path, pattern))
        images = []
        
        # Sort images numerically (eyeHand1_Color.png, eyeHand2_Color.png, etc.)
        def extract_number(filename):
            match = re.search(r'eyeHand(\d+)_Color\.png', filename)
            return int(match.group(1)) if match else 0
        
        # Sort paths by the extracted number
        image_paths.sort(key=lambda path: extract_number(os.path.basename(path)))
        
        for path in image_paths:
            img = cv2.imread(path)
            if img is not None:
                images.append((os.path.basename(path), img))
                self.get_logger().info(f"Loaded image: {os.path.basename(path)}")
        
        return images

    def detect_aruco_markers(self, images, camera_matrix, dist_coeffs, marker_size=0.07):
        """Detect ArUco markers in each image and compute their poses."""
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        marker_poses = []
        valid_image_indices = []
        
        # Calculate half size for corner coordinates
        half_size = marker_size / 2
        
        for idx, (image_name, image) in enumerate(images):
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                # For each detected marker
                for i in range(len(ids)):
                    # Define object points based on correct marker size (7cm = 0.07m)
                    objPoints = np.array([
                        [-half_size, -half_size, 0],  # Bottom-left
                        [ half_size, -half_size, 0],  # Bottom-right
                        [ half_size,  half_size, 0],  # Top-right
                        [-half_size,  half_size, 0]   # Top-left
                    ])
                    
                    # Get the corners of this marker
                    imagePoints = corners[i][0].astype(np.float32)
                    
                    # Solve for pose
                    success, rvec, tvec = cv2.solvePnP(
                        objPoints, 
                        imagePoints, 
                        camera_matrix, 
                        dist_coeffs
                    )
                    
                    if success:
                        # Convert rotation vector to rotation matrix
                        R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                        
                        # Create transformation matrix
                        T_marker_to_cam = np.eye(4)
                        T_marker_to_cam[:3, :3] = R_marker_to_cam
                        T_marker_to_cam[:3, 3] = tvec.reshape(-1)
                        
                        # Invert to get camera to marker transformation
                        T_cam_to_marker = np.linalg.inv(T_marker_to_cam)
                        
                        marker_poses.append(T_cam_to_marker)
                        valid_image_indices.append(idx)
                        
                        self.get_logger().info(f"Found marker in image: {image_name}")
                        break  # Only use the first marker in each image
        
        return marker_poses, valid_image_indices

    def perform_hand_eye_calibration(self, camera_poses, robot_poses):
        """Perform hand-eye calibration using the TSAI method."""
        # Prepare rotation matrices and translation vectors
        R_cam = []
        t_cam = []
        R_robot = []
        t_robot = []
        
        for T_cam in camera_poses:
            R_cam.append(T_cam[:3, :3])
            t_cam.append(T_cam[:3, 3])
        
        for T_robot in robot_poses:
            R_robot.append(T_robot[:3, :3])
            t_robot.append(T_robot[:3, 3])
        
        # Convert to format needed by OpenCV
        R_cam = [np.array(r) for r in R_cam]
        t_cam = [np.array(t).reshape(3, 1) for t in t_cam]
        R_robot = [np.array(r) for r in R_robot]
        t_robot = [np.array(t).reshape(3, 1) for t in t_robot]
        
        # Perform calibration (camera to end-effector transformation)
        R_cam_gripper, t_cam_gripper = cv2.calibrateHandEye(
            R_gripper2base=R_robot,  # Gripper to base transformations
            t_gripper2base=t_robot,
            R_target2cam=R_cam,      # Marker to camera transformations
            t_target2cam=t_cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # Create the transformation matrix
        T_cam_gripper = np.eye(4)
        T_cam_gripper[:3, :3] = R_cam_gripper
        T_cam_gripper[:3, 3] = t_cam_gripper.reshape(-1)
        
        return T_cam_gripper

    def create_axis_marker(self, frame_id, ns, id, transform_matrix, scale=0.05, lifetime_sec=0):
        """Create a marker array visualizing coordinate axes."""
        markers = []
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # X axis (red)
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Y axis (green)
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)   # Z axis (blue)
        ]
        
        # Extract position and orientation from transform matrix
        position = transform_matrix[:3, 3]
        
        for axis in range(3):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = id * 3 + axis
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set start point at the origin of the transform
            start_point = Point(x=position[0], y=position[1], z=position[2])
            
            # Set end point along the corresponding axis
            direction = transform_matrix[:3, axis]  # Extract axis column
            end_point = Point(
                x=position[0] + direction[0] * scale,
                y=position[1] + direction[1] * scale,
                z=position[2] + direction[2] * scale
            )
            
            marker.points = [start_point, end_point]
            marker.scale.x = 0.005  # Shaft diameter
            marker.scale.y = 0.01   # Head diameter
            marker.scale.z = 0.0    # Head length
            marker.color = colors[axis]
            
            if lifetime_sec > 0:
                marker.lifetime = rclpy.duration.Duration(seconds=lifetime_sec).to_msg()
                
            markers.append(marker)
            
        return markers

    def broadcast_transform(self, parent_frame, child_frame, transform_matrix):
        """Broadcast a transform to tf2."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # Set translation
        t.transform.translation.x = transform_matrix[0, 3]
        t.transform.translation.y = transform_matrix[1, 3]
        t.transform.translation.z = transform_matrix[2, 3]
        
        # Set rotation (convert to quaternion)
        r = R.from_matrix(transform_matrix[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_transforms(self):
        """Publish all transforms and marker arrays for visualization."""
        if self.aruco_pose is None or self.ee_pose is None or self.camera_pose is None:
            return
            
        # Broadcast transforms
        self.broadcast_transform("world", "aruco_marker", self.aruco_pose)
        self.broadcast_transform("world", "end_effector", self.ee_pose)
        self.broadcast_transform("world", "camera", self.camera_pose)
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Add ArUco marker axes visualization
        aruco_markers = self.create_axis_marker("world", "aruco_axes", 0, self.aruco_pose, scale=0.1)
        marker_array.markers.extend(aruco_markers)
        
        # Add end effector axes visualization
        ee_markers = self.create_axis_marker("world", "ee_axes", 1, self.ee_pose, scale=0.1)
        marker_array.markers.extend(ee_markers)
        
        # Add camera axes visualization
        camera_markers = self.create_axis_marker("world", "camera_axes", 2, self.camera_pose, scale=0.1)
        marker_array.markers.extend(camera_markers)
        
        # Publish marker array
        self.marker_publisher.publish(marker_array)

    def run_calibration(self):
        """Run the hand-eye calibration and set up visualization."""
        # Define the path to your image folder
        image_folder_path = 'extrinsics_realsense'
        
        # Define camera calibration parameters
        camera_matrix = np.array([
            [910.88428579,   0.0,         646.56956061],
            [0.0,           909.49046737, 357.94919134],
            [0.0,             0.0,           1.0]
        ])

        dist_coeffs = np.array([0.08300782, 0.09867322, -0.00272421, 0.0034501, -0.91121635])
        
        # Define base-to-marker transformation
        T_base_to_ee_at_aruco = np.array([
            [-0.466, -0.885, 0.014, 0.268],
            [-0.885, 0.466, -0.011, -0.077],
            [0.003, -0.017, -1.000, 0.000],
            [0.000, 0.000, 0.000, 1.000]
        ])

        # Coordinate transformation matrix (end effector to aruco marker)
        T_ee_to_aruco = np.array([
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ])

        # Calculate base to aruco marker transformation
        T_base_to_marker = np.matmul(T_base_to_ee_at_aruco, T_ee_to_aruco)
        
        # Set ArUco marker pose for visualization
        self.aruco_pose = T_base_to_marker

        # Example end effector poses
        end_effector_poses = [
            np.array([
                [-0.245, -0.961, -0.131,  0.230],
                [-0.908,  0.275, -0.317,  0.038],
                [ 0.341,  0.041, -0.939,  0.468],
                [ 0.000,  0.000,  0.000,  1.000]
            ]),
            np.array([
                [-0.954,  0.027,  0.298,  0.149],
                [ 0.179,  0.850,  0.496, -0.253],
                [-0.240,  0.527, -0.815,  0.359],
                [ 0.000,  0.000,  0.000,  1.000]
            ]),
            np.array([
                [ 0.687, -0.647,  0.331,  0.054],
                [-0.413, -0.722, -0.555,  0.172],
                [ 0.598,  0.245, -0.763,  0.498],
                [ 0.000,  0.000,  0.000,  1.000]
            ]),
            np.array([
                [-0.755, -0.655,  0.031,  0.189],
                [-0.656,  0.755, -0.004, -0.015],
                [-0.020, -0.024, -1.000,  0.516],
                [ 0.000,  0.000,  0.000,  1.000]
            ]),
            np.array([
                [ 0.510, -0.853, -0.111,  0.291],
                [-0.733, -0.363, -0.575,  0.218],
                [ 0.451,  0.374, -0.810,  0.381],
                [ 0.000,  0.000,  0.000,  1.000]
            ])
        ]

        rot_z_90 = R.from_euler('y', 270, degrees=True).as_matrix()
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = rot_z_90

        for i in range(len(end_effector_poses)):
            end_effector_poses[i] = end_effector_poses[i] @ rotation_matrix_4x4
        
        # Store one end effector pose for visualization
        self.ee_pose = end_effector_poses[0]

        # Load images
        images = self.load_specific_images(image_folder_path)
        self.get_logger().info(f"Loaded {len(images)} images")
        
        # Marker size in meters (7cm = 0.07m)
        marker_size = 0.07
        
        # Detect ArUco markers and compute camera poses
        camera_poses, valid_indices = self.detect_aruco_markers(images, camera_matrix, dist_coeffs, marker_size)
        self.get_logger().info(f"Found markers in {len(camera_poses)} images")
        
        # Filter end-effector poses to match valid images
        valid_end_effector_poses = [end_effector_poses[i] for i in valid_indices]
        
        if len(camera_poses) < 2 or len(valid_end_effector_poses) < 2:
            self.get_logger().error("Error: Need at least 2 valid poses for calibration.")
            return
        
        # Perform calibration using TSAI method
        T_camera_to_gripper = self.perform_hand_eye_calibration(camera_poses, valid_end_effector_poses)
        self.get_logger().info("\nCalibration result using TSAI method:")
        self.get_logger().info(str(T_camera_to_gripper))
        
        # Calculate the camera to base transformation
        T_gripper_to_base = valid_end_effector_poses[0]
        T_camera_to_base = T_gripper_to_base @ T_camera_to_gripper
        
        # Set camera pose for visualization
        self.camera_pose = T_camera_to_base
        
        self.get_logger().info("\nCamera to Base Transformation:")
        self.get_logger().info(str(T_camera_to_base))
        
        # Calculate the camera to marker transformation using T_base_to_marker
        T_camera_to_marker = np.linalg.inv(T_base_to_marker) @ T_camera_to_base
        
        self.get_logger().info("\nCamera to Marker Transformation:")
        self.get_logger().info(str(T_camera_to_marker))
        
        # Save the calibration result to a file for later use
        np.savetxt('hand_eye_calibration_result.txt', T_camera_to_gripper, delimiter=',')
        self.get_logger().info("\nCalibration result saved to 'hand_eye_calibration_result.txt'")


def main(args=None):
    rclpy.init(args=args)
    
    calibration_node = HandEyeCalibration()
    
    # Run the calibration process
    calibration_node.run_calibration()
    
    try:
        print("Starting ROS2 node to visualize transformations in RViz")
        print("Press Ctrl+C to exit")
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()