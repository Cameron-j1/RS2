import numpy as np
import cv2
import glob
import os
import re
from scipy.spatial.transform import Rotation as R

def load_specific_images(image_folder_path, pattern="eyeHand*_Color.png"):
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
            print(f"Loaded image: {os.path.basename(path)}")
    
    return images

def detect_aruco_markers(images, camera_matrix, dist_coeffs, marker_size=0.07):
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
                    
                    print(f"Found marker in image: {image_name}")
                    break  # Only use the first marker in each image
    
    return marker_poses, valid_image_indices

def perform_hand_eye_calibration(camera_poses, robot_poses, method=cv2.CALIB_HAND_EYE_TSAI):
    """Perform hand-eye calibration using the collected poses."""
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
        method=method
    )
    
    # Create the transformation matrix
    T_cam_gripper = np.eye(4)
    T_cam_gripper[:3, :3] = R_cam_gripper
    T_cam_gripper[:3, 3] = t_cam_gripper.reshape(-1)
    
    return T_cam_gripper

def try_all_calibration_methods(camera_poses, robot_poses):
    """Try all available hand-eye calibration methods and return results."""
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS
    }
    
    results = {}
    for name, method in methods.items():
        try:
            results[name] = perform_hand_eye_calibration(camera_poses, robot_poses, method)
            print(f"\nCalibration result using {name} method:")
            print(results[name])
        except Exception as e:
            print(f"Method {name} failed with error: {e}")
    
    return results

def main():
    # Define the path to your image folder
    image_folder_path = 'extrinsics_realsense'
    
    # Define camera calibration parameters
    # These should be replaced with your actual camera calibration values
    camera_matrix = np.array([
        [910.88428579,   0.0,         646.56956061],
        [0.0,           909.49046737, 357.94919134],
        [0.0,             0.0,           1.0]
    ])

    dist_coeffs = np.array([0.08300782, 0.09867322, -0.00272421, 0.0034501, -0.91121635])
    
    # Replace this with your actual base-to-marker transformation
    # This is left as a variable as requested
    T_base_to_marker = np.eye(4)  # Identity matrix placeholder

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

    # For demonstration, creating variations of the example pose
    # In practice, replace these with your actual poses corresponding to each image
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

    # Load images
    images = load_specific_images(image_folder_path)
    print(f"Loaded {len(images)} images")
    
    # Marker size in meters (now 7cm = 0.07m)
    marker_size = 0.07
    
    # Detect ArUco markers and compute camera poses
    camera_poses, valid_indices = detect_aruco_markers(images, camera_matrix, dist_coeffs, marker_size)
    print(f"Found markers in {len(camera_poses)} images")
    
    # Filter end-effector poses to match valid images
    valid_end_effector_poses = [end_effector_poses[i] for i in valid_indices]
    
    if len(camera_poses) < 2 or len(valid_end_effector_poses) < 2:
        print("Error: Need at least 2 valid poses for calibration.")
        return
    
    # Try all calibration methods
    print("\nTrying all calibration methods...")
    calibration_results = try_all_calibration_methods(camera_poses, valid_end_effector_poses)
    
    # Use the TSAI method for further calculations as default
    T_camera_to_gripper = calibration_results.get("TSAI", None)
    
    if T_camera_to_gripper is not None:
        # Calculate the camera to base transformation
        T_gripper_to_base = valid_end_effector_poses[0]
        T_camera_to_base = T_gripper_to_base @ T_camera_to_gripper
        
        print("\nCamera to Base Transformation:")
        print(T_camera_to_base)
        
        # Calculate the camera to marker transformation using T_base_to_marker
        T_camera_to_marker = np.linalg.inv(T_base_to_marker) @ T_camera_to_base
        
        print("\nCamera to Marker Transformation:")
        print(T_camera_to_marker)
    
    # Save the TSAI calibration result to a file for later use
    if "TSAI" in calibration_results:
        np.savetxt('hand_eye_calibration_result.txt', calibration_results["TSAI"], delimiter=',')
        print("\nCalibration result saved to 'hand_eye_calibration_result.txt'")

if __name__ == "__main__":
    main()