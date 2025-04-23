import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from pose_pub_class import CameraPosePublisher
import rclpy
from rclpy.node import Node

from image_sub_class import CameraSubscriber

from pose_sub_class import FrameListener  # Adjust if needed

def invert_transform(T):
    if T.shape != (4, 4):
        raise ValueError("Input must be a 4x4 transformation matrix.")
    
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    
    R_inv = R.T
    t_inv = -R_inv @ t
    
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    
    return T_inv

def calculate_transforms(img, publisher, T_base_to_ee):
    image = img

    #calibration bullshit - a bit here or there
    # parameters = cv2.aruco.DetectorParameters()
    # Disable ArUco3 detection (use ArUco classic detection)
    # parameters.useAruco3Detection = False

    # # Apply the 1/20 factor to make smaller tags more detectable
    # factor = 1/40
    # parameters.minMarkerPerimeterRate *= factor  # Default 0.03 → 0.0015
    # parameters.minCornerDistanceRate *= factor   # Default 0.05 → 0.0025
    # parameters.minMarkerDistanceRate *= factor   # Default 0.05 → 0.0025

    # parameters.perspectiveRemoveIgnoredMarginPerCell *= 0.25

    # # Other parameters that might help with small markers
    # parameters.polygonalApproxAccuracyRate = 0.08      # More tolerance for polygon approximation
    # parameters.errorCorrectionRate = 0.8               # More tolerance for bit errors

    # # ↓↓↓ Filtering tweaks ↓↓↓
    # parameters.minMarkerPerimeterRate = 0.015    # lower = detect smaller tags (default 0.03)
    # parameters.maxErroneousBitsInBorderRate *= 1.5 # allow more border noise
    # parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR




    # Create detector
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detector = cv2.aruco.ArucoDetector(dictionary)

    corners, ids, _ = detector.detectMarkers(image)

    camera_matrix = np.array([
        [913.0385,   0.0,     642.7385],   # fx,  0,  cx
        [0.0,       910.9561, 364.3235],   # 0,  fy,  cy
        [0.0,         0.0,       1.0   ]   # 0,   0,   1
    ], dtype=np.float32)

    # Distortion coefficients (plumb_bob: k1, k2, t1, t2, k3)
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    print(f"ee is X:{T_base_to_ee[0, 3]}, Y:{T_base_to_ee[1, 3]}, Z:{T_base_to_ee[2, 3]} from the base")

    rot_y_270 = R.from_euler('y', 270, degrees=True).as_matrix()
    rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = rot_y_270

    T_ee_to_base = invert_transform(T_base_to_ee)

    publisher.add_pose(T_base_to_ee, 123) #using 123 as a placeholder

    #from eye in hand calibration
    T_cam_to_ee = (np.array([
        [ 0.000338072668,  0.999999801432,  0.000329791286,  0.017143913668],
        [-0.999998857238,  0.000411703512,  0.001278763264, -0.060065853878],
        [ 0.001278926362, -0.000328268593,  0.999999125353,  0.015976452427],
        [ 0.000000000000,  0.000000000000,  0.000000000000,  1.000000000000],
    ])) 

    # --- Step 4: Estimate pose of ArUco marker ---
    # marker_length = 22/1000  # meters
    marker_length = 0.0943  # meters
    marker_corners_3d = np.array([
        [-0.5,  0.5, 0],
        [ 0.5,  0.5, 0],
        [ 0.5, -0.5, 0],
        [-0.5, -0.5, 0]
    ]) * marker_length

    if ids is not None:
        for i, corner in enumerate(corners):
            aruco_id = ids[i][0]
            image_points = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(marker_corners_3d, image_points, camera_matrix, dist_coeffs)

            if success:
                # --- Step 5: Convert pose to homogeneous transform (camera ← marker) ---
                R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                T_marker_to_cam = np.eye(4)
                T_marker_to_cam[:3, :3] = R_marker_to_cam
                T_marker_to_cam[:3, 3] = tvec.flatten()

                T_desired_to_aruco = np.eye(4)
                T_desired_to_aruco[:3, :3] = np.array([
                    [-1, 0, 0],  # Rotate 180 degrees around Z
                    [0, -1, 0],
                    [0, 0, 1]
                ])

                tempRot = R.from_euler('z', 180, degrees=True).as_matrix()
                T_camera_to_base_link_frame = np.eye(4)
                T_camera_to_base_link_frame[:3, :3] = tempRot

                #convert to base frame
                # T_marker_to_cam = T_marker_to_cam @ T_camera_to_base_link_frame

                print(f"marker is X:{T_marker_to_cam[0, 3]}, Y:{T_marker_to_cam[1, 3]}, Z:{T_marker_to_cam[2, 3]} from the camera")

                # print(f"T_marker_to_cam: {T_marker_to_cam}")


                # publisher.add_pose(T_marker_to_cam)

                # T_marker_to_cam = T_marker_to_cam @ T_desired_to_aruco

                # print(f"T_ee_to_base: {T_ee_to_base}")
                # print(f"T_cam_to_ee: {T_cam_to_ee}")
                # print(f"T_marker_to_cam: {T_marker_to_cam}")
                            
                # --- Step 6: Chain transforms ---
                # Robot base ← end effector ← camera ← marker
                # T_marker_to_base = T_ee_to_base @ T_cam_to_ee @ T_marker_to_cam
                T_marker_to_base = T_marker_to_cam @ T_cam_to_ee @ T_ee_to_base
                T_base_to_marker = invert_transform(T_marker_to_base)

                # T_test_intermediate = T_ee_to_base @ T_marker_to_cam

                # print(f"T_base_to_marker: {T_base_to_marker}")

                publisher.add_pose(T_base_to_marker, aruco_id)

                # Extract position
                pos_in_base = T_base_to_marker[:3, 3]
                print("ArUco marker position in robot base frame:", np.round(pos_in_base, 3))

                # Draw axes and marker
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.035)
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                cv2.imshow("Aruco Pose", image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break




#start ROS crap
rclpy.init()
transform_publisher_node = Node('aruco_checker_node')

# Create publisher
transform_publisher = CameraPosePublisher(transform_publisher_node)
camera_subscriber = CameraSubscriber()
transform_subscriber = FrameListener(target_frame='tool0', source_frame='base_link')

try:
    while rclpy.ok():
        rclpy.spin_once(camera_subscriber, timeout_sec=0.1)
        frame = camera_subscriber.get_image()

        ee_T = None
        while ee_T is None:
            rclpy.spin_once(transform_subscriber, timeout_sec=1.0)
            ee_T = transform_subscriber.get_transform_matrix()
        
        if frame is not None and ee_T is not None:
            transform_publisher.clear_poses()
            calculate_transforms(frame, transform_publisher, ee_T)
            transform_publisher.publish_once()
            

        if frame is not None:
            cv2.imshow("Camera Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    transform_publisher.shutdown()
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()