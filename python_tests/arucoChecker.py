import cv2
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R

from pose_pub_class import CameraPosePublisher
import rclpy
from rclpy.node import Node

from image_sub_class import CameraSubscriber

from pose_sub_class import FrameListener  # Adjust if needed

#run vision with this: ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x15

#global variables
#distances from H1
aruco_board_span = 281/1000 #long distance
aruco_square_span = 36/1000 #short distance
H1_marker_ID = 5
aruco_to_H1 = {
    #aruco id: [x to H1 (m), y to H1 (m), rot z (rad)]
    1: [-aruco_board_span, -aruco_square_span, 0],
    2: [aruco_square_span, aruco_board_span, 0],
    3: [-aruco_board_span+30/1000, aruco_board_span, 0],
    4: [aruco_square_span, - aruco_square_span, 0]
}

def calculate_H1_T(T_base_to_marker, marker_ID):
    # print(f"ID using to calc H1: {marker_ID}")
    T_H1 = np.copy(T_base_to_marker)
    T_H1[0, 3] = T_H1[0, 3] + aruco_to_H1[marker_ID][0] #x position
    T_H1[1, 3] = T_H1[1, 3] + aruco_to_H1[marker_ID][1] #y position
    pos_in_base = T_H1[:3, 3]
    # print("H1 marker position in robot base frame:", np.round(pos_in_base, 5))
    return T_H1

def get_aruco_transforms(image, marker_size_mm=250):
    """
    Detect 6x6 ArUco markers in an image and return the 4x4 transformation matrices
    from camera to each detected marker.
    
    Args:
        image: OpenCV image (numpy array)
        marker_size_mm: Size of the ArUco marker in millimeters (default: 250mm)
        
    Returns:
        tuple: (transforms, ids)
            - transforms: numpy array of shape (n, 4, 4) containing the transformation matrices
            - ids: numpy array of shape (n,) containing the corresponding marker IDs
    """
    marker_size_mm = marker_size_mm/1000

    # Convert to grayscale if the image is colored
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Define the ArUco dictionary (6x6 markers)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    
    # Create ArUco parameters
    parameters = aruco.DetectorParameters()
    
    
    # Create detector
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect markers
    corners, ids, rejected = detector.detectMarkers(gray)
    
    # Initialize empty lists to store results
    transform_list = []
    id_list = []
    
    # Check if any markers were detected
    if ids is not None and len(ids) > 0:
        # cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Get camera matrix and distortion coefficients
        # Note: In a real application, you would load these from a calibration file
        # Here we're creating a simple estimate based on image dimensions
        height, width = gray.shape
        focal_length = width
        center = (width / 2, height / 2)
        #640x480
        camera_matrix = np.array([
            [608.69232,   0.        , 321.825],
            [  0.        , 607.304016, 242.8823],
            [  0.        ,   0.        ,   1.        ]
        ], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        #1280x720 intrinics
        camera_matrix = np.array([
            [904.31097901,   0.        , 641.57999223],
            [  0.        , 903.36941042, 362.54800006],
            [  0.        ,   0.        ,   1.        ]
        ])

        dist_coeffs = np.array([
            [ 4.44499317e-02],
            [ 5.56966659e-01],
            [ 2.99253301e-04],
            [-1.49486083e-03],
            [-2.25401697e+00]
        ])
        
        # For each detected marker
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i]
            
            # Estimate pose for this marker
            # Create object points for a square marker
            objPoints = np.array([
                [-marker_size_mm/2, marker_size_mm/2, 0],
                [marker_size_mm/2, marker_size_mm/2, 0],
                [marker_size_mm/2, -marker_size_mm/2, 0],
                [-marker_size_mm/2, -marker_size_mm/2, 0]
            ], dtype=np.float32)
            
            # Get marker corners and reshape for solvePnP
            imgPoints = marker_corners.reshape(4, 2)
            
            # Solve for pose
            success, rvec, tvec = cv2.solvePnP(
                objPoints, imgPoints, camera_matrix, dist_coeffs
            )
            
            if success:
                # Convert rotation vector to rotation matrix
                rot_matrix, _ = cv2.Rodrigues(rvec)
                
                # Create 4x4 transformation matrix
                transform = np.eye(4)
                transform[:3, :3] = rot_matrix
                transform[:3, 3] = tvec.flatten()
                
                # Add to lists
                transform_list.append(transform)
                id_list.append(marker_id)
    
    # Convert lists to numpy arrays
    if transform_list:
        transforms = np.array(transform_list)
        ids = np.array(id_list)
    else:
        # Return empty arrays with correct shapes if no markers detected
        transforms = None
        ids = None
    
    return transforms, ids



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

    # print(f"ee is X:{T_base_to_ee[0, 3]}, Y:{T_base_to_ee[1, 3]}, Z:{T_base_to_ee[2, 3]} from the base")

    T_ee_to_base = invert_transform(T_base_to_ee)

    # publisher.add_pose(T_base_to_ee, 123) #using 123 as a placeholder 

    #720p
    T_cam_to_ee = (np.array([
        [ 0.000338072668,  0.999999801432,  0.000329791286,  -0.037143913668],
        [-0.999998857238,  0.000411703512,  0.001278763264, -0.060065853878-0.008],
        [ 0.001278926362, -0.000328268593,  0.999999125353,  0.015976452427],
        [ 0.000000000000,  0.000000000000,  0.000000000000,  1.000000000000],
    ])) 

    # #from eye in hand calibration
    #480p extrinsics
    # T_cam_to_ee = (np.array([
    #     [ 0.000338072668,  0.999999801432,  0.000329791286,  -0.037143913668-0.015],
    #     [-0.999998857238,  0.000411703512,  0.001278763264, -0.060065853878-0.008],
    #     [ 0.001278926362, -0.000328268593,  0.999999125353,  0.015976452427],
    #     [ 0.000000000000,  0.000000000000,  0.000000000000,  1.000000000000],
    # ])) 

    tempRot = R.from_euler('z', 90, degrees=True).as_matrix()
    T_frame_adjust_z = np.eye(4)
    T_frame_adjust_z[:3, :3] = tempRot

    T_cam_to_ee = T_cam_to_ee @ T_frame_adjust_z #- this makes the axis on rviz for the camera line up

    #calulate camera pose from end effector and T_cam_to_ee
    T_cam = T_base_to_ee @ (T_cam_to_ee)
    # publisher.add_pose(T_cam, 124)
    # publisher.add_pose(np.eye(4), 6969)
    
    aruco_transforms = None
    aruco_transforms, ids = get_aruco_transforms(image, 33)
    # aruco_transforms, ids = get_aruco_transforms(image, 94.2)
    print(ids)
    # print(aruco_transforms)

    H1_published = False
    arucos_found = []
    aruco_Ts = []
    if aruco_transforms is not None:
        for i, transform in enumerate(aruco_transforms):
            # print(f'detected aruco X: {transform[0][3]}, Y: {transform[1][3]}, Z: {transform[2][3]}')
            T_marker_to_cam  = invert_transform(transform)
            T_cam_to_marker = transform
            # print(f"T_marker_to_cam X:{T_marker_to_cam[0, 3]}, Y:{T_marker_to_cam[1, 3]}, Z:{T_marker_to_cam[2, 3]}")
            # print(f"T_cam_to_marker X:{T_cam_to_marker[0, 3]}, Y:{T_cam_to_marker[1, 3]}, Z:{T_cam_to_marker[2, 3]}")

            T_base_to_marker = T_cam @ T_cam_to_marker
            pos_in_base = T_base_to_marker[:3, 3]
            if pos_in_base[2] < 0.1:
                publisher.add_pose(T_base_to_marker, ids[i]) 
                print("ArUco marker position in robot base frame:", np.round(pos_in_base, 5))
                arucos_found.append(ids[i])
                aruco_Ts.append(T_base_to_marker)
                try:
                    H1_T = calculate_H1_T(T_base_to_marker, ids[i])
                    if(H1_published == False):
                        publisher.add_pose(H1_T, H1_marker_ID, )
                        H1_published = True
                    pass
                except:
                    print('No saved transform between aruco and H1') 

        # Calculate Euclidean distance in X-Y plane between all pairs of ArUco markers
        print("\nEuclidean distances between markers (X-Y plane only):")
        for i in range(len(aruco_Ts)):
            for j in range(i+1, len(aruco_Ts)):
                # Extract X and Y coordinates from transformation matrices
                x1, y1 = aruco_Ts[i][0, 3], aruco_Ts[i][1, 3]
                x2, y2 = aruco_Ts[j][0, 3], aruco_Ts[j][1, 3]
                
                # Calculate Euclidean distance in X-Y plane only
                distance_xy = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                print(f"Distance between marker {arucos_found[i]} and marker {arucos_found[j]}: {distance_xy:.5f} units")


#start ROS crap
rclpy.init()

transform_publisher_node = Node('aruco_checker_node')

# Create publisher
transform_publisher = CameraPosePublisher(transform_publisher_node)
camera_subscriber = CameraSubscriber()
transform_subscriber = FrameListener(target_frame='tool0', source_frame='base_link')

prev_ee_T = 0

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
