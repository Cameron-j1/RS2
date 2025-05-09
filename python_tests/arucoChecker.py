import cv2
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

from pose_pub_class import CameraPosePublisher
import rclpy
from rclpy.node import Node

from image_sub_class import CameraSubscriber

from pose_sub_class import FrameListener  # Adjust if needed

#run vision with this: ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x15

#global variables
#distances from H1
aruco_board_span = 281/1000 + 22.75/1000 - 5/1000#long distance 22.75
aruco_square_span = 36/1000 + 22.75/1000 + 5/1000#short distance 22.75
H1_marker_ID = 5
aruco_to_H1 = {
    #aruco id: [x to H1 (m), y to H1 (m), rot z (rad)]
    # 1: [-aruco_board_span, -aruco_square_span, 0],
    2: [-0.2594, 0.2961, 0],
    3: [0.0144+0.006, 0.2961, 0],
    # 4: [aruco_square_span, - aruco_square_span, 0]
}

chess_board_side_ID_pairs = [
    (3, 2),
    #untested pairs only testing 2,3 for now
    # (3, 1),
    # (4, 1),
    # (2, 4)
]

# side_Req_rot = {
#     (2, 3): 0
#     (3, 1): (3/2)*math.pi
#     (4, 1): 0
#     (4, 2): (3/2)*math.pi
# }

def calculate_H1_T(T_base_to_marker, marker_ID):
    # print(f"ID using to calc H1: {marker_ID}")
    T_H1 = np.copy(T_base_to_marker)

    T_mod = np.eye(4)
    T_mod[0][3] = aruco_to_H1[marker_ID][0]
    T_mod[1][3] = aruco_to_H1[marker_ID][1]

    T_H1 = T_H1 @ T_mod

    pos_in_base = T_H1[:3, 3]
    print("H1 marker position in robot base frame:", np.round(pos_in_base, 5))
    return T_H1

def transform_calc_rot_matrix(yaw_calc_T1, yaw_calc_T2):
    # Extract X-Y positions from yaw_calc_T1 and yaw_calc_T2
    x1, y1 = yaw_calc_T1[0][3], yaw_calc_T1[1][3]
    x2, y2 = yaw_calc_T2[0][3], yaw_calc_T2[1][3]

    # Compute yaw angle in radians
    yaw = np.arctan2(y2 - y1, x2 - x1)

    yaw_deg = yaw * (180/math.pi)
    print(f"Z rotation of the chessboard in the global frame: {yaw_deg} (0 degrees for standard chessboard position)")

    # Construct a yaw-only rotation matrix (Z-axis rotation)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R_yaw = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw,  cos_yaw, 0],
        [0,        0,       1]
    ])

    return R_yaw

def average_T_pos(Transforms):
    #calculate the average position transformation matrix with 0 rotation
    avg_x = 0
    avg_y = 0
    avg_z = 0
    for transform in Transforms:
        avg_x = avg_x + transform[0][3]
        avg_y = avg_y + transform[1][3]
        avg_z = avg_z + transform[2][3]
    
    avg_T = np.eye(4)
    avg_T[0][3] = avg_x/max(1, len(Transforms))
    avg_T[1][3] = avg_y/max(1, len(Transforms))
    avg_T[2][3] = avg_z/max(1, len(Transforms))
    return avg_T



def get_aruco_transforms(image, marker_size_mm=250.0):
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
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    
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

        #1280x720 intrinics pre 9/05/25
        # camera_matrix = np.array([
        #     [904.31097901,   0.        , 641.57999223],
        #     [  0.        , 903.36941042, 362.54800006],
        #     [  0.        ,   0.        ,   1.        ]
        # ])

        # dist_coeffs = np.array([
        #     [ 4.44499317e-02],
        #     [ 5.56966659e-01],
        #     [ 2.99253301e-04],
        #     [-1.49486083e-03],
        #     [-2.25401697e+00]
        # ])

        #1280x720 intrinics post 9/05/25
        camera_matrix = np.array([
            [905.85707232, 0.0, 644.68513826],
            [0.0, 905.26944593, 357.56503199],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        dist_coeffs = np.array([
            [0.11422684],
            [-0.17355504],
            [-0.00203388],
            [0.00122792],
            [-0.17927661]
        ], dtype=np.float64)

        
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
    
    aruco_transforms = None
    # aruco_transforms, ids = get_aruco_transforms(image, 77.8)
    # aruco_transforms, ids = get_aruco_transforms(image, 22.5)
    aruco_transforms, ids = get_aruco_transforms(image, 47.9)
    print(ids)
    # print(aruco_transforms)

    arucos_found = []
    H1_transforms = []
    if aruco_transforms is not None:
        for i, transform in enumerate(aruco_transforms):
            # print(f'detected aruco X: {transform[0][3]}, Y: {transform[1][3]}, Z: {transform[2][3]}')
            T_marker_to_cam  = invert_transform(transform)
            T_cam_to_marker = transform
            # print(f"T_marker_to_cam X:{T_marker_to_cam[0, 3]}, Y:{T_marker_to_cam[1, 3]}, Z:{T_marker_to_cam[2, 3]}")
            # print(f"T_cam_to_marker X:{T_cam_to_marker[0, 3]}, Y:{T_cam_to_marker[1, 3]}, Z:{T_cam_to_marker[2, 3]}")

            T_base_to_marker = T_cam @ T_cam_to_marker
            pos_in_base = T_base_to_marker[:3, 3]
            # if pos_in_base[2] < 0.1: #and pos_in_base[2] > -0.05:
            publisher.add_pose(T_base_to_marker, ids[i]) 
            print("ArUco marker position in robot base frame:", np.round(pos_in_base, 5))
            temp_aruco_data_inner = [ids[i], T_base_to_marker]
            arucos_found.append(temp_aruco_data_inner)

            print(f"IDs pre filter: {ids}")
                

        # Calculate Euclidean distance in X-Y plane between all pairs of ArUco markers for debug purposes
        # print("\nEuclidean distances between markers (X-Y plane only):")
        # for i in range(len(arucos_found)):
            # for j in range(i+1, len(arucos_found)):
                # Extract X and Y coordinates from transformation matrices
                # x1, y1 = arucos_found[i][1][0, 3], arucos_found[i][1][1, 3]
                # x2, y2 = arucos_found[j][1][0, 3], arucos_found[j][1][1, 3]
                
                # Calculate Euclidean distance in X-Y plane only
                # distance_xy = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                # print(f"Distance between marker {arucos_found[i][0]} and marker {arucos_found[j][0]}: {distance_xy:.5f} units")

        #calculate the yaw of the chessboard
        yaw_calc_ID_pair = [0,0]
        ID_pair_exists = False
        for ID_pair in chess_board_side_ID_pairs:
            ID_pair_detected = [False, False]
            #loop through ids within the pair
            for i, marker_id in enumerate(ID_pair):
                #loop through all the ids of found arucos to see if they match the pair
                for aruco in arucos_found:
                    if aruco[0] == marker_id:
                        ID_pair_detected[i] = True

            #if you get a true for both of the pair you have a pair
            if ID_pair_detected == [True, True]:
                yaw_calc_ID_pair = ID_pair
                ID_pair_exists = True
                print("ID PAIR DETECTED")
                break #leave as we have found a pair
        
        if(ID_pair_exists):
            #find the indexes of the relevant aruco transforms
            yaw_t1_idx = -1
            yaw_t2_idx = -1
            #find the index's of the aruco you need for the side of the chessboard
            for i, aruco in enumerate(arucos_found):
                if aruco[0] == yaw_calc_ID_pair[0]:
                    yaw_t1_idx = i
                if aruco[0] == yaw_calc_ID_pair[1]:
                    yaw_t2_idx = i         
    
            if yaw_t1_idx != -1 and yaw_t2_idx != -1:
                #with the transforms you have (order inportant) calculate the rotation matrix of the board
                board_rot_matrix = transform_calc_rot_matrix(arucos_found[yaw_t1_idx][1], arucos_found[yaw_t2_idx][1])
                
                #calculate the aruco transforms with calculated rotation matrix
                for aruco in arucos_found:
                    aruco[1][:3, :3] = board_rot_matrix
                    #calculate the H1 position with aruco transform, id
                    try:
                        if(aruco[0] == 3):
                            H1_T = calculate_H1_T(aruco[1], aruco[0])
                            H1_transforms.append(H1_T)
                    except:
                        print(f'No saved transform between aruco ID: {aruco[0]} and H1')
                
                #contruct the final H1 position using the average position of all the calculated H1 position and the board rotation
                H1_T_xyz = average_T_pos(H1_transforms)
                H1_T_final = np.eye(4)
                H1_T_final[:3, :3] = board_rot_matrix
                H1_T_final[:3, 3] = H1_T_xyz[:3, 3]
                
                #publish the finalised H1 transform
                publisher.add_pose(H1_T_final, H1_marker_ID)

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
