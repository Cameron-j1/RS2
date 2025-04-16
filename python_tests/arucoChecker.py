import numpy as np
import cv2
import os

def detect_and_visualize_aruco(image_path, camera_matrix, dist_coeffs, marker_size=0.07):
    """
    Load an image, detect an ArUco marker, and visualize its coordinate axes.
    
    Args:
        image_path: Path to the image file
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        marker_size: Size of the ArUco marker in meters
    """
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        return
    
    # Create a copy of the image for visualization
    vis_image = image.copy()
    
    # Convert image to grayscale for ArUco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Set up ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect ArUco markers
    corners, ids, rejected = detector.detectMarkers(gray)
    
    # Draw detected markers on the image
    vis_image = cv2.aruco.drawDetectedMarkers(vis_image, corners, ids)
    
    if ids is not None and len(ids) > 0:
        print(f"Found {len(ids)} marker(s)")
        
        # Calculate half size for corner coordinates
        half_size = marker_size / 2
        
        for i in range(len(ids)):
            # Define object points based on marker size
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
                # Draw coordinate axes on the marker (x=red, y=green, z=blue)
                axis_length = marker_size * 1.5  # Make axis 1.5 times the marker size for visibility
                vis_image = cv2.drawFrameAxes(vis_image, camera_matrix, dist_coeffs, 
                                             rvec, tvec, axis_length)
                
                # Get the marker pose (rotation and translation)
                R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                
                # Create transformation matrix
                T_marker_to_cam = np.eye(4)
                T_marker_to_cam[:3, :3] = R_marker_to_cam
                T_marker_to_cam[:3, 3] = tvec.reshape(-1)
                
                print(f"\nMarker {ids[i][0]} position (x, y, z) in camera frame:")
                print(f"X: {tvec[0][0]:.4f} meters")
                print(f"Y: {tvec[1][0]:.4f} meters")
                print(f"Z: {tvec[2][0]:.4f} meters")
                
                # Print rotation as Euler angles for readability
                euler_angles = cv2.RQDecomp3x3(R_marker_to_cam)[0]
                print(f"\nMarker orientation (roll, pitch, yaw):")
                print(f"Roll: {np.rad2deg(euler_angles[0]):.2f} degrees")
                print(f"Pitch: {np.rad2deg(euler_angles[1]):.2f} degrees")
                print(f"Yaw: {np.rad2deg(euler_angles[2]):.2f} degrees")
                
                # Print the transformation matrix
                print("\nTransformation matrix (marker to camera):")
                print(T_marker_to_cam)
            else:
                print(f"Failed to estimate pose for marker {ids[i][0]}")
    else:
        print("No ArUco markers detected in the image")
    
    # Display image with visualized markers and axes
    cv2.imshow("ArUco Marker with Axes", vis_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save the visualization
    output_path = f"aruco_visualization_{os.path.basename(image_path)}"
    cv2.imwrite(output_path, vis_image)
    print(f"Visualization saved as {output_path}")
    
    return vis_image

def main():
    # Define the path to your image
    image_path = "/home/charles/git/RS2/python_tests/extrinsics_realsense/eyeHand1_Color.png"
    
    # Define camera calibration parameters
    # These should be replaced with your actual camera calibration values
    camera_matrix = np.array([
        [910.88428579,   0.0,         646.56956061],
        [0.0,           909.49046737, 357.94919134],
        [0.0,             0.0,           1.0]
    ])

    dist_coeffs = np.array([0.08300782, 0.09867322, -0.00272421, 0.0034501, -0.91121635])
    
    # Marker size in meters (7cm = 0.07m)
    marker_size = float(input("Enter the marker size in meters (default: 0.07): ") or 0.07)
    
    # Call the detection and visualization function
    detect_and_visualize_aruco(image_path, camera_matrix, dist_coeffs, marker_size)

if __name__ == "__main__":
    main()