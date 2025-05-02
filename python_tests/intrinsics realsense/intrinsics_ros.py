import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Note: topic name should match exactly
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = threading.Lock()
        
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            
    def get_image(self):
        with self.lock:
            return self.latest_image.copy() if self.latest_image is not None else None

def calibrate_camera(camera_node, num_images=10, pattern_size=(7, 5), square_size=24.95, display_results=False):
    """
    Calibrate camera using images captured from ROS topic when prompted.
    
    Args:
        camera_node: CameraSubscriber node instance
        num_images: Number of images to capture for calibration
        pattern_size: Tuple of (width, height) - inner corners of the chessboard pattern
        square_size: Size of a square in the chessboard (any unit)
        display_results: Whether to display detected chessboard corners
    
    Returns:
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        rms_error: Re-projection error
    """
    # Prepare object points: (0,0,0), (1,0,0), (2,0,0) ... (8,5,0)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    successful_images = 0
    image_size = None
    
    print(f"We will capture {num_images} images for calibration.")
    print("Press 'c' to capture an image when the chessboard is visible.")
    print("Press ESC to abort the calibration process.")
    
    # Spin ROS node a few times to make sure we're receiving images
    for _ in range(10):
        rclpy.spin_once(camera_node, timeout_sec=0.1)
    
    captured_images = []
    
    # Capture loop
    while successful_images < num_images:
        # Show the current camera view
        rclpy.spin_once(camera_node, timeout_sec=0.1)
        frame = camera_node.get_image()
        
        if frame is not None:
            # Display the frame
            cv2.imshow("Camera View - Press 'c' to capture, ESC to abort", frame)
            key = cv2.waitKey(100) & 0xFF
            
            # Check for keyboard input
            if key == ord('c'):  # 'c' key for capture
                print(f"\nCapturing image {successful_images + 1}/{num_images}")
                
                # Process the captured frame
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                if image_size is None:
                    image_size = gray.shape[::-1]
                
                # Find the chessboard corners
                ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
                
                if ret:
                    # Add object points and image points
                    objpoints.append(objp)
                    
                    # Refine corner positions
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    imgpoints.append(corners2)
                    
                    # Display the corners
                    if display_results:
                        corner_img = frame.copy()
                        cv2.drawChessboardCorners(corner_img, pattern_size, corners2, ret)
                        cv2.imshow("Detected Corners", corner_img)
                        cv2.waitKey(500)
                    
                    successful_images += 1
                    captured_images.append(frame)
                    print(f"Success! Found chessboard corners. {successful_images}/{num_images} images captured.")
                else:
                    print("Could not find chessboard corners in this image. Please try again.")
            
            elif key == 27:  # ESC key
                print("Calibration aborted by user.")
                cv2.destroyAllWindows()
                return None, None, None
    
    cv2.destroyAllWindows()
    
    if successful_images == 0:
        raise ValueError("Could not find chessboard corners in any of the captured images")
    
    print(f"Successfully captured {successful_images} images with chessboard corners")
    
    # Optional: Save the captured images
    if not os.path.exists("captured_calibration_images"):
        os.makedirs("captured_calibration_images")
    
    for i, img in enumerate(captured_images):
        cv2.imwrite(f"captured_calibration_images/calib_img_{i:02d}.jpg", img)
    
    # Calibrate camera
    flags = 0
    # flags |= cv2.CALIB_FIX_ASPECT_RATIO
    # flags |= cv2.CALIB_ZERO_TANGENT_DIST
    
    # Initialize camera matrix and distortion coefficients
    camera_matrix = np.eye(3, 3)
    dist_coeffs = np.zeros((5, 1))
    
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, camera_matrix, dist_coeffs, flags=flags)
    
    # Calculate re-projection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    rms_error = mean_error/len(objpoints)
    print(f"Re-projection error: {rms_error}")
    
    return camera_matrix, dist_coeffs, rms_error

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create camera subscriber node
    camera_node = CameraSubscriber()
    
    try:
        # Wait for the camera to start publishing
        print("Waiting for camera images to be available...")
        while rclpy.ok():
            rclpy.spin_once(camera_node, timeout_sec=0.1)
            if camera_node.get_image() is not None:
                print("Camera is ready!")
                break
            time.sleep(0.1)
        
        # Start the calibration process
        print("\nStarting camera calibration...")
        camera_matrix, dist_coeffs, rms_error = calibrate_camera(
            camera_node=camera_node,
            num_images=15,  # Adjust as needed
            pattern_size=(7, 5),
            square_size=24.95,
            display_results=True
        )
        
        if camera_matrix is not None:
            # Print calibration results
            print("\nCamera Intrinsic Matrix:")
            print(camera_matrix)
            print("\nDistortion Coefficients:")
            print(dist_coeffs)
            print(f"\nCalibration RMS Error: {rms_error}")
            
            # Save calibration results
            np.savez('calibration_output', 
                    camera_matrix=camera_matrix, 
                    dist_coeffs=dist_coeffs, 
                    rms_error=rms_error)
            print(f"\nCalibration parameters saved to calibration_output.npz")
        
    except Exception as e:
        print(f"Error during calibration: {e}")
    finally:
        # Clean up
        camera_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()