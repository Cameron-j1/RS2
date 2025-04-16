import cv2
import numpy as np
import glob
import argparse
import os

def calibrate_camera(images_folder='folder', pattern_size=(7, 5), square_size=24.95, display_results=False):
    """
    Calibrate camera using images of a chessboard pattern.
    
    Args:
        images_folder: Path to folder containing calibration images
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
    
    # Get all images from the folder
    image_paths = glob.glob(os.path.join(images_folder, '*.jpg')) + \
                  glob.glob(os.path.join(images_folder, '*.png'))
    
    if not image_paths:
        raise ValueError(f"No images found in {images_folder}. Make sure they have .jpg or .png extensions.")
    
    # Process each image
    successful_images = 0
    image_size = None
    
    print(f"Found {len(image_paths)} images. Processing...")
    
    for i, image_path in enumerate(image_paths):
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if image_size is None:
            image_size = gray.shape[::-1]
        
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        # If found, add object points and image points
        if ret:
            successful_images += 1
            objpoints.append(objp)
            
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Display the corners
            if display_results:
                cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
                window_name = f"Chessboard Corners - {os.path.basename(image_path)}"
                cv2.imshow(window_name, img)
                cv2.waitKey(500)  # Display each image for 500ms
        else:
            print(f"Could not find chessboard corners in {os.path.basename(image_path)}")
    
    if display_results:
        cv2.destroyAllWindows()
    
    print(f"Successfully processed {successful_images}/{len(image_paths)} images")
    
    if successful_images == 0:
        raise ValueError("Could not find chessboard corners in any of the provided images")
    
    # Calibrate camera
    flags = 0
    # flags |= cv2.CALIB_FIX_ASPECT_RATIO
    # flags |= cv2.CALIB_ZERO_TANGENT_DIST
    
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags)
    
    # Calculate re-projection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    print(f"Re-projection error: {mean_error/len(objpoints)}")
    
    return camera_matrix, dist_coeffs, mean_error/len(objpoints)

def main():
    
    try:
        camera_matrix, dist_coeffs, rms_error = calibrate_camera()
        
        # Print calibration results
        print("\nCamera Intrinsic Matrix:")
        print(camera_matrix)
        print("\nDistortion Coefficients:")
        print(dist_coeffs)
        print(f"\nCalibration RMS Error: {rms_error}")
        
        # Save calibration results
        np.savez('output', 
                camera_matrix=camera_matrix, 
                dist_coeffs=dist_coeffs, 
                rms_error=rms_error)
        print(f"\nCalibration parameters saved to output")
        
    except Exception as e:
        print(f"Error during calibration: {e}")

if __name__ == "__main__":
    main()