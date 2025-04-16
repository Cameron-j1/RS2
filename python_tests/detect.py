import math
import operator
import sys
from collections import defaultdict

import numpy as np
import time
import cv2

import scipy.spatial as spatial
import scipy.cluster as clstr

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ChessboardProcessor(Node):
    def __init__(self):
        super().__init__('chessboard_processor')
        
        # Create a subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Initialize CV bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Set output path
        self.output_path = ""  # Set your desired output path here
        self.output_prefix = ""
        self.debug = True
        
        self.get_logger().info('Chessboard processor initialized. Waiting for images...')

    def image_callback(self, msg):
        self.get_logger().info('Received image. Processing...')
        
        try:
            # Convert ROS Image message to OpenCV image
            src = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image
            self.process_chessboard(src)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_chessboard(self, src):
        if src is None:
            self.get_logger().error("Invalid image!")
            return

        src = cv2.resize(src, (1000, 1000))
        src_copy = src.copy()

        # Convert to grayscale
        process = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        if self.debug:
            cv2.imshow("Grayscale", process)
            cv2.waitKey(1)

        # Blur to remove disturbing things
        process = cv2.blur(process, (4, 4))

        if self.debug:
            cv2.imshow("Blur", process)
            cv2.waitKey(1)

        # Use Canny Edge Detector
        process = cv2.Canny(process, 25, 75)

        if self.debug:
            cv2.imshow("Canny", process)
            cv2.waitKey(1)

        # Dilate image (thicker lines)
        process = cv2.dilate(process, np.ones((3, 3), dtype=np.uint8))

        if self.debug:
            cv2.imshow("Dilate", process)
            cv2.waitKey(1)

        # Use Hough transform to detect lines
        lines = self.hough_lines(process)
        
        if lines is None:
            self.get_logger().warn("No lines detected in the image")
            return

        # Sort lines by horizontal and vertical
        h, v = self.sort_lines(lines)

        if self.debug:
            src_line_viz = src_copy.copy()
            self.render_lines(src_line_viz, h, (0, 255, 0))
            self.render_lines(src_line_viz, v, (0, 0, 255))
            cv2.imshow("Sorted lines", src_line_viz)
            cv2.waitKey(1)

        if len(h) < 9 or len(v) < 9:
            self.get_logger().warn(f"Not enough lines detected: {len(h)} horizontal, {len(v)} vertical")
            return

        # Calculate intersections of the horizontal and vertical lines
        intersections = self.calculate_intersections(h, v)

        if self.debug:
            src_int_viz = src_copy.copy()
            self.render_intersections(src_int_viz, intersections, (255, 0, 0), 1)
            cv2.imshow("Intersections", src_int_viz)
            cv2.waitKey(1)

        # Cluster intersection since there are many
        clustered = self.cluster_intersections(intersections)

        if self.debug:
            src_cluster_viz = src_copy.copy()
            self.render_intersections(src_cluster_viz, clustered, (0, 255, 0), 5, True)
            cv2.imshow("Clustered Intersections", src_cluster_viz)
            cv2.waitKey(1)

        if len(clustered) != 81:
            self.get_logger().warn(f"Unexpected number of intersections: {len(clustered)} (expected 81)")

        # Find outer corners of the chessboard
        corners = self.find_chessboard_corners(clustered)

        if self.debug:
            src_corner_viz = src_copy.copy()
            self.render_intersections(src_corner_viz, corners, (255, 0, 0), 5)
            cv2.imshow("Corners", src_corner_viz)
            cv2.waitKey(1)

        # Warp and crop image
        dst = self.warp_image(src, corners)

        if self.debug:
            cv2.imshow("Warped", dst)
            cv2.waitKey(1)

        # Cut chessboard into 64 tiles
        self.cut_chessboard(dst, self.output_path, self.output_prefix)
        
        self.get_logger().info("Processing complete")

    def hough_lines(self, img):
        rho, theta, thresh = 2, np.pi / 180, 600
        return cv2.HoughLines(img, rho, theta, thresh)

    def sort_lines(self, lines):
        """
        Sorts lines by horizontal and vertical
        """
        h = []
        v = []
        for i in range(lines.shape[0]):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            if theta < np.pi / 4 or theta > np.pi - np.pi / 4:
                v.append([rho, theta])
            else:
                h.append([rho, theta])
        return h, v

    def calculate_intersections(self, h, v):
        """
        Finds the intersection of two lines given in Hesse normal form.
        """
        points = []
        for rho1, theta1 in h:
            for rho2, theta2 in v:
                A = np.array([
                    [np.cos(theta1), np.sin(theta1)],
                    [np.cos(theta2), np.sin(theta2)]
                ])
                b = np.array([[rho1], [rho2]])
                try:
                    point = np.linalg.solve(A, b)
                    point = int(np.round(point[0])), int(np.round(point[1]))
                    points.append(point)
                except np.linalg.LinAlgError:
                    continue
        return np.array(points)

    def cluster_intersections(self, points, max_dist=40):
        if len(points) == 0:
            return []
            
        Y = spatial.distance.pdist(points)
        Z = clstr.hierarchy.single(Y)
        T = clstr.hierarchy.fcluster(Z, max_dist, 'distance')
        clusters = defaultdict(list)
        for i in range(len(T)):
            clusters[T[i]].append(points[i])
        clusters = clusters.values()
        clusters = map(lambda arr: (np.mean(np.array(arr)[:, 0]), np.mean(np.array(arr)[:, 1])), clusters)

        result = []
        for point in clusters:
            result.append([point[0], point[1]])
        return result

    def find_dot(self, image):
        # Convert BGR to HSV color space (better for color detection)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 150, 10])    # Lower range for red
        upper_red1 = np.array([20, 255, 255])  

        lower_red2 = np.array([160, 150, 50])  # Upper range for red (due to hue wrapping)
        upper_red2 = np.array([180, 255, 255])

        # Create two masks for the red color using the defined ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine the two masks to capture the full red range
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the combined mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming it's the dot we want)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the center and radius of the minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            # Only consider it a dot if it's roughly circular and within size limits
            if 0.5 < radius < 5:  # Adjust these values based on your needs
                if self.debug:
                    # Draw circle on the original image
                    img_copy = image.copy()
                    cv2.circle(img_copy, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.imshow('Red Dot Found', img_copy)
                    cv2.waitKey(1)
                
                return True
            else:
                return False
            
        else:
            return False

    def find_chessboard_corners(self, points):
        """
        Find the corners of the chessboard
        """
        if len(points) < 4:
            # Return dummy corners if we don't have enough points
            self.get_logger().error("Not enough points to find chessboard corners")
            return [[0, 0], [100, 0], [0, 100], [100, 100]]
            
        # Bottom-right point has the largest (x + y) value
        # Top-left has point smallest (x + y) value
        # Bottom-left point has smallest (x - y) value
        # Top-right point has largest (x - y) value
        bottom_right, _ = max(enumerate([pt[0] + pt[1] for pt in points]), key=operator.itemgetter(1))
        top_left, _ = min(enumerate([pt[0] + pt[1] for pt in points]), key=operator.itemgetter(1))
        bottom_left, _ = min(enumerate([pt[0] - pt[1] for pt in points]), key=operator.itemgetter(1))
        top_right, _ = max(enumerate([pt[0] - pt[1] for pt in points]), key=operator.itemgetter(1))
        
        return [points[top_left], points[top_right], points[bottom_left], points[bottom_right]]

    def distance_between(self, p1, p2):
        """
        Calculate Euclidean distance between two points
        """
        a = p2[0] - p1[0]
        b = p2[1] - p1[1]
        return np.sqrt((a ** 2) + (b ** 2))

    def warp_image(self, img, edges):
        """
        Warp the image to get a square chessboard
        """
        top_left, top_right, bottom_left, bottom_right = edges[0], edges[1], edges[2], edges[3]

        # Explicitly set the data type to float32 or 'getPerspectiveTransform' will throw an error
        warp_src = np.array([top_left, top_right, bottom_right, bottom_left], dtype='float32')

        side = max([
            self.distance_between(bottom_right, top_right),
            self.distance_between(top_left, bottom_left),
            self.distance_between(bottom_right, bottom_left),
            self.distance_between(top_left, top_right)
        ])

        # Describe a square with side of the calculated length, this is the new perspective we want to warp to
        warp_dst = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype='float32')

        # Gets the transformation matrix for skewing the image to fit a square by comparing the 4 before and after points
        m = cv2.getPerspectiveTransform(warp_src, warp_dst)

        # Performs the transformation on the original image
        return cv2.warpPerspective(img, m, (int(side), int(side)))

    def cut_chessboard(self, img, output_path, output_prefix=""):
        """
        Cut the chessboard into 64 tiles and detect if they're empty or occupied
        """
        side_len = int(img.shape[0] / 8)
        timestamp = int(time.time())
        
        for i in range(8):
            for j in range(8):
                tile = img[i * side_len: (i + 1) * side_len, j * side_len: (j + 1) * side_len]
                
                # Check if tile is empty or occupied
                is_empty = self.find_dot(tile)
                
                # Save the tile
                if output_path:
                    status = "empty" if is_empty else "occupied"
                    filename = f"{output_path}{output_prefix}-{j + i * 8}-{status}-{timestamp}.jpg"
                    cv2.imwrite(filename, tile)
                
                # Display the tile
                if self.debug:
                    status_text = "Empty" if is_empty else "Occupied"
                    tile_copy = tile.copy()
                    cv2.putText(tile_copy, status_text, (10, 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow(f"Tile {j + i * 8}", tile_copy)
                    cv2.waitKey(1)

    def render_lines(self, img, lines, color):
        """
        Helper function to render lines
        """
        for rho, theta in lines:
            a = math.cos(theta)
            b = math.sin(theta)
            x0, y0 = a * rho, b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
            cv2.line(img, pt1, pt2, color, 1, cv2.LINE_AA)

    def render_intersections(self, img, points, color, size, slow=False):
        """
        Helper function to render intersection points
        """
        for point in points:
            cv2.circle(img, (int(point[0]), int(point[1])), 2, color, size)
            if slow:
                cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    
    chessboard_processor = ChessboardProcessor()
    
    try:
        rclpy.spin(chessboard_processor)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        chessboard_processor.destroy_node()
        rclpy.shutdown()
        # Close all OpenCV windows
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()