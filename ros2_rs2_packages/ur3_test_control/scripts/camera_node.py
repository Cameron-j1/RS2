#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import operator
import sys
from collections import defaultdict
import cv2
import scipy.spatial as spatial
import scipy.cluster as clstr

class CameraNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Topic for the image stream
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.get_logger().info('Node started. Displaying image stream. Press Ctrl+C to stop.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (BGR)
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            chessBoard = self.process_chessboard(self.latest_image, False)
            chessStr = np.array2string(chessBoard, separator=', ')
            self.get_logger().info(f'ChessBoard:\n{chessStr}')
            # Display the image in a window
            if self.latest_image is not None:
                cv2.imshow('Camera Stream', self.latest_image)
                cv2.waitKey(1)  # Refresh the window (1ms delay)
            else:
                self.get_logger().warn('No image data received yet.')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def canny(self, img):
        # Maybe add some auto thresholding here
        edges = cv2.Canny(img, 80, 200)
        return edges


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
        See https://stackoverflow.com/a/383527/5087436
        """
        points = []
        for rho1, theta1 in h:
            for rho2, theta2 in v:
                A = np.array([
                    [np.cos(theta1), np.sin(theta1)],
                    [np.cos(theta2), np.sin(theta2)]
                ])
                b = np.array([[rho1], [rho2]])
                point = np.linalg.solve(A, b)
                point = int(np.round(point[0])), int(np.round(point[1]))
                points.append(point)
        return np.array(points)


    def cluster_intersections(self, points, max_dist=40):
        # I want to change this to kmeans
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

    def find_blue_dot(self, image):
            
        # Convert BGR to HSV color space (better for color detection), make sure image is an opencv image.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for blue color in HSV
        lower_blue = np.array([100, 150, 0])    # Lower bound (Hue, Saturation, Value)
        upper_blue = np.array([140, 255, 255])  # Upper bound
        
        # Create a mask for blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours are found
        if contours:
            # Find the largest contour (assuming it's the dot we want)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the center and radius of the minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            # Only consider it a dot if it's roughly circular and not too large
            if radius > 5 and radius < 50:  # Adjust these values based on your needs
                return True
            else:
                return False
        else:
            return False

    def find_chessboard_corners(self, points):
        """
        Code from https://medium.com/@neshpatel/solving-sudoku-part-ii-9a7019d196a2
        """
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
        Code from https://medium.com/@neshpatel/solving-sudoku-part-ii-9a7019d196a2
        """
        a = p2[0] - p1[0]
        b = p2[1] - p1[1]
        return np.sqrt((a ** 2) + (b ** 2))


    def warp_image(self, img, edges):
        """
        Code from https://medium.com/@neshpatel/solving-sudoku-part-ii-9a7019d196a2
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


    def cut_chessboard(self, img):
        side_len = int(img.shape[0] / 8)
        occupancy = np.zeros((8, 8))
        for i in range(8):
            for j in range(8):
                tile = img[i * side_len: (i + 1) * side_len, j * side_len: (j + 1) * side_len]
                if self.find_blue_dot(tile) == False:
                    occupancy[i][j] = 1    
        return occupancy


    def resize_image(self, img):
        """
        Resizes image to a maximum width of 800px
        """
        width = img.shape[1]
        if width > 800:
            scale = 800 / width
            return cv2.resize(img, None, fx=scale, fy=scale)
        else:
            return img


    def process_chessboard(self, src, debug=True):

        if src is None:
            sys.exit("There is no file with this path!")

        src = self.resize_image(src)
        src_copy = src.copy()

        # Convert to grayscale
        process = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        if debug:
            cv2.imshow("Grayscale", process)
            cv2.imwrite('grayscale.png', process)
            cv2.waitKey()
            cv2.destroyWindow("Grayscale")

        # Blur to remove disturbing things
        process = cv2.blur(process, (4, 4))

        if debug:
            cv2.imshow("Blur", process)
            cv2.imwrite('blur.png', process)
            cv2.waitKey()
            cv2.destroyWindow("Blur")

        # Use Canny Edge Detector https://en.wikipedia.org/wiki/Canny_edge_detector
        process = self.canny(process)

        if debug:
            cv2.imshow("Canny", process)
            cv2.imwrite('canny.png', process)
            cv2.waitKey()
            cv2.destroyWindow("Canny")

        # Dilate image (thicker lines)
        process = cv2.dilate(process, np.ones((3, 3), dtype=np.uint8))

        if debug:
            cv2.imshow("Dilate", process)
            cv2.imwrite('dilate.png', process)
            cv2.waitKey()
            cv2.destroyWindow("Dilate")

        # Use Hough transform to detect lines https://en.wikipedia.org/wiki/Hough_transform
        lines = self.hough_lines(process)

        # Sort lines by horizontal and vertical
        h, v = self.sort_lines(lines)

        if debug:
            self.render_lines(src_copy, h, (0, 255, 0))
            self.render_lines(src_copy, v, (0, 0, 255))
            cv2.imshow("Sorted lines", src_copy)
            cv2.imwrite('sorted-lines.png', src_copy)
            cv2.waitKey()
            cv2.destroyWindow("Sorted lines")

        if len(h) < 9 or len(v) < 9:
            print("There are not enough horizontal and vertical lines in this image. Try it anyway!")

        # Calculate intersections of the horizontal and vertical lines
        intersections = self.calculate_intersections(h, v)

        if debug:
            self.render_intersections(src_copy, intersections, (255, 0, 0), 1)
            cv2.imshow("Intersections", src_copy)
            cv2.imwrite('intersections.png', src_copy)
            cv2.waitKey()
            cv2.destroyWindow("Intersections")

        # Cluster intersection since there are many
        # Note that if it has more than 81 intersections, try to take a better pic of the chess board
        # without any environment artifacts
        clustered = self.cluster_intersections(intersections)
        if (len(clustered) > 81):
            print("Yo lol take a better pic asshole")
            return np.zeros((8, 8))

        if debug:
            src_copy = src.copy()
            self.render_intersections(src_copy, clustered, (0, 255, 0), 5, True)
            cv2.imshow("Clustered Intersections", src_copy)
            cv2.imwrite('clustered-intersections.png', src_copy)
            cv2.waitKey()
            cv2.destroyWindow("Clustered Intersections")

        if len(clustered) != 81:
            print("Something is wrong. There are " + str(len(intersections)) + " instead of 81 intersections.")

        # Find outer corners of the chessboard
        corners = self.find_chessboard_corners(clustered)

        if debug:
            src_copy = src.copy()
            self.render_intersections(src_copy, corners, (255, 0, 0), 5)
            cv2.imshow("Corners", src_copy)
            cv2.imwrite('corners.png', src_copy)
            cv2.waitKey()
            cv2.destroyWindow("Corners")

        # Warp and crop image
        dst = self.warp_image(src, corners)

        if debug:
            cv2.imshow("Warped", dst)
            cv2.imwrite('warped.png', dst)
            cv2.waitKey()
            cv2.destroyWindow("Warped")

        # Cut chessboard into 64 tiles
        return self.cut_chessboard(dst)


    def render_lines(self, img, lines, color):
        for rho, theta in lines:
            a = math.cos(theta)
            b = math.sin(theta)
            x0, y0 = a * rho, b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
            cv2.line(img, pt1, pt2, color, 1, cv2.LINE_AA)


    def render_intersections(self, img, points, color, size, slow=False):
        for point in points:
            cv2.circle(img, (int(point[0]), int(point[1])), 2, color, size)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Publisher Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()