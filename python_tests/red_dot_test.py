import cv2
import numpy as np

# --- Load image ---
image = cv2.imread('input.jpg')  # Replace with your file path
if image is None:
    raise ValueError("Image not found or invalid path")

# --- Convert to HSV color space ---
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# --- Threshold for red in HSV (both lower and upper red ranges) ---
# Lower red hue range
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

# Upper red hue range
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])
mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

# Combine the two masks
red_mask = cv2.bitwise_or(mask1, mask2)

# --- Morphological operations to accentuate red blobs ---
# Use a circular kernel roughly the expected dot size
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
closed = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
clean = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

# --- Find contours (potential red dots) ---
contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

found = False
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 50:  # You can adjust this threshold based on expected size
        found = True
        # Draw the detected contour on the original image
        cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)

if found:
    print("Red dot detected.")
else:
    print("No red dot detected.")

# --- Display result ---
cv2.imshow("Detected Red Dot", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
