# camera processing
# If see aruco code that set its coordinates + coordinates ard to 0 or else set to 1

# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# This is the ArUco library for fiducial markers
import cv2.aruco as aruco

import matplotlib.pyplot as plt


# Load the camera calibration file
Camera= np.load('Calibration.npz')
# Get the camera matrix
CM = Camera['CM']
# Get the distortion coefficients
dist_coef = Camera['dist_coef']

# Load ArUco Disctionary 4x4_50 and set detection parameters
# Define the size of the aruco marker in mm
marker_size = 40
# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# Define the aruco detection parameters
parameters = aruco.DetectorParameters()

# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
cap = cv2.VideoCapture(0)

# Set the width and heigth of the camera to 
cap.set(3,4000)
cap.set(4,4000)

#Create two opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_NORMAL)

#Position the windows next to eachother
cv2.moveWindow("frame-image",0,100)

#Create a 40x40 grid initialized to 1s
grid = np.ones((40,40),np.int8)
# Grid dimensions (used for mapping pixel coords -> grid indices)
grid_height, grid_width = grid.shape
# Define the 3D coordinates of the center of the marker in its own coordinate system
half = marker_size / 2
center_3d = np.array([[half, half, 0]], dtype=np.float32)
# Precompute grid coordinates for masking
y_coords, x_coords = np.ogrid[:grid_height, :grid_width]
# Define the radius around the center of obstacle (can calibrate)
radius = 1

# Execute this continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    # Reset the 40x40 grid for this frame
    grid.fill(1)

    # Capture current frame from the camera
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the grey image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  

    # If markers are detected, draw them, estimate pose and overlay axes
    if ids is not None and len(ids) > 0:
        out = aruco.drawDetectedMarkers(frame, corners, ids)

        # Calculate the pose of each detected marker
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
        

        for i in range(len(ids)):
            # Project the marker origin (0,0,0) into qimage pixels using the detected pose
            center_2d, _ = cv2.projectPoints(center_3d, rvecs[i], tvecs[i], CM, dist_coef)
            center_pixel = center_2d[0][0].astype(int)

            # Map pixel coordinates to grid (uses current frame size)
            img_h, img_w = frame.shape[:2]
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)

            # Draw circle around center point
            x, y = center_grid
            mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius**2
            grid[mask] = 0

    else:
        out = frame

    # Display the original frame in a window
    cv2.imshow('frame-image',frame)
    
    # Stop the performance counter
    end = time.perf_counter()

    # If the button q is pressed in one of the windows 
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Exit the While loop
        break
    
print (grid)
plt.imshow(grid, cmap="gray")
plt.colorbar()
plt.show()

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)