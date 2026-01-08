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
Camera= np.load('Sample_Calibration.npz')
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
cap.set(3,1080)
cap.set(4,1080)

#Create two opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

#Position the windows next to eachother
cv2.moveWindow("frame-image",0,100)

#Create an empty matrix of ones of size 40x40 for later use
grid = np.ones((40,40))

# Execute this continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    
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
        
        border = 3
        for i in range(len(ids)):
            # Project the marker origin (0,0,0) into image pixels using the detected pose
            center_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
            center_2d, _ = cv2.projectPoints(center_3d, rvecs[i], tvecs[i], CM, dist_coef)
            center_pixel = center_2d[0][0].astype(int)
            # Debug: uncomment to print projected pixel and grid coords
            # print(f"id={ids[i][0]}, center_pixel={center_pixel}")

            # Scale to your 40x40 grid and clamp to valid indices (0..39)
            center_grid = np.clip((center_pixel / 27).astype(int), 0, 39)

            # Draw circle around center point
            x, y = center_grid
            radius = border
            y_coords, x_coords = np.ogrid[:40, :40]
            mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius**2
            grid[mask] = 0

        # # Draw the axis for each marker and print the transformation vectors
        # for i in range(len(ids)):
        #     # rvecs and tvecs are returned as arrays of shape (N,1,3), take the first row
        #     rvec = rvecs[i][0]
        #     tvec = tvecs[i][0]

        #     # set the area of the detected marker in the grid to white (1)
        #     corner = corners[i][0]
        #     corner_grid = corner/27
        #     int_corners = corner_grid.astype(int)
        #     grid [int_corners[0][1]:int_corners[2][1], int_corners[0][0]:int_corners[2][0]] = 0

        #     #cv2.drawFrameAxes(out, CM, dist_coef, rvec, tvec, 50)
        #     #cv2.fillConvexPoly(grid, int_corners, 0) 
        #     #print("Id: ", int(ids[i][0]), "rvecs", rvec[1], " tvecs: ", tvec[1])
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