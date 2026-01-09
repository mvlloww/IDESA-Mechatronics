# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# This is the ArUco library for fiducial markers
import cv2.aruco as aruco
# This is a library for plotting and visualizing data
import matplotlib.pyplot as plt
# This is a library for roguelike game development (used for pathfinding)
import tcod

## Pathfinding setup ## (current location of ball)
start_points=[(0,0)]
end_points=[(39,39)]

# Function to simplify pathfinding -> ball do not move and stop at every grid cell
def simplify_path(path):
    cardinal_path = [path[0]]
    i = 1 #start with second waypoint

    ## deals with straight segments
    while i < len(path) - 1:
        n_i = abs(path[i+1][0] - path[i-1][0]) #compares next and previous waypoints i
        n_j = abs(path[i+1][1] - path[i-1][1]) #compares next and previous waypoints j
        n_ij = n_i*n_j #if there is a change, n_ij is non-zero

        if n_ij == 0 and (path[i][0] == path[i-1][0] or path[i][1] == path[i-1][1]):
            pass #skip waypoint
        else:
            cardinal_path.append(path[i])
        i += 1
    cardinal_path.append(path[-1])
    cardinal_path = np.array(cardinal_path) #convert to numpy array

    ## deals with falling diagonal segments
    diagonaldown_path = [cardinal_path[0]]
    i = 1
    while i < len(cardinal_path) - 1:
        c_current = cardinal_path[i]
        c_prev = cardinal_path[i-1]
        c_next = cardinal_path[i+1]
        c_change_i_prev = c_current[0] - c_prev[0]
        c_change_j_prev = c_current[1] - c_prev[1]
        c_change_i_next = c_current[0] - c_next[0]
        c_change_j_next = c_current[1] - c_next[1]

        n_i = c_change_i_prev + c_change_i_next
        n_j = c_change_j_prev + c_change_j_next

        if n_i == 0 and n_j == 0:
            pass #skip waypoint
        else:
            diagonaldown_path.append(cardinal_path[i])
        i += 1
    diagonaldown_path.append(cardinal_path[-1])

    diagonaldown_path = np.array(diagonaldown_path)
    
    ## deals with rising diagonal segments

    return cardinal_path, diagonaldown_path

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

## Camera setup ##

# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
# open camera with DirectShow (Windows)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# optional: check opened
if not cap.isOpened():
    raise RuntimeError("Camera not available (cap.isOpened() is False)")

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

# Capture frame continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    # Reset the 40x40 grid for this frame
    grid.fill(1)

    # Capture current frame from the camera
    ret, frame = cap.read()
    if not ret or frame is None or frame.size == 0:
        print("Warning: failed to read frame; attempting to reopen camera...")
        # small retry: try reopening up to 3 times
        cap.release()
        for _ in range(3):
            time.sleep(0.5)
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    break
        else:
            print("Error: unable to recover camera; skipping this loop iteration")
            continue

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

    # Implement pathfinging function 
    path = tcod.path.path2d(cost = grid, start_points=start_points, end_points=end_points, cardinal=10, diagonal=14)
    # Simplify path to reduce waypoints
    cardinal_path, diagonaldown_path = simplify_path(path)
    
    # Display pathfinding results
    display_simple_grid = grid.astype(int)
    display_simple_grid[tuple(np.array(path).T)] = 1
    display_simple_grid[tuple(np.array(diagonaldown_path).T)] = 0.5
    display_simple_grid[tuple(np.array(start_points).T)] = 0.75
    display_simple_grid[tuple(np.array(end_points).T)] = 0.75
    # print('simplified instructions:')
    # print(display_simple_grid)
    # plt.imshow(grid, cmap="gray")
    # plt.colorbar()
    # plt.show()

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
plt.show(1)

print('simplified instructions:')
print(display_simple_grid)
plt.imshow(grid, cmap="gray")
plt.colorbar()
plt.show(2)

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)
