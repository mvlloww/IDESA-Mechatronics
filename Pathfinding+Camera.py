def simplify_path(path):
    '''
    simplify_path function 

    :input: array of (i,j) grid coordinates from A* pathfinding algorithm
    :param path: delete any repeating outputs between two point on the grid. So the ball movement will be continuous instead of stopping every second.
    :return: cardinal_path (array: only changed horizontal and vertical), diagonaldown_path (array: horizontal, vertical and diagonal changes made) 
    '''
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

# # to be changed later with camera input 
# start_points=[(0,0)]
# end_points=[(19,39)]

'''
Initiate camera and set parameters
1. Load camera calibration file
2. Load ArUco dictionary and set detection parameters
3. Open camera and set resolution to 1280x720
4. Create window for camera feed
5. Create grid for pathfinding and set parameters
'''
# 1. Load the camera calibration file
Camera= np.load('Calibration.npz')
# Get the camera matrix
CM = Camera['CM']
# Get the distortion coefficients
dist_coef = Camera['dist_coef']

# 2. Load ArUco Disctionary 4x4_50 and set detection parameters
# Define the size of the aruco marker in mm
marker_size = 40
# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# Define the aruco detection parameters
parameters = aruco.DetectorParameters()

# 3. Opens camera and set resolution to 1280x720
# CAP_DSHOW to make sure it uses DirectShow backend on Windows (more stable)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# 4. Create window for camera feed
#Create window
cv2.namedWindow("frame-image", cv2.WINDOW_NORMAL)
#Position the window at x=0,y=100 on the computer screen
cv2.moveWindow("frame-image",0,100)

# 5. Create grid for pathfinding and set parameters
# Create a 20x40 grid all initialized to 1
grid = np.ones((20,40),np.int8)
# Detect set height and width of grid
grid_height, grid_width = grid.shape
# Define the 3D coordinates of the center of the marker in its own coordinate system
# Output marker center is at the corner of the marker so need this
half = marker_size / 2
center_3d = np.array([[half, half, 0]], dtype=np.float32)

# Precompute grid coordinates for masking
y_coords, x_coords = np.ogrid[:grid_height, :grid_width]
# Define the radius around the center of obstacle (can calibrate)
radius = 1

# Set target end points for pathfinding (can be changed later)
end_points = {1:[19,39], 2:[19,39], 3:[19,39], 4:[19,39], 5:[19,39]}

# Capture frame continuously
while(True):
    # Start the performance clock
    start = time.perf_counter()
    # Reset the 40x40 grid for this frame
    grid.fill(1)
    # Open a dictionary for sorting targets
    target_dict = {}

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

        # First pass: mark all markers as obstacles
        for i in range(len(ids)):
            center_2d, _ = cv2.projectPoints(center_3d, rvecs[i], tvecs[i], CM, dist_coef)
            center_pixel = center_2d[0][0].astype(int)
            img_h, img_w = frame.shape[:2]
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
            x, y = center_grid
            mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius**2
            grid[mask] = 0

        # Second pass: pathfind for each marker
        for i in range(len(ids)):
            center_2d, _ = cv2.projectPoints(center_3d, rvecs[i], tvecs[i], CM, dist_coef)
            center_pixel = center_2d[0][0].astype(int)
            img_h, img_w = frame.shape[:2]
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            start_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
            
            # Clear this marker from obstacles to use as start point
            x, y = start_grid
            mask = (x_coords - x)**2 + (y_coords - y)**2 <= radius**2
            grid[mask] = 1
            
            marker_id = int(ids[i][0])
            end_point = end_points.get(marker_id, None)
            print ('0')
            if end_point is not None:
                # tcod expects (row, col) pairs. start_grid is [grid_x, grid_y], so convert.
                start_pt = (int(start_grid[1]), int(start_grid[0]))  # (row, col)
                end_pt = (int(end_point[0]), int(end_point[1]))      # (row, col)
                print ('1')
                if start_pt != end_pt:
                    path = tcod.path.path2d(cost=grid, start_points=[start_pt], end_points=[end_pt], cardinal=10, diagonal=14)
                    print ('path',path)
                    num_path = path.shape[0]
                    target_dict[marker_id] = num_path
                    print(f"Marker {marker_id}: path_waypoints = {path}, shape = {path.shape}, num_path = {num_path}")
                    print ('2')
        
        # Sort dictionary by path length (ascending order)
        target_dict = dict(sorted(target_dict.items(), key=lambda item: item[1]))
        print("Sorted target dict:", target_dict)
        
        # Sequential pathfinding loop: process targets one by one
        target_ids = list(target_dict.keys())
        
        quit_flag = False
        for target_id in target_ids:
            if quit_flag:
                break
            
            # Recapture frame to ensure we have fresh data
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                
            # Get the marker's current center as start point
            marker_idx = None
            for i in range(len(ids)):
                if int(ids[i][0]) == target_id:
                    marker_idx = i
                    break
            
            if marker_idx is None:
                # Marker not visible, skip
                continue
            
            # Calculate pose for initial frame
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
            
            # Get end point from dictionary
            end_point = end_points.get(target_id, None)
            if end_point is None:
                continue
            end_pt = (int(end_point[0]), int(end_point[1]))  # (row, col)
            
            # Define distance threshold (in grid units)
            distance_threshold = 1.5
            
            # While loop to continuously navigate until reaching destination
            while True:
                # Recapture frame to get updated marker position
                ret, frame = cap.read()
                if not ret:
                    break
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                
                # Check if markers detected
                if ids is None or len(ids) == 0:
                    break
                
                # Find current marker
                marker_idx = None
                for i in range(len(ids)):
                    if int(ids[i][0]) == target_id:
                        marker_idx = i
                        break
                
                if marker_idx is None:
                    break
                
                # Recalculate pose for updated frame
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                
                # Compute current center position
                center_2d, _ = cv2.projectPoints(center_3d, rvecs[marker_idx], tvecs[marker_idx], CM, dist_coef)
                center_pixel = center_2d[0][0].astype(int)
                img_h, img_w = frame.shape[:2]
                grid_x = int(center_pixel[0] / img_w * grid_width)
                grid_y = int(center_pixel[1] / img_h * grid_height)
                start_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
                start_pt = (int(start_grid[1]), int(start_grid[0]))  # (row, col)
                
                # Calculate distance to end point
                distance = np.sqrt((start_pt[0] - end_pt[0])**2 + (start_pt[1] - end_pt[1])**2)
                
                # Check if reached destination
                if distance < distance_threshold:
                    print(f"Target {target_id} reached destination!")
                    break
                else:
                    # Compute path using tcod.path.path2d
                    path = tcod.path.path2d(cost=grid, start_points=[start_pt], end_points=[end_pt], cardinal=10, diagonal=14)
                    path_waypoints = np.asarray(path[1])
                    print(f"Processing target {target_id}: distance = {distance:.2f}, waypoints = {path_waypoints.shape[0]}")
                
                # Display frame
                cv2.imshow('frame-image', frame)
                if cv2.waitKey(20) & 0xFF == ord('q'):
                    quit_flag = True
                    break
        
        if quit_flag:
            break
    else:
        out = frame

    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # Stop the performance counter
    end = time.perf_counter()

    # If the button q is pressed in one of the windows 
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Print last sorted target_list before exiting
        print(target_dict)
        # Exit the While loop
        break
    
# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)
