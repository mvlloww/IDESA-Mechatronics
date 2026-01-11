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
def get_center(center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height):
    '''
    get center of ArUco code function

    input: 3D center coordinates of ArUco code, rotation and translation vectors, camera matrix, distortion coefficients, current frame, grid width and height
    output: x and y coordinates on grid of the center of the ArUco code
    '''
    center_2d, _ = cv2.projectPoints(center_3d, rvecs, tvecs, CM, dist_coef)
    center_pixel = center_2d[0][0].astype(int)
    img_h, img_w = frame.shape[:2]
    grid_x = int(center_pixel[0] / img_w * grid_width)
    grid_y = int(center_pixel[1] / img_h * grid_height)
    center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
    x, y = center_grid
    
    return x, y
def grid_obstacle(ids, target_id, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height):
    '''
    generate obstacles on grid function

    input: detected ArUco ids, target id to ignore, x/y coordinate arrays of grid, current grid, obstacle radius
    output: updated grid with obstacles marked for all non-target markers
    '''
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id != int(target_id):
            center_2d, _ = cv2.projectPoints(center_3d, rvecs[i], tvecs[i], CM, dist_coef)
            center_pixel = center_2d[0][0].astype(int)
            img_h, img_w = frame.shape[:2]
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
            ox, oy = center_grid

            mask = (x_coords - ox)**2 + (y_coords - oy)**2 <= radius**2
            grid[mask] = 0
    return grid
def rotate_dict(d, k=2):
    items = list(d.items())
    k = k % len(items)   # safety for large shifts
    rotated = items[-k:] + items[:-k]
    return dict(rotated)

''' Import necessary libraries '''
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
import os
import numpy as np
'''Initiate camera parameters'''
# 1. Load the camera calibration file
script_dir = os.path.dirname(os.path.abspath(__file__))
calib_path = os.path.join(script_dir, 'Calibration.npz')
if not os.path.exists(calib_path):
    raise FileNotFoundError(f"Calibration file not found at: {calib_path}")
Camera = np.load(calib_path)
# Get the camera matrix
CM = Camera['CM']
# Get the distortion coefficients
dist_coef = Camera['dist_coef']

# Define the size of the aruco marker in mm
marker_size = 40
# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# Define the aruco detection parameters
parameters = aruco.DetectorParameters()

# CAP_DSHOW to make sure it uses DirectShow backend on Windows (more stable)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#Create window
cv2.namedWindow("frame-image", cv2.WINDOW_NORMAL)
#Position the window at x=0,y=100 on the computer screen
cv2.moveWindow("frame-image",0,100)

'''Initiate grid for pathfinding'''
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
end_points = {1:[10,39], 2:[11,39], 3:[12,39], 4:[13,39]}
# Set ball ArUco id
ball_id = 5
# Create id_buffer dictionary
id_buffer = {}

'''Temperary variables'''
IsFire = False
quit_flag = False

''' Main loop '''
while True:
    # Start the performance clock
    start = time.perf_counter()

    while IsFire == False and not quit_flag:
        # Capture current frame from the camera
        ret, frame = cap.read()

        ## check IsFire status here##

        # Convert the image from the camera to Gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers in the grey image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  
        cv2.imshow('frame-image', frame)
        
        # Check for 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit_flag = True
            break

        # If markers are detected, draw them, estimate pose and overlay axes
        if ids is not None and len(ids) > 0:
            # Draw detected markers on the frame
            out = aruco.drawDetectedMarkers(frame, corners, ids)
            # Calculate the pose of each detected marker
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)

            # Reset grid for pathfinding
            grid.fill(1)
            # Open dictionary for sorting targets 
            target_dict = {}

            for t in range(len(ids)):
                marker_id = int(ids[t][0])
                # Skip markers without defined endpoints (e.g., ball marker)
                if marker_id not in end_points:
                    continue
                # Update grid with obstacles for all non-target markers
                grid = grid_obstacle(ids, marker_id, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height)
                # Get center of target ArUco code on grid
                start_point = get_center(center_3d, rvecs[t], tvecs[t], CM, dist_coef, frame, grid_width, grid_height)
                print("Start Point for ID ", marker_id, ": ", start_point)

                # append start points and id[t] to id_buffer
                if marker_id not in id_buffer:
                    id_buffer[marker_id] = start_point

                # Get path from target to end point and ball to target
                # tcod.path.path2d expects sequences of (i,j) pairs
                sp = (int(start_point[1]), int(start_point[0])) # Note the (y,x) order for (i,j)
                target_endpoint = end_points.get(marker_id)
                if target_endpoint is not None:
                    ep = (int(target_endpoint[0]), int(target_endpoint[1]))
                    path_t2e = tcod.path.path2d(cost=grid, start_points=[sp], end_points=[ep], cardinal=10, diagonal=14)
                    ball_idx = np.where(ids == ball_id)[0]
                    if ball_idx.size > 0:
                        ball_center = get_center(center_3d, rvecs[ball_idx[0]], tvecs[ball_idx[0]], CM, dist_coef, frame, grid_width, grid_height)
                        bs = (int(ball_center[1]), int(ball_center[0]))
                        path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[sp], cardinal=10, diagonal=14)
                    else:
                        path_b2t = []
                    total_path = np.concatenate((path_b2t, path_t2e)) if len(path_b2t) > 0 else path_t2e
                    # append total path to target_dict 
                    target_dict[marker_id] = len(total_path)
            # Sort target_dict based on path length
            sorted_targets = sorted(target_dict.items(), key=lambda item: item[1])
            print("Sorted Targets based on path length: ", sorted_targets) 

            for keys, _len in sorted_targets:
                position_status = False
                while position_status == False and not quit_flag:
                    ret, frame = cap.read()

                    ## check IsFire status here##

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                    
                    # Check for 'q' key press
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        quit_flag = True
                        break
                    if ids is not None and len(ids) > 0 and keys in ids.flatten() and ball_id in ids.flatten():
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        cv2.imshow('frame-image', frame)
                        if end_points.get(keys) == get_center(center_3d, rvecs[np.where(ids==keys)[0][0]], tvecs[np.where(ids==keys)[0][0]], CM, dist_coef, frame, grid_width, grid_height):
                            position_status = True
                            print ("Target ID ", keys, " reached endpoint. switching to next target.")

                        # compare current position with buffered position
                        for i in range(len(ids)):
                            if ids[i][0] != keys:
                                # Recompute obstacles for current positions of non-target markers
                                grid = grid_obstacle(ids, keys, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height)
                                print ("Updated grid with obstacles for non-target markers.")

                        # ball to target
                        ball_idx = np.where(ids == ball_id)[0]
                        key_idx = np.where(ids == keys)[0]
                        if ball_idx.size > 0 and key_idx.size > 0:
                            ball_start = get_center(center_3d, rvecs[ball_idx[0]], tvecs[ball_idx[0]], CM, dist_coef, frame, grid_width, grid_height)
                            target_start = get_center(center_3d, rvecs[key_idx[0]], tvecs[key_idx[0]], CM, dist_coef, frame, grid_width, grid_height)
                            if abs(ball_start[0] - target_start[0]) > 2 and abs(ball_start[1] - target_start[1]) > 2:
                                bs = (int(ball_start[1]), int(ball_start[0]))
                                ts = (int(target_start[1]), int(target_start[0]))
                                path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[ts], cardinal=10, diagonal=14)
                                _, diagonaldown_path_b2t = simplify_path(path_b2t)
                                print('target id', keys, "path_b2t:", diagonaldown_path_b2t)

                                # Visualize path on frame
                                # Use float so fractional values (0.5, 0.75) are preserved
                                display_simple_grid = grid.astype(float)
                                # Safe assignments: only assign if the path arrays are non-empty and within bounds
                                if len(path_b2t) > 0:
                                    idx = tuple(np.array(path_b2t).T)
                                    display_simple_grid[idx] = 0.25
                                if len(diagonaldown_path_b2t) > 0:
                                    idx = tuple(np.array(diagonaldown_path_b2t).T)
                                    display_simple_grid[idx] = 0.5
                                # Highlight current ball and target cells
                                display_simple_grid[bs] = 1
                                display_simple_grid[ts] = 0.75
                                
                                fig, ax = plt.subplots(figsize=(4, 4), dpi=100)
                                ax.imshow(display_simple_grid, cmap="gray", vmin=0, vmax=1)
                                ax.set_title("Grid & Path")
                                ax.axis("off")

                                # Render the Matplotlib figure
                                fig.canvas.draw()
                                # Get RGBA buffer from the figure
                                buf = fig.canvas.buffer_rgba()
                                # Convert buffer to NumPy array
                                plot = np.asarray(buf)
                                # Convert RGBA/RGB → BGR for OpenCV
                                plot = cv2.cvtColor(plot, cv2.COLOR_RGB2BGR)
                                plt.close(fig)  # IMPORTANT: prevent memory leak

                                h, w = plot.shape[:2]
                                frame[0:h, 0:w] = plot
                                cv2.imshow('frame-image', frame)

                            else:
                                ts = (int(target_start[1]), int(target_start[0]))
                                ep = (int(end_points.get(keys)[0]), int(end_points.get(keys)[1]))
                                path_t2e = tcod.path.path2d(cost=grid, start_points=[ts], end_points=[ep], cardinal=10, diagonal=14)
                                _, diagonaldown_path_t2e = simplify_path(path_t2e)
                                print('target id', keys, "path_t2e:", diagonaldown_path_t2e)

                                # Use float so fractional values (0.5, 0.75) are preserved
                                display_simple_grid = grid.astype(float)
                                # Safe assignments: only assign if the path arrays are non-empty and within bounds
                                if len(path_t2e) > 0:
                                    idx = tuple(np.array(path_t2e).T)
                                    display_simple_grid[idx] = 0.25
                                if len(diagonaldown_path_t2e) > 0:
                                    idx = tuple(np.array(diagonaldown_path_t2e).T)
                                    display_simple_grid[idx] = 0.5
                                # Highlight current target position and endpoint
                                display_simple_grid[ts] = 1
                                display_simple_grid[ep] = 0.75
                                
                                fig, ax = plt.subplots(figsize=(4, 4), dpi=100)
                                ax.imshow(display_simple_grid, cmap="gray", vmin=0, vmax=1)
                                ax.set_title("Grid & Path")
                                ax.axis("off")

                                # Render the Matplotlib figure
                                fig.canvas.draw()
                                # Get RGBA buffer from the figure
                                buf = fig.canvas.buffer_rgba()
                                # Convert buffer to NumPy array
                                plot = np.asarray(buf)
                                # Convert RGBA/RGB → BGR for OpenCV
                                plot = cv2.cvtColor(plot, cv2.COLOR_RGB2BGR)
                                plt.close(fig)  # IMPORTANT: prevent memory leak

                                h, w = plot.shape[:2]
                                frame[0:h, 0:w] = plot
                                print('target id', keys, "path_t2e:", path_t2e)

                                cv2.imshow('frame-image', frame)
                    else:
                        print("Ball or target not detected.")
                        break
                
                if quit_flag:
                    break

            # Crop rotation
            end_points = rotate_dict(end_points, k=2)
            print("Rotated end points: ", end_points)
    
    if quit_flag:
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit()
