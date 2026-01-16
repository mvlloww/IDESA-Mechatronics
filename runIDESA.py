def get_endpoint_xy(endpoint_value):
    """Return endpoint as (x, y) ints. Supports [[x, y], flag], [x, y], (x, y), np.ndarray."""
    if endpoint_value is None:
        return None
    # Handle [[x, y], flag]
    if isinstance(endpoint_value, (list, tuple)) and len(endpoint_value) == 2:
        first = endpoint_value[0]
        if isinstance(first, (list, tuple, np.ndarray)) and len(first) == 2:
            try:
                return (int(first[0]), int(first[1]))
            except Exception:
                return None
    # Handle [x, y] or (x, y)
    if isinstance(endpoint_value, (list, tuple, np.ndarray)) and len(endpoint_value) == 2:
        try:
            return (int(endpoint_value[0]), int(endpoint_value[1]))
        except Exception:
            return None
    return None
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
def get_center(corners, frame, grid_width, grid_height):
    '''
    get center of ArUco code function

    input: corners of ArUco marker, current frame, grid width and height
    output: x and y coordinates on grid of the center of the ArUco code
    '''

    pts = corners.reshape((4, 2))
    center_pixel = pts.mean(axis=0)  # Mean of all 4 corner points
    img_h, img_w = frame.shape[:2]
    grid_x = int(center_pixel[0] / img_w * grid_width)
    grid_y = int(center_pixel[1] / img_h * grid_height)
    center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
    x, y = center_grid
    
    return x, y
def grid_obstacle(ids, corners, target_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height):
    '''
    generate obstacles on grid function

    input: detected ArUco ids, target id to ignore, x/y coordinate arrays of grid, current grid, obstacle radius
    output: updated grid with obstacles marked for all non-target markers
    '''
    img_h, img_w = frame.shape[:2]
    
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        # Ignore target marker and ball marker
        if marker_id != int(target_id) and marker_id != int(ball_id):
            # Get corners for this specific marker
            pts = corners[i].reshape((4, 2))
            center_pixel = pts.mean(axis=0)
            
            # Convert to grid coordinates
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
            ox, oy = center_grid

            # Set obstacle on grid
            mask = (x_coords - ox)**2 + (y_coords - oy)**2 <= radius**2
            grid[mask] = 0

            # Draw circle on frame using pixel coordinates
            ox_pixel = int(ox * img_w / grid_width)
            oy_pixel = int(oy * img_h / grid_height)
            radius_pixel = int(radius * img_w / grid_width)
            cv2.circle(frame, (ox_pixel, oy_pixel), radius_pixel, (0,0,255), 2)
        # # Enforce boundary: x=1,39 and y=1,19 to zeros
        # grid[0, :] = 0         # y=1 (row 0)
        # grid[-1, :] = 0        # y=19 (row 19)
        # grid[:, 0] = 0         # x=1 (col 0)
        # grid[:, -1] = 0        # x=39 (col 39)
    return grid
def rotate_dict(d, k=2):
    items = list(d.items())
    k = k % len(items)   # safety for large shifts
    rotated = items[-k:] + items[:-k]
    return dict(rotated)
def compute_theta_send(theta):
    theta_send = theta
    return theta_send
def bouncing_ball(dy, dx, ball_start):
    '''
    bouncing ball function

    input: dy, dx (grid movement instructions), ball start position
    output: adjusted dy, dx to simulate bouncing effect
    '''
    # ball_x = ball_start[0] + dx
    # ball_y = ball_start[1] + dy
    # if ball_x <=1:
    #     dx = -19
    # elif ball_x>=39:
    #     dx = 19
    # if ball_y <=1:
    #     dy = 9
    # elif ball_y>=19:
    #     dy = -9
    return dy, dx
def send_UDP_signal(ip, port, next_target, frequency = 30):
    '''
    send UDP function

    input: next_target (array of dy, dx, theta, angle_b2t)
    output: sends UDP packet to specified IP and port
    '''
    interval = 1.0 / frequency
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    global shared_next_target, next_target_lock, udp_thread_running
    while udp_thread_running:
        with next_target_lock:
            target = shared_next_target.copy()
        sock.sendto(struct.pack('<iif', int(target[0]), int(target[1]), float(target[2])), (ip, port))
        time.sleep(interval)

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
import socket   #communicate over the network
import struct
import threading


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
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2750)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

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
radius = 1.5

# Set target end points for pathfinding (can be changed later)
#end_points = {1:[5,35], 4:[15,10]}
end_points = {1:[16,6], 2:[16,10], 3:[4,16], 4:[4,24], 5:[16,32], 6:[16,36]}
# Set ball ArUco id
ball_id = 8
# Create id_buffer dictionary
id_buffer = {}

'''Temporary variables'''
IsFire = False
quit_flag = False
mode = 'auto'  # can be 'auto' or 'manual'

''' Setup UDP communication '''
# This is the IP address of the machine that the data will be send to
UDP_IP = "138.38.229.206" #clearpass IP address for RPI 3B Model +
# This is the RENOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
UDP_PORT = 50000

# Shared variables for UDP thread
shared_next_target = [0, 0, 0]
next_target_lock = threading.Lock()
udp_thread_running = True
udp_thread = None

# Crop Rotation True/False
In_range = {}

''' Main loop '''
while True:
    # Start the performance clock
    start = time.perf_counter()
    quit_flag = False

    while IsFire == False and not quit_flag and mode == 'auto':
        # Capture current frame from the camera
        ret, frame = cap.read()

        # Skip frame if capture failed
        if not ret or frame is None:
            continue

        ## check IsFire status here##

        # Convert the image from the camera to Gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers in the grey image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  
        cv2.imshow('frame-image', frame)
        
        # Single key poll per frame to avoid missing inputs
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            quit_flag = True
            break
        if key == ord('m'):
            mode = 'manual'
            print ("Manual mode activated. Use 'awsd' keys to control the ball, 'c' to switch to auto mode, 'q' to quit.")
            break

        # If markers are detected, draw them, estimate pose and overlay axes
        if ids is not None and len(ids) > 0:
            try:
                print("DEBUG: Detected IDs:", ids.flatten().tolist())
            except Exception:
                print("DEBUG: Detected IDs present, but could not print list.")
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

                if marker_id not in end_points.keys():
                    continue  # Skip markers not in end_points

                # Update grid with obstacles for all non-target markers
                grid = grid_obstacle(ids, corners, marker_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)
                
                if get_center(corners[t], frame, grid_width, grid_height) == end_points.get(marker_id):
                    In_range[marker_id] = True 
                    continue
                elif marker_id in end_points.keys() and get_center(corners[t], frame, grid_width, grid_height) != end_points.get(marker_id):
                    In_range[marker_id] = False

                # Get center of target ArUco code on grid
                start_point = get_center(corners[t], frame, grid_width, grid_height)
                #print("1. Start Point for ID ", marker_id, ": ", start_point)

                # append start points and id[t] to id_buffer
                if marker_id not in id_buffer:
                    id_buffer[marker_id] = start_point

                # Get path from target to end point and ball to target
                # tcod.path.path2d expects sequences of (i,j) pairs
                sp = (int(start_point[1]), int(start_point[0])) # Note the (y,x) order for (i,j)
                target_endpoint = get_endpoint_xy(end_points.get(marker_id))
                if target_endpoint is not None:
                    ep = (int(target_endpoint[0]), int(target_endpoint[1]))
                    path_t2e = tcod.path.path2d(cost=grid, start_points=[sp], end_points=[ep], cardinal=10, diagonal=14)
                    ball_idx = np.where(ids == ball_id)[0]
                    if ball_idx.size > 0:
                        ball_center = get_center(corners[ball_idx[0]], frame, grid_width, grid_height)
                        bs = (int(ball_center[1]), int(ball_center[0]))
                        path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[sp], cardinal=10, diagonal=14)
                    else:
                        path_b2t = []
                    total_path = np.concatenate((path_b2t, path_t2e)) if len(path_b2t) > 0 else path_t2e
                    # append total path to target_dict 
                    target_dict[marker_id] = len(total_path)
            # Sort target_dict based on path length
            sorted_targets = sorted(target_dict.items(), key=lambda item: item[1])
            
            if sorted_targets:
                keys = sorted_targets[0][0]  # Lock onto the first target only
                position_status = False
                while position_status == False and not quit_flag:
                    ret, frame = cap.read()
                    
                    # Skip frame if capture failed
                    if not ret or frame is None:
                        continue

                    ## check IsFire status here##

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                    
                    # refresh grid
                    grid.fill(1)

                    # Single key poll per frame to avoid missing inputs
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        quit_flag = True
                        break
                    if key == ord('m'):
                        mode = 'manual'
                        print ("Manual mode activated. Use 'awsd' keys to control the ball, 'c' to switch to auto mode, 'q' to quit.")
                        break

                    if ids is not None and len(ids) > 0 and keys in ids.flatten() and ball_id in ids.flatten():
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        center_key = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        center_ball = get_center(corners[np.where(ids==ball_id)[0][0]], frame, grid_width, grid_height)
                        # Print grid positions of ball and target
                        print(f"Ball (ID {ball_id}) grid position: {center_ball}, Target (ID {keys}) grid position: {center_key}")
                        # Get angle of ball to target
                        angle_b2t = np.arctan2(center_key[1]-center_ball[1], center_key[0]-center_ball[0])
                        
                        # cv2.imshow('frame-image', frame)
                        if end_points.get(keys) == center_key:
                            position_status = True
                            In_range[keys] = True
                            break
                            #print ("3. Target ID ", keys, " reached endpoint. switching to next target.")

                        grid = grid_obstacle(ids, corners, keys, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)

                        if center_ball is not None and center_key is not None:
                            ball_start = center_ball
                            target_start = center_key

                            # pathfinding target to end point
                            # Clip coordinates to grid bounds
                            ts = (max(0, min(grid_height-1, int(target_start[1]))), max(0, min(grid_width-1, int(target_start[0])))) # (x,y) -> (y,x) format
                            endpoint_xy = get_endpoint_xy(end_points.get(keys))
                            ep = (max(0, min(grid_height-1, int(endpoint_xy[1]))), max(0, min(grid_width-1, int(endpoint_xy[0])))) # (x,y) format
                            path_t2e = tcod.path.path2d(cost=grid, start_points=[ts], end_points=[ep], cardinal=10, diagonal=14)
                            
                            ballTarget_distance = 2

                            if abs(ball_start[0] - target_start[0]) > ballTarget_distance or abs(ball_start[1] - target_start[1]) > ballTarget_distance:
                                # Set target to obstacle
                                mask = (x_coords - target_start[0])**2 + (y_coords - target_start[1])**2 <= radius**2
                                grid[mask] = 0
                                # Ensure img_h and img_w are defined from frame
                                img_h, img_w = frame.shape[:2]
                                # Draw circle on frame using pixel coordinates
                                ox_pixel = int(target_start[0] * img_w / grid_width)
                                oy_pixel = int(target_start[1] * img_h / grid_height)
                                radius_pixel = int(radius * img_w / grid_width)
                                cv2.circle(frame, (ox_pixel, oy_pixel), radius_pixel, (0,0,255), 2)

                                # Determine fake target position opposite to endpoint
                                if len(path_t2e) >= 2:
                                    # Vector from target to endpoint
                                    vec_t2e = (path_t2e[1][1] - target_start[0], path_t2e[1][0] - target_start[1])
                                    # Place fake target opposite the endpoint, at a certain distance
                                    push_distance = 2  # adjust as needed
                                    fake_target = (target_start[0] - push_distance * vec_t2e[0], target_start[1] - push_distance * vec_t2e[1])
                                else:
                                    # Path too short, use target position
                                    fake_target = target_start
                                
                                # pathfinding ball to fake target
                                # Clip coordinates to grid bounds
                                bs = (max(0, min(grid_height-1, int(ball_start[1]))), max(0, min(grid_width-1, int(ball_start[0]))))
                                ts = (max(0, min(grid_height-1, int(fake_target[1]))), max(0, min(grid_width-1, int(fake_target[0]))))
                                path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[ts], cardinal=10, diagonal=14)
                                # tcod can return an empty array; guard simplify_path
                                if len(path_b2t) > 0:
                                    _, diagonaldown_path_b2t = simplify_path(path_b2t)
                                else:
                                    diagonaldown_path_b2t = []
                                #print('6. target id', keys, "path_b2t:", diagonaldown_path_b2t)

                                # Check if path has at least 2 points before accessing [1]
                                if len(diagonaldown_path_b2t) > 0:
                                    # convert to dx, dy instructions for UDP sending
                                    dy = diagonaldown_path_b2t[1][0] - ball_start[1]
                                    dx = diagonaldown_path_b2t[1][1] - ball_start[0]
                                    dy, dx = bouncing_ball(dy, dx, ball_start)

                                    # Recalculate ball_idx for current frame
                                    ball_idx = np.where(ids == ball_id)[0]
                                    if ball_idx.size > 0:
                                        rotate_vec = rvecs[ball_idx[0]][0]
                                        R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                        yaw = np.arctan2(R[1, 0], R[0, 0])
                                        # UDP sending
                                        next_target = np.array([dy, dx,compute_theta_send(yaw), angle_b2t])  # example data to send (y,x (i,j)) coordinates of next target point
                                        with next_target_lock:
                                            shared_next_target = next_target.copy()
                                        print ("9. message:", next_target)
                                # Convert grid coordinates to pixel coordinates
                                img_h, img_w = frame.shape[:2]
                                
                                # Draw grid lines FIRST (before other elements)
                                for i in range(grid_height + 1):
                                    y = int(round(i * img_h / grid_height))
                                    cv2.line(frame, (0, y), (img_w, y), (100, 100, 100), 1)
                                for j in range(grid_width + 1):
                                    x = int(round(j * img_w / grid_width))
                                    cv2.line(frame, (x, 0), (x, img_h), (100, 100, 100), 1)
                                
                                # Convert path from grid to pixel coordinates
                                if len(diagonaldown_path_b2t) >= 1:
                                    path_pixels = []
                                    for point in diagonaldown_path_b2t:
                                        # point is (i, j) in grid coords - convert to (x, y) pixels
                                        x_pixel = int(point[1] * img_w / grid_width)
                                        y_pixel = int(point[0] * img_h / grid_height)
                                        path_pixels.append([x_pixel, y_pixel])
                                    path_pixels = np.array(path_pixels, dtype=np.int32)
                                    cv2.polylines(frame, [path_pixels], False, color=(255,0,0), thickness=2)
                                
                                # Convert marker positions to pixel coordinates
                                ball_pixel = (int(center_ball[0] * img_w / grid_width), int(center_ball[1] * img_h / grid_height))
                                target_pixel = (int(target_start[0] * img_w / grid_width), int(target_start[1] * img_h / grid_height))
                                fake_pixel = (int(fake_target[0] * img_w / grid_width), int(fake_target[1] * img_h / grid_height))
                                
                                cv2.drawMarker(frame, ball_pixel, (0,255,0), cv2.MARKER_TILTED_CROSS, 40, 2)
                                cv2.drawMarker(frame, target_pixel, (0,0,255), cv2.MARKER_TILTED_CROSS, 40, 2)
                                cv2.drawMarker(frame, fake_pixel, (255,0,255), cv2.MARKER_DIAMOND, 40, 2)
                                for k in end_points.keys():
                                    ep_coords = end_points.get(k)
                                    ep_pixel = (int(ep_coords[1] * img_w / grid_width), int(ep_coords[0] * img_h / grid_height))
                                    cv2.drawMarker(frame, ep_pixel, (255,255,0), cv2.MARKER_DIAMOND, 40, 2)
                                    # Draw the id number next to the endpoint marker
                                    text_pos = (ep_pixel[0] + 10, ep_pixel[1] - 10)
                                    cv2.putText(frame, str(k), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2, cv2.LINE_AA)
                                
                                cv2.imshow('frame-image', frame)
                            else:
                                if len(path_t2e) > 0:
                                    _, diagonaldown_path_t2e = simplify_path(path_t2e)
                                else:
                                    diagonaldown_path_t2e = []

                                # convert to dx, dy instructions for UDP sending
                                # Check if path has at least 2 points before accessing [1]
                                if len(diagonaldown_path_t2e) < 1:
                                    continue
                                
                                dy = diagonaldown_path_t2e[1][0] - target_start[1]
                                dx = diagonaldown_path_t2e[1][1] - target_start[0]
                                dy, dx = bouncing_ball(dy, dx, ball_start)

                                # Recalculate ball_idx for current frame
                                ball_idx = np.where(ids == ball_id)[0]
                                if ball_idx.size > 0:
                                    rotate_vec = rvecs[ball_idx[0]][0]
                                    R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                    yaw = np.arctan2(R[1, 0], R[0, 0])

                                    # UDP sending
                                    next_target = np.array([dy, dx, compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                    with next_target_lock:
                                        shared_next_target = next_target.copy()
                                    print ("13. message:", next_target)

                                # Convert path from grid to pixel coordinates
                                img_h, img_w = frame.shape[:2]
                                
                                # Draw grid lines FIRST (before other elements)
                                for i in range(grid_height + 1):
                                    y = int(round(i * img_h / grid_height))
                                    cv2.line(frame, (0, y), (img_w, y), (100, 100, 100), 1)
                                for j in range(grid_width + 1):
                                    x = int(round(j * img_w / grid_width))
                                    cv2.line(frame, (x, 0), (x, img_h), (100, 100, 100), 1)
                                
                                if len(diagonaldown_path_t2e) >= 1:
                                    path_pixels = []
                                    for point in diagonaldown_path_t2e:
                                        x_pixel = int(point[1] * img_w / grid_width)
                                        y_pixel = int(point[0] * img_h / grid_height)
                                        path_pixels.append([x_pixel, y_pixel])
                                    path_pixels = np.array(path_pixels, dtype=np.int32)
                                    cv2.polylines(frame, [path_pixels], False, color=(255,0,0), thickness=2)
                                
                                # Convert markers to pixel coordinates
                                ball_pixel = (int(center_ball[0] * img_w / grid_width), int(center_ball[1] * img_h / grid_height))
                                target_pixel = (int(target_start[0] * img_w / grid_width), int(target_start[1] * img_h / grid_height))
                                for k in end_points.keys():
                                    ep_coords = end_points.get(k)
                                    ep_pixel = (int(ep_coords[1] * img_w / grid_width), int(ep_coords[0] * img_h / grid_height))
                                    cv2.drawMarker(frame, ep_pixel, (255,255,0), cv2.MARKER_DIAMOND, 40, 2)
                                    # Draw the id number next to the endpoint marker
                                    text_pos = (ep_pixel[0] + 10, ep_pixel[1] - 10)
                                    cv2.putText(frame, str(k), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2, cv2.LINE_AA)

                                cv2.drawMarker(frame, ball_pixel, (0,255,0), cv2.MARKER_TILTED_CROSS, 40, 2)
                                cv2.drawMarker(frame, target_pixel, (0,0,255), cv2.MARKER_TILTED_CROSS, 40, 2)
                                cv2.imshow('frame-image', frame)
                    else:
                        cv2.imshow('frame-image', frame)
                        break
                
                # Display the frame once after all processing
                #cv2.imshow('frame-image', frame)
                if quit_flag or mode == 'manual':
                    break

            # Crop rotation
            # if In_range.get(keys)==True:
            #     end_points = rotate_dict(end_points, k=2)

            # Final check for all targets reached - rotate if so
            if len(In_range) == 6 and all(In_range.get(k, False) for k in end_points.keys()):
                end_points = rotate_dict(end_points, k=2)
        
    while mode == 'manual' and not quit_flag:
        
        # Capture current frame from the camera
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        yaw = 0.0  # Default yaw value
        
        # Only fetch yaw if we see the ball marker
        if ids is not None and len(ids) > 0 and ball_id in ids.flatten():
            out = aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
            ball_idx = np.where(ids == ball_id)[0]
            if ball_idx.size > 0:
                rotate_vec = rvecs[ball_idx[0]][0]
                R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                yaw = np.arctan2(R[1, 0], R[0, 0])
                print ('M1. rotate_vec:', yaw)

        # Check for 'awsd' key press for manual control
        key = cv2.waitKey(1) & 0xFF
        next_target = None
        if key == ord('w'):
            print ('direction = up')
            next_target = np.array([50, 0, compute_theta_send(yaw), 0])
            
        elif key == ord('a'):
            print ('direction = left')
            next_target = np.array([0, 50, compute_theta_send(yaw), 0])
        elif key == ord('s'):
            print ('direction = down')
            next_target = np.array([-50, 0, compute_theta_send(yaw), 0])

        elif key == ord('d'):
            print ('direction = right')
            next_target = np.array([0, -50, compute_theta_send(yaw), 0])
        elif key == ord('q'):
            quit_flag = True
            break
        elif key == ord('c'): # c to change back to auto mode
            mode = 'auto'
            print ("Auto mode activated: normal operation")
            break

        # Default to zero command when no movement key was pressed
        if next_target is None:
            next_target = np.array([0, 0, compute_theta_send(yaw), 0])
            print ('no movement key pressed')

        with next_target_lock:
            shared_next_target = next_target.copy()
        cv2.imshow('frame-image', frame)

    if quit_flag:
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# Stop UDP thread on exit
udp_thread_running = False
if udp_thread is not None:
    udp_thread.join()
# exit the kernel
exit()
