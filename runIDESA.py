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

def draw_in_range_status(frame, in_range_dict):
    """Draw the In_range status on the bottom left of the frame."""
    img_h, img_w = frame.shape[:2]
    # Build status text
    status_lines = ["In Range:"]
    for marker_id, is_in_range in sorted(in_range_dict.items()):
        status = "YES" if is_in_range else "NO"
        status_lines.append(f"  ID {marker_id}: {status}")
    
    # Draw each line from bottom up
    line_height = 25
    start_y = img_h - 20 - (len(status_lines) - 1) * line_height
    for i, line in enumerate(status_lines):
        y_pos = start_y + i * line_height
        # Draw background rectangle for better visibility
        cv2.putText(frame, line, (10, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)  # Black outline
        cv2.putText(frame, line, (10, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # Green text

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
    theta_send = -theta
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

def get_2d_angle_from_corners(corners):
    """
    Calculate 2D rotation angle from ArUco marker corners.
    Corners are ordered: [top-left, top-right, bottom-right, bottom-left]
    
    Coordinate system:
    - 0° = marker's forward direction pointing UP the screen (negative Y)
    - Positive theta = anticlockwise rotation from up
    - Negative theta = clockwise rotation from up
    
    This ignores tilt and pitch, only measuring in-plane rotation.
    """
    pts = corners.reshape((4, 2))
    # Vector from top-left to top-right (top edge)
    top_edge = pts[1] - pts[0]
    
    # Forward direction is perpendicular to top edge (90° CW from top edge)
    # When marker is upright, top edge points right, forward points up
    forward_x = top_edge[1]   # rotated 90° CW: [y, -x]
    forward_y = -top_edge[0]
    
    # Calculate angle from screen-up (negative Y axis), with CCW positive
    # atan2(-x, -y) gives angle from Y- axis with CCW (anticlockwise) positive
    angle = np.arctan2(-forward_x, -forward_y)
    return angle

def detect_fire(frame):
    """
    Detect large red object in the frame (fire detection).
    Returns: (is_fire_detected, fire_center_pixel, fire_area)
    - is_fire_detected: True if large red object found
    - fire_center_pixel: (x, y) pixel coordinates of fire center, or None
    - fire_area: area of detected red region in pixels
    
    TODO: Adjust HSV ranges and area threshold based on actual fire/red object
    """
    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red color range in HSV (red wraps around, so need two ranges)
    # TODO: Adjust these values based on actual red object to detect
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Create masks for both red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the largest red contour
    min_fire_area = 500  # TODO: Adjust minimum area threshold
    largest_area = 0
    fire_center = None
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area and area > min_fire_area:
            largest_area = area
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                fire_center = (cx, cy)
    
    is_fire = fire_center is not None and largest_area > min_fire_area
    return is_fire, fire_center, largest_area

def get_turret_forward_direction(corners):
    """
    Get the forward direction vector of the turret (north direction of ArUco).
    Returns unit vector (fx, fy) in pixel coordinates pointing in turret's forward direction.
    Forward is defined as perpendicular to the top edge, pointing "up" from the marker.
    """
    pts = corners.reshape((4, 2))
    # Vector from top-left to top-right (top edge)
    top_edge = pts[1] - pts[0]
    
    # Forward direction is perpendicular to top edge (90° CCW from top edge in screen coords)
    # In screen coords (Y down), rotating 90° CCW: (x, y) -> (-y, x)
    # But we want "up" from marker, which is 90° CW: (x, y) -> (y, -x)
    forward_x = top_edge[1]
    forward_y = -top_edge[0]
    
    # Normalize to unit vector
    length = np.sqrt(forward_x**2 + forward_y**2)
    if length > 0:
        forward_x /= length
        forward_y /= length
    
    return forward_x, forward_y

def find_closest_target_to_fire(fire_center_pixel, ids, corners, end_points, frame, grid_width, grid_height):
    """
    Find which target (1-6) is closest to the detected fire.
    Returns the marker_id of the closest target.
    """
    min_distance = float('inf')
    closest_target_id = None
    
    if ids is None or fire_center_pixel is None:
        return None
    
    img_h, img_w = frame.shape[:2]
    
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in end_points.keys():  # Only check targets 1-6
            # Get pixel center of this marker
            pts = corners[i].reshape((4, 2))
            center_pixel = pts.mean(axis=0)
            
            # Calculate distance to fire
            dist = np.sqrt((center_pixel[0] - fire_center_pixel[0])**2 + 
                          (center_pixel[1] - fire_center_pixel[1])**2)
            
            if dist < min_distance:
                min_distance = dist
                closest_target_id = int(marker_id)
    
    return closest_target_id



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
# Define the aruco detection parameters (tuned for distant markers)
parameters = aruco.DetectorParameters()
# For distant/small markers - lower the minimum perimeter rate
parameters.minMarkerPerimeterRate = 0.01  # Default 0.03, lower = detect smaller markers
parameters.maxMarkerPerimeterRate = 4.0
# Adaptive thresholding - more robust to lighting variations
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.adaptiveThreshConstant = 7
# Corner refinement for more stable detection
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 5
parameters.cornerRefinementMaxIterations = 30
# Error correction - helps with partially obscured markers
parameters.errorCorrectionRate = 0.6

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

# Ball detection temporal smoothing
ball_last_corners = None  # Last known ball corners
ball_last_center = None   # Last known ball center (grid coords)
ball_last_yaw = 0.0       # Last known ball yaw angle
ball_missing_frames = 0   # Counter for frames ball has been missing
ball_max_missing = 5      # Max frames to keep using last known position

'''Temporary variables'''
IsFire = False
quit_flag = False
mode = 'auto'  # can be 'auto' or 'manual'

# Turret/Fire detection variables
turret_id = 10  # ArUco ID for the turret
fire_center_pixel = None  # Pixel coords of detected fire
fire_target_id = None  # Which target (1-6) is closest to fire
turret_endpoint = None  # Calculated endpoint for turret [y, x] format
pump_active = False  # Track if pump is currently on

''' Setup UDP communication '''
# Ball control UDP
UDP_IP = "138.38.229.206" #clearpass IP address for RPI 3B Model +
UDP_PORT = 50000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Turret pump UDP (separate socket)
# TODO: Set actual IP and port for turret pump control
TURRET_UDP_IP = "138.38.229.207"  # Placeholder - update with actual turret IP
TURRET_UDP_PORT = 50001  # Placeholder - update with actual turret port
turret_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

        # Check for fire (large red object)
        is_fire_detected, fire_center_pixel, fire_area = detect_fire(frame)
        if is_fire_detected:
            IsFire = True
            # Draw fire indicator on frame
            cv2.circle(frame, fire_center_pixel, 20, (0, 0, 255), 3)
            cv2.putText(frame, "FIRE DETECTED!", (fire_center_pixel[0]-50, fire_center_pixel[1]-30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            print(f"Fire detected at pixel {fire_center_pixel}, area: {fire_area}")
            break  # Exit to fire handling loop

        # Convert the image from the camera to Gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers in the grey image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  
        draw_in_range_status(frame, In_range)
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

                # Get center of this marker
                marker_center = get_center(corners[t], frame, grid_width, grid_height)  # returns (x, y)
                target_endpoint = end_points.get(marker_id)  # stored as [y, x]
                
                # Check if marker is at its endpoint (with tolerance of 1 grid cell)
                # marker_center is (x, y), endpoint is [y, x]
                endpoint_x = target_endpoint[1]
                endpoint_y = target_endpoint[0]
                distance_to_endpoint = abs(marker_center[0] - endpoint_x) + abs(marker_center[1] - endpoint_y)
                
                if distance_to_endpoint <= 1:  # Within 1 grid cell tolerance
                    In_range[marker_id] = True
                    # Mark this target as an obstacle since it's in position
                    mask = (x_coords - marker_center[0])**2 + (y_coords - marker_center[1])**2 <= radius**2
                    grid[mask] = 0
                    continue  # Skip pathfinding for this target
                else:
                    In_range[marker_id] = False

                # Update grid with obstacles for all non-target markers
                grid = grid_obstacle(ids, corners, marker_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)

                # Get center of target ArUco code on grid
                start_point = marker_center
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

                    # Check if current target is still visible
                    target_visible = ids is not None and len(ids) > 0 and keys in ids.flatten()
                    
                    if not target_visible:
                        # Target disappeared - break out to recalculate and pick new target
                        print(f"Target ID {keys} lost - switching to new target...")
                        break
                    
                    if target_visible and ball_id in ids.flatten():
                        # Ball detected - update tracking
                        ball_missing_frames = 0
                        ball_idx_temp = np.where(ids == ball_id)[0][0]
                        ball_last_corners = corners[ball_idx_temp].copy()
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        center_key = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        center_ball = get_center(corners[np.where(ids==ball_id)[0][0]], frame, grid_width, grid_height)
                        ball_last_center = center_ball  # Update last known center
                        # Print grid positions of ball and target
                        print(f"Ball (ID {ball_id}) grid position: {center_ball}, Target (ID {keys}) grid position: {center_key}")
                    elif target_visible and ball_last_center is not None and ball_missing_frames < ball_max_missing:
                        # Ball NOT detected but we have recent data - use last known position
                        ball_missing_frames += 1
                        print(f"Ball missing frame {ball_missing_frames}/{ball_max_missing} - using last known position")
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        center_key = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        center_ball = ball_last_center  # Use last known ball center
                        # Print grid positions of ball and target
                        print(f"Ball (ID {ball_id}) grid position (CACHED): {center_ball}, Target (ID {keys}) grid position: {center_key}")
                    else:
                        # Ball not detected and cache expired - reset cache and wait for ball
                        if ball_missing_frames >= ball_max_missing:
                            # Cache expired - reset so fresh detection can work
                            print(f"Ball cache expired - waiting for ball detection...")
                            ball_missing_frames = 0  # Reset counter for next detection cycle
                        elif ball_last_corners is not None:
                            ball_missing_frames += 1
                        draw_in_range_status(frame, In_range)
                        cv2.imshow('frame-image', frame)
                        continue
                    
                    # Common code for both detected and cached ball
                    if True:  # Replaces the old if block indent
                        # Update In_range for ALL visible targets (not just current one)
                        for t_idx in range(len(ids)):
                            t_id = int(ids[t_idx][0])
                            if t_id in end_points.keys() and t_id != ball_id:
                                t_center = get_center(corners[t_idx], frame, grid_width, grid_height)
                                t_endpoint = end_points.get(t_id)
                                t_dist = abs(t_center[0] - t_endpoint[1]) + abs(t_center[1] - t_endpoint[0])
                                In_range[t_id] = (t_dist <= 1)
                        
                        # Get angle of ball to target
                        angle_b2t = np.arctan2(center_key[1]-center_ball[1], center_key[0]-center_ball[0])
                        
                        # cv2.imshow('frame-image', frame)
                        # Check if target reached endpoint (with tolerance)
                        target_endpoint = end_points.get(keys)  # [y, x]
                        endpoint_x = target_endpoint[1]
                        endpoint_y = target_endpoint[0]
                        distance_to_endpoint = abs(center_key[0] - endpoint_x) + abs(center_key[1] - endpoint_y)
                        
                        if distance_to_endpoint <= 1:  # Within 1 grid cell tolerance
                            position_status = True
                            In_range[keys] = True
                            print(f"Target ID {keys} reached endpoint!")
                            break
                            #print ("3. Target ID ", keys, " reached endpoint. switching to next target."))

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

                                # Threshold vector to determine attack angle onto target
                                attackThreshold = 3

                                # Determine fake target position opposite to endpoint
                                if len(path_t2e) >= attackThreshold:
                                    # Vector from target to endpoint
                                    vec_t2e = (path_t2e[1][1] - target_start[0], path_t2e[1][0] - target_start[1])
                                    # Place fake target opposite the endpoint, at a certain distance
                                    push_distance = attackThreshold  # adjust as needed
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

                                    # Get ball corners - use detected or cached
                                    ball_idx = np.where(ids == ball_id)[0]
                                    if ball_idx.size > 0:
                                        ball_corners_for_yaw = corners[ball_idx[0]]
                                    elif ball_last_corners is not None:
                                        ball_corners_for_yaw = ball_last_corners
                                    else:
                                        ball_corners_for_yaw = None
                                    
                                    if ball_corners_for_yaw is not None:
                                        # Use 2D corner-based angle (ignores tilt/pitch)
                                        yaw = get_2d_angle_from_corners(ball_corners_for_yaw)
                                        ball_last_yaw = yaw  # Update cached yaw
                                        # UDP sending
                                        next_target = np.array([dy, dx,compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                        sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                        print ("9. message:", next_target)
                                        
                                        # Visualize theta (2D angle) on frame
                                        theta_deg = np.degrees(yaw)
                                        theta_send_deg = np.degrees(compute_theta_send(yaw))
                                        # Get ball center in pixels for visualization
                                        ball_corners_vis = ball_corners_for_yaw.reshape((4, 2))
                                        ball_center_px = ball_corners_vis.mean(axis=0)
                                        # Draw arrow showing marker's forward direction
                                        # Arrow points UP when theta=0, rotates CW for positive theta
                                        arrow_length = 60
                                        # Convert from our angle system (0=up, CW+) to image coords for drawing
                                        # In image: up is -Y, so we use sin for X and -cos for Y
                                        arrow_start = (int(ball_center_px[0]), int(ball_center_px[1]))
                                        arrow_end = (int(ball_center_px[0] + arrow_length * np.sin(yaw)),
                                                     int(ball_center_px[1] - arrow_length * np.cos(yaw)))
                                        cv2.arrowedLine(frame, arrow_start, arrow_end, (0, 255, 255), 3, tipLength=0.3)
                                        # Display theta text
                                        cv2.putText(frame, f"Theta: {theta_deg:.1f} deg", (10, 30), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                                        cv2.putText(frame, f"Theta_send: {theta_send_deg:.1f} deg", (10, 60), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
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
                                
                                draw_in_range_status(frame, In_range)
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

                                # Get ball corners - use detected or cached
                                ball_idx = np.where(ids == ball_id)[0]
                                if ball_idx.size > 0:
                                    ball_corners_for_yaw = corners[ball_idx[0]]
                                elif ball_last_corners is not None:
                                    ball_corners_for_yaw = ball_last_corners
                                else:
                                    ball_corners_for_yaw = None
                                
                                if ball_corners_for_yaw is not None:
                                    # Use 2D corner-based angle (ignores tilt/pitch)
                                    yaw = get_2d_angle_from_corners(ball_corners_for_yaw)
                                    ball_last_yaw = yaw  # Update cached yaw

                                    # UDP sending
                                    next_target = np.array([dy, dx, compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                    sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                    print ("13. message:", next_target)
                                    
                                    # Visualize theta (2D angle) on frame
                                    theta_deg = np.degrees(yaw)
                                    theta_send_deg = np.degrees(compute_theta_send(yaw))
                                    # Get ball center in pixels for visualization
                                    ball_corners_vis = ball_corners_for_yaw.reshape((4, 2))
                                    ball_center_px = ball_corners_vis.mean(axis=0)
                                    # Draw arrow showing marker's forward direction
                                    # Arrow points UP when theta=0, rotates CW for positive theta
                                    arrow_length = 60
                                    # Convert from our angle system (0=up, CW+) to image coords for drawing
                                    # In image: up is -Y, so we use sin for X and -cos for Y
                                    arrow_start = (int(ball_center_px[0]), int(ball_center_px[1]))
                                    arrow_end = (int(ball_center_px[0] + arrow_length * np.sin(yaw)),
                                                 int(ball_center_px[1] - arrow_length * np.cos(yaw)))
                                    cv2.arrowedLine(frame, arrow_start, arrow_end, (0, 255, 255), 3, tipLength=0.3)
                                    # Display theta text
                                    cv2.putText(frame, f"Theta: {theta_deg:.1f} deg", (10, 30), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                                    cv2.putText(frame, f"Theta_send: {theta_send_deg:.1f} deg", (10, 60), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

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
                                draw_in_range_status(frame, In_range)
                                cv2.imshow('frame-image', frame)
                
                # Display the frame once after all processing
                #cv2.imshow('frame-image', frame)
                if quit_flag or mode == 'manual':
                    break

            # Crop rotation
            # if In_range.get(keys)==True:
            #     end_points = rotate_dict(end_points, k=2)

            # Final check for all targets reached - rotate endpoints
            if len(In_range) >= len(end_points) and all(In_range.get(k, False) for k in end_points.keys()):
                print("All targets in position! Rotating endpoints...")
                end_points = rotate_dict(end_points, k=2)
                In_range.clear()  # Reset In_range for new endpoint assignments
                print(f"New endpoints: {end_points}")
    
    # ==================== FIRE HANDLING LOOP ====================
    while IsFire == True and not quit_flag and mode == 'auto':
        # Capture current frame from the camera
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        
        # Check if fire is still present
        is_fire_detected, fire_center_pixel, fire_area = detect_fire(frame)
        
        if not is_fire_detected:
            # Fire extinguished - send pump OFF and return to normal mode
            if pump_active:
                turret_sock.sendto(b'0', (TURRET_UDP_IP, TURRET_UDP_PORT))
                pump_active = False
            print("Fire extinguished! Returning to normal auto mode...")
            IsFire = False
            fire_target_id = None
            turret_endpoint = None
            pump_active = False
            break  # Exit fire loop, outer while True will re-enter normal auto loop
        
        # Draw fire indicator
        cv2.circle(frame, fire_center_pixel, 20, (0, 0, 255), 3)
        cv2.putText(frame, "FIRE MODE!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Convert to grayscale and detect markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Single key poll
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            quit_flag = True
            break
        if key == ord('m'):
            mode = 'manual'
            IsFire = False  # Exit fire mode if switching to manual
            break
        
        if ids is None or len(ids) == 0:
            draw_in_range_status(frame, In_range)
            cv2.imshow('frame-image', frame)
            continue
        
        out = aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Find which target is closest to fire (only need to do once or when fire moves)
        if fire_target_id is None:
            fire_target_id = find_closest_target_to_fire(fire_center_pixel, ids, corners, end_points, frame, grid_width, grid_height)
            if fire_target_id is not None:
                print(f"Fire nearest to target ID {fire_target_id}")
                # Set turret endpoint to be near the fire target
                # Get the current position of the fire target as the turret's destination
                fire_target_idx = np.where(ids == fire_target_id)[0]
                if fire_target_idx.size > 0:
                    fire_target_center = get_center(corners[fire_target_idx[0]], frame, grid_width, grid_height)
                    # Set endpoint slightly offset (you can adjust this offset)
                    turret_endpoint = [fire_target_center[1], fire_target_center[0]]  # [y, x] format
                    print(f"Turret endpoint set to: {turret_endpoint}")
        
        # Check if turret (ID 10) and ball (ID 8) are visible
        turret_idx = np.where(ids == turret_id)[0]
        ball_idx = np.where(ids == ball_id)[0]
        
        if turret_idx.size == 0:
            cv2.putText(frame, "Turret (ID 10) not found!", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            draw_in_range_status(frame, In_range)
            cv2.imshow('frame-image', frame)
            continue
        
        if ball_idx.size == 0 and ball_last_center is None:
            cv2.putText(frame, "Ball not found!", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            draw_in_range_status(frame, In_range)
            cv2.imshow('frame-image', frame)
            continue
        
        # Get turret position and orientation
        turret_center = get_center(corners[turret_idx[0]], frame, grid_width, grid_height)  # (x, y)
        turret_forward_x, turret_forward_y = get_turret_forward_direction(corners[turret_idx[0]])
        turret_yaw = get_2d_angle_from_corners(corners[turret_idx[0]])
        
        # Get ball position (use cached if not visible)
        if ball_idx.size > 0:
            ball_center = get_center(corners[ball_idx[0]], frame, grid_width, grid_height)
            ball_last_center = ball_center
            ball_last_corners = corners[ball_idx[0]].copy()
            ball_missing_frames = 0
        elif ball_last_center is not None and ball_missing_frames < ball_max_missing:
            ball_center = ball_last_center
            ball_missing_frames += 1
        else:
            draw_in_range_status(frame, In_range)
            cv2.imshow('frame-image', frame)
            continue
        
        # Reset grid and add obstacles (all targets 1-6 are obstacles in fire mode)
        grid.fill(1)
        for t in range(len(ids)):
            m_id = int(ids[t][0])
            if m_id in end_points.keys():  # Targets 1-6 are obstacles
                m_center = get_center(corners[t], frame, grid_width, grid_height)
                mask = (x_coords - m_center[0])**2 + (y_coords - m_center[1])**2 <= radius**2
                grid[mask] = 0
        
        # Get image dimensions for coordinate conversion
        img_h, img_w = frame.shape[:2]
        
        # Convert fire pixel position to grid coordinates
        fire_grid_x = int(fire_center_pixel[0] / img_w * grid_width)
        fire_grid_y = int(fire_center_pixel[1] / img_h * grid_height)
        fire_grid_pos = (fire_grid_x, fire_grid_y)
        
        # ===== TURRET PATHFINDING TO FIRE =====
        # Calculate path from turret to fire position
        turret_start = (max(0, min(grid_height-1, int(turret_center[1]))), 
                        max(0, min(grid_width-1, int(turret_center[0]))))  # (row, col) format
        fire_end = (max(0, min(grid_height-1, fire_grid_y)), 
                    max(0, min(grid_width-1, fire_grid_x)))  # (row, col) format
        
        path_turret2fire = tcod.path.path2d(cost=grid, start_points=[turret_start], end_points=[fire_end], cardinal=10, diagonal=14)
        
        # Get the direction turret needs to move (from path)
        if len(path_turret2fire) >= 2:
            _, turret_path_simplified = simplify_path(path_turret2fire)
            
            if len(turret_path_simplified) >= 2:
                # Direction from turret to next waypoint in path
                path_dy = turret_path_simplified[1][0] - turret_center[1]  # row direction
                path_dx = turret_path_simplified[1][1] - turret_center[0]  # col direction
                
                # Normalize the path direction
                path_length = np.sqrt(path_dx**2 + path_dy**2)
                if path_length > 0:
                    path_dir_x = path_dx / path_length
                    path_dir_y = path_dy / path_length
                else:
                    path_dir_x, path_dir_y = 0, 0
            else:
                # Fallback: direct direction to fire
                path_dx = fire_grid_x - turret_center[0]
                path_dy = fire_grid_y - turret_center[1]
                path_length = np.sqrt(path_dx**2 + path_dy**2)
                if path_length > 0:
                    path_dir_x = path_dx / path_length
                    path_dir_y = path_dy / path_length
                else:
                    path_dir_x, path_dir_y = 0, 0
        else:
            # No path found, use direct direction to fire
            path_dx = fire_grid_x - turret_center[0]
            path_dy = fire_grid_y - turret_center[1]
            path_length = np.sqrt(path_dx**2 + path_dy**2)
            if path_length > 0:
                path_dir_x = path_dx / path_length
                path_dir_y = path_dy / path_length
            else:
                path_dir_x, path_dir_y = 0, 0
            turret_path_simplified = []
        
        # Check distance from turret to fire
        dist_to_fire = np.sqrt((turret_center[0] - fire_grid_x)**2 + (turret_center[1] - fire_grid_y)**2)
        
        if dist_to_fire <= 3:  # Turret close enough to fire
            # Check if turret is facing the fire
            angle_to_fire = np.arctan2(fire_grid_y - turret_center[1], fire_grid_x - turret_center[0])
            turret_facing_angle = -turret_yaw + np.pi/2  # Convert to standard angle
            
            angle_diff = abs(angle_to_fire - turret_facing_angle)
            angle_diff = min(angle_diff, 2*np.pi - angle_diff)  # Handle wrap-around
            
            if angle_diff < 0.5:  # Within ~30 degrees tolerance
                # Turret in position and facing fire - activate pump!
                if not pump_active:
                    turret_sock.sendto(b'1', (TURRET_UDP_IP, TURRET_UDP_PORT))
                    pump_active = True
                    print("Turret in position and aimed! Pump ON!")
                cv2.putText(frame, "PUMP ACTIVE!", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"Aiming... ({np.degrees(angle_diff):.1f} deg off)", (10, 180), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(frame, f"Moving to fire... (dist: {dist_to_fire:.1f})", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # ===== BALL PATHFINDING (push turret along path direction) =====
        # Place fake target BEHIND the turret (opposite to path direction)
        # This makes the ball push the turret in the path direction
        # AND aligns the turret's forward with the movement direction
        push_distance = 3
        fake_target_x = turret_center[0] - push_distance * path_dir_x
        fake_target_y = turret_center[1] - push_distance * path_dir_y
        fake_target = (fake_target_x, fake_target_y)
        
        # Pathfinding: ball to fake target (behind turret relative to path)
        bs = (max(0, min(grid_height-1, int(ball_center[1]))), max(0, min(grid_width-1, int(ball_center[0]))))
        ts = (max(0, min(grid_height-1, int(fake_target[1]))), max(0, min(grid_width-1, int(fake_target[0]))))
        
        path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[ts], cardinal=10, diagonal=14)
        
        if len(path_b2t) > 0:
            _, diagonaldown_path = simplify_path(path_b2t)
            
            if len(diagonaldown_path) >= 2:
                dy = diagonaldown_path[1][0] - ball_center[1]
                dx = diagonaldown_path[1][1] - ball_center[0]
                
                # Get ball yaw for sending
                if ball_idx.size > 0:
                    ball_yaw = get_2d_angle_from_corners(corners[ball_idx[0]])
                elif ball_last_corners is not None:
                    ball_yaw = get_2d_angle_from_corners(ball_last_corners)
                else:
                    ball_yaw = 0
                
                # Calculate angle from ball to turret
                angle_b2t = np.arctan2(turret_center[1] - ball_center[1], turret_center[0] - ball_center[0])
                
                # Send UDP command to ball
                next_target = np.array([dy, dx, compute_theta_send(ball_yaw), angle_b2t])
                sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                print(f"Fire mode - message: {next_target}")
        
        # Visualization
        img_h, img_w = frame.shape[:2]
        
        # Draw grid
        for i in range(grid_height + 1):
            y = int(round(i * img_h / grid_height))
            cv2.line(frame, (0, y), (img_w, y), (100, 100, 100), 1)
        for j in range(grid_width + 1):
            x = int(round(j * img_w / grid_width))
            cv2.line(frame, (x, 0), (x, img_h), (100, 100, 100), 1)
        
        # Draw turret path to fire (red dashed line effect)
        if len(turret_path_simplified) >= 1:
            turret_path_pixels = []
            for point in turret_path_simplified:
                x_pixel = int(point[1] * img_w / grid_width)
                y_pixel = int(point[0] * img_h / grid_height)
                turret_path_pixels.append([x_pixel, y_pixel])
            turret_path_pixels = np.array(turret_path_pixels, dtype=np.int32)
            cv2.polylines(frame, [turret_path_pixels], False, color=(0, 0, 255), thickness=3)  # Red for turret path
        
        # Draw ball path to fake target (orange)
        if len(path_b2t) > 0 and len(diagonaldown_path) >= 1:
            path_pixels = []
            for point in diagonaldown_path:
                x_pixel = int(point[1] * img_w / grid_width)
                y_pixel = int(point[0] * img_h / grid_height)
                path_pixels.append([x_pixel, y_pixel])
            path_pixels = np.array(path_pixels, dtype=np.int32)
            cv2.polylines(frame, [path_pixels], False, color=(255, 128, 0), thickness=2)  # Orange for ball path
        
        # Draw markers
        ball_pixel = (int(ball_center[0] * img_w / grid_width), int(ball_center[1] * img_h / grid_height))
        turret_pixel = (int(turret_center[0] * img_w / grid_width), int(turret_center[1] * img_h / grid_height))
        fake_pixel = (int(fake_target[0] * img_w / grid_width), int(fake_target[1] * img_h / grid_height))
        fire_pixel = (int(fire_grid_x * img_w / grid_width), int(fire_grid_y * img_h / grid_height))
        
        cv2.drawMarker(frame, ball_pixel, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 40, 2)
        cv2.drawMarker(frame, turret_pixel, (0, 128, 255), cv2.MARKER_STAR, 50, 2)  # Orange star for turret
        cv2.drawMarker(frame, fake_pixel, (255, 0, 255), cv2.MARKER_DIAMOND, 40, 2)  # Magenta for ball's fake target
        cv2.drawMarker(frame, fire_pixel, (0, 0, 255), cv2.MARKER_CROSS, 60, 3)  # Red cross at fire
        cv2.putText(frame, "FIRE", (fire_pixel[0]+10, fire_pixel[1]-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Draw turret forward direction arrow
        arrow_length = 50
        arrow_end = (int(turret_pixel[0] + arrow_length * turret_forward_x),
                    int(turret_pixel[1] + arrow_length * turret_forward_y))
        cv2.arrowedLine(frame, turret_pixel, arrow_end, (0, 128, 255), 3, tipLength=0.3)
        
        # Draw path direction arrow (cyan - shows which way turret should move)
        path_arrow_end = (int(turret_pixel[0] + arrow_length * path_dir_x),
                         int(turret_pixel[1] + arrow_length * path_dir_y))
        cv2.arrowedLine(frame, turret_pixel, path_arrow_end, (255, 255, 0), 2, tipLength=0.3)
        
        draw_in_range_status(frame, In_range)
        cv2.imshow('frame-image', frame)
    # ==================== END FIRE HANDLING LOOP ====================
        
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

        sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
        draw_in_range_status(frame, In_range)
        cv2.imshow('frame-image', frame)

    if quit_flag:
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit()
