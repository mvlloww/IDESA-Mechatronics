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

def get_phi1_angle(target_center_px, ball_center_px):
    """
    Calculate phi1: angle from vertical (from target center to ball center).
    Uses pixel coordinates for fine angular resolution (~0.1 degrees).
    
    Coordinate system:
    - 0° = ball is directly ABOVE target (negative Y direction in screen coords)
    - Positive phi1 = anticlockwise rotation from vertical up
    - Negative phi1 = clockwise rotation from vertical up
    
    Parameters:
    - target_center_px: (x, y) PIXEL coordinates of target ArUco center
    - ball_center_px: (x, y) PIXEL coordinates of ball ArUco center
    
    Returns angle in radians from vertical.
    """
    # Vector from target to ball (in pixels for fine resolution)
    dx = ball_center_px[0] - target_center_px[0]  # positive = ball is to the right
    dy = ball_center_px[1] - target_center_px[1]  # positive = ball is below (screen coords)
    
    # Calculate angle from vertical up (negative Y axis)
    # atan2(x, -y) gives angle from Y- axis with CCW positive
    phi1 = np.arctan2(dx, -dy)
    return phi1

def get_phi2_angle(target_center, next_waypoint):
    """
    Calculate phi2: required angle where ball should be to push target towards next waypoint.
    This is the OPPOSITE direction of where the target needs to go.
    
    The angle is snapped to the nearest 45 degree interval:
    0°, 45°, 90°, 135°, 180°, -135°, -90°, -45°
    
    Parameters:
    - target_center: (x, y) grid coordinates of target ArUco center
    - next_waypoint: (y, x) or (i, j) grid coordinates of next waypoint (note: path format is (i,j) = (y,x))
    
    Returns angle in radians snapped to 45 degree intervals.
    """
    # Vector from target to next waypoint (direction target should move)
    # Note: next_waypoint is in (i,j) = (y,x) format from pathfinding
    dx_to_waypoint = next_waypoint[1] - target_center[0]  # waypoint x - target x
    dy_to_waypoint = next_waypoint[0] - target_center[1]  # waypoint y - target y
    
    # Ball should be on the OPPOSITE side to push target towards waypoint
    dx_ball_needed = -dx_to_waypoint
    dy_ball_needed = -dy_to_waypoint
    
    # Calculate angle from vertical (same convention as phi1)
    raw_angle = np.arctan2(dx_ball_needed, -dy_ball_needed)
    
    # Snap to nearest 45 degree interval
    # 45 degrees = pi/4 radians
    snap_interval = np.pi / 4
    phi2 = round(raw_angle / snap_interval) * snap_interval
    
    # Normalize to [-pi, pi]
    if phi2 > np.pi:
        phi2 -= 2 * np.pi
    elif phi2 < -np.pi:
        phi2 += 2 * np.pi
    
    return phi2

def compute_phi(phi1, phi2):
    """
    Calculate phi: the angular difference between current ball position (phi1)
    and required ball position (phi2).
    
    Returns the shortest angular difference in radians.
    Positive = ball needs to move anticlockwise around target
    Negative = ball needs to move clockwise around target
    """
    phi = phi1 - phi2
    
    # Normalize to [-pi, pi] for shortest rotation
    while phi > np.pi:
        phi -= 2 * np.pi
    while phi < -np.pi:
        phi += 2 * np.pi
    
    return phi



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

# Target detection temporal smoothing (3 second timeout)
target_last_center = None      # Last known target center (grid coords)
target_last_corners = None     # Last known target corners
target_last_seen_time = None   # Timestamp when target was last detected
target_cache_timeout = 3.0     # Seconds before switching to new target

'''Temporary variables'''
quit_flag = False
mode = 'auto'  # can be 'auto' or 'manual'

''' Setup UDP communication '''
# Ball control UDP
UDP_IP = "138.38.229.206" #clearpass IP address for RPI 3B Model +
UDP_PORT = 50000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Crop Rotation True/False
In_range = {}

''' Main loop '''
while True:
    # Start the performance clock
    start = time.perf_counter()
    quit_flag = False

    while not quit_flag and mode == 'auto':
        # Capture current frame from the camera
        ret, frame = cap.read()

        # Skip frame if capture failed
        if not ret or frame is None:
            continue

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
                # Reset target cache when locking onto a new target
                target_last_center = None
                target_last_corners = None
                target_last_seen_time = None
                
                while position_status == False and not quit_flag:
                    ret, frame = cap.read()
                    
                    # Skip frame if capture failed
                    if not ret or frame is None:
                        continue

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
                    
                    if target_visible:
                        # Target detected - update cache and timestamp
                        target_idx = np.where(ids == keys)[0][0]
                        target_last_corners = corners[target_idx].copy()
                        target_last_center = get_center(corners[target_idx], frame, grid_width, grid_height)
                        target_last_seen_time = time.perf_counter()
                    elif target_last_center is not None and target_last_seen_time is not None:
                        # Target not visible - check if cache is still valid (within 3 seconds)
                        time_since_last_seen = time.perf_counter() - target_last_seen_time
                        if time_since_last_seen >= target_cache_timeout:
                            # Cache expired - break out to find new target
                            print(f"Target ID {keys} lost for {time_since_last_seen:.1f}s - switching to new target...")
                            target_last_center = None
                            target_last_corners = None
                            target_last_seen_time = None
                            break
                        else:
                            # Use cached position
                            print(f"Target ID {keys} not visible - using cached position ({time_since_last_seen:.1f}s / {target_cache_timeout}s)")
                    else:
                        # No cache available - break immediately
                        print(f"Target ID {keys} lost - no cached position available, switching to new target...")
                        break
                    
                    # Determine if we have a valid target position (either detected or cached)
                    target_position_available = target_visible or (target_last_center is not None and target_last_seen_time is not None 
                                                                    and (time.perf_counter() - target_last_seen_time) < target_cache_timeout)
                    
                    # Check if ball is visible in current frame
                    ball_visible = ids is not None and len(ids) > 0 and ball_id in ids.flatten()
                    
                    if target_position_available and ball_visible:
                        # Ball detected - update tracking
                        ball_missing_frames = 0
                        ball_idx_temp = np.where(ids == ball_id)[0][0]
                        ball_last_corners = corners[ball_idx_temp].copy()
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        # Get target center - use detected if visible, otherwise use cached
                        if target_visible:
                            center_key = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        else:
                            center_key = target_last_center  # Use cached target position
                        center_ball = get_center(corners[np.where(ids==ball_id)[0][0]], frame, grid_width, grid_height)
                        ball_last_center = center_ball  # Update last known center
                        # Print grid positions of ball and target
                        cached_str = "" if target_visible else " (CACHED)"
                        print(f"Ball (ID {ball_id}) grid position: {center_ball}, Target (ID {keys}) grid position{cached_str}: {center_key}")
                    elif target_position_available and ball_last_center is not None and ball_missing_frames < ball_max_missing:
                        # Ball NOT detected but we have recent data - use last known position
                        ball_missing_frames += 1
                        print(f"Ball missing frame {ball_missing_frames}/{ball_max_missing} - using last known position")
                        # Only draw markers if there are any detected
                        if ids is not None and len(ids) > 0:
                            out = aruco.drawDetectedMarkers(frame, corners, ids)
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        # Get target center - use detected if visible, otherwise use cached
                        if target_visible:
                            center_key = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        else:
                            center_key = target_last_center  # Use cached target position
                        center_ball = ball_last_center  # Use last known ball center
                        # Print grid positions of ball and target
                        target_cached_str = "" if target_visible else " (CACHED)"
                        print(f"Ball (ID {ball_id}) grid position (CACHED): {center_ball}, Target (ID {keys}) grid position{target_cached_str}: {center_key}")
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
                        if ids is not None and len(ids) > 0:
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

                        # Only update obstacles if we have detected markers
                        if ids is not None and len(ids) > 0:
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
                                    if ids is not None and len(ids) > 0:
                                        ball_idx = np.where(ids == ball_id)[0]
                                    else:
                                        ball_idx = np.array([])  # Empty array when no markers detected
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
                                # PUSHING MODE - ball is close enough to push target
                                if len(path_t2e) > 0:
                                    _, diagonaldown_path_t2e = simplify_path(path_t2e)
                                else:
                                    diagonaldown_path_t2e = []

                                # convert to dx, dy instructions for UDP sending
                                # Check if path has at least 2 points before accessing [1]
                                if len(diagonaldown_path_t2e) < 1:
                                    continue
                                
                                # Get ball corners for phi1 calculation - use detected or cached
                                if ids is not None and len(ids) > 0:
                                    ball_idx_phi = np.where(ids == ball_id)[0]
                                else:
                                    ball_idx_phi = np.array([])
                                if ball_idx_phi.size > 0:
                                    ball_corners_for_phi = corners[ball_idx_phi[0]]
                                elif ball_last_corners is not None:
                                    ball_corners_for_phi = ball_last_corners
                                else:
                                    ball_corners_for_phi = None
                                
                                # Get target corners for phi1 calculation - use detected or cached
                                if target_visible:
                                    target_corners_for_phi = corners[np.where(ids == keys)[0][0]]
                                elif target_last_corners is not None:
                                    target_corners_for_phi = target_last_corners
                                else:
                                    target_corners_for_phi = None
                                
                                # Calculate phi1 using PIXEL coordinates for fine resolution
                                if ball_corners_for_phi is not None and target_corners_for_phi is not None:
                                    # Get pixel centers from corners
                                    ball_pts = ball_corners_for_phi.reshape((4, 2))
                                    ball_center_px = ball_pts.mean(axis=0)  # (x, y) in pixels
                                    target_pts = target_corners_for_phi.reshape((4, 2))
                                    target_center_px = target_pts.mean(axis=0)  # (x, y) in pixels
                                    
                                    # phi1: angle from vertical (from target center) to ball center using pixels
                                    phi1 = get_phi1_angle(target_center_px, ball_center_px)
                                else:
                                    # Fallback to grid-based if corners not available
                                    phi1 = get_phi1_angle(
                                        (target_start[0] * frame.shape[1] / grid_width, target_start[1] * frame.shape[0] / grid_height),
                                        (ball_start[0] * frame.shape[1] / grid_width, ball_start[1] * frame.shape[0] / grid_height)
                                    )
                                
                                # phi2: required angle where ball should be (snapped to 45 deg intervals)
                                # Use next waypoint from path_t2e (target to endpoint path)
                                next_waypoint = diagonaldown_path_t2e[1]  # (i, j) = (y, x) format
                                phi2 = get_phi2_angle(target_start, next_waypoint)
                                
                                # phi: difference between current and required ball position
                                phi = compute_phi(phi1, phi2)
                                
                                # Print phi values for debugging
                                print(f"PUSHING MODE - phi1: {np.degrees(phi1):.1f}°, phi2: {np.degrees(phi2):.1f}°, phi: {np.degrees(phi):.1f}°")
                                
                                # ===== CONTROL SYSTEM FOR BALL MOVEMENT =====
                                # Control gains (tune these values)
                                k1 = 1.0  # Gain for pushing direction (towards waypoint)
                                k2 = 0.5  # Gain for phi correction (repositioning around target)
                                
                                # dx1, dy1: Direction to push target towards waypoint
                                dy1 = diagonaldown_path_t2e[1][0] - target_start[1]
                                dx1 = diagonaldown_path_t2e[1][1] - target_start[0]
                                dy1, dx1 = bouncing_ball(dy1, dx1, ball_start)
                                
                                # dx2, dy2: Direction to move ball to reduce phi to 0
                                # Calculate using trigonometry - positions on a circle around target
                                # Get the radius from target to ball (in grid units)
                                radius_ball_to_target = np.sqrt((ball_start[0] - target_start[0])**2 + 
                                                                 (ball_start[1] - target_start[1])**2)
                                
                                # Current ball position relative to target (at angle phi1)
                                # Using convention: x = r*sin(phi), y = -r*cos(phi) where 0° is up
                                ball_rel_x_current = radius_ball_to_target * np.sin(phi1)
                                ball_rel_y_current = -radius_ball_to_target * np.cos(phi1)
                                
                                # Desired ball position relative to target (at angle phi2)
                                ball_rel_x_desired = radius_ball_to_target * np.sin(phi2)
                                ball_rel_y_desired = -radius_ball_to_target * np.cos(phi2)
                                
                                # dx2, dy2: Vector from current to desired position
                                # Note: In grid coords, x is horizontal (columns), y is vertical (rows)
                                dx2 = ball_rel_x_desired - ball_rel_x_current
                                dy2 = ball_rel_y_desired - ball_rel_y_current
                                
                                # Combine dx1/dy1 (push direction) with dx2/dy2 (phi correction)
                                dx_out = (k1 * dx1) + (k2 * dx2)
                                dy_out = (k1 * dy1) + (k2 * dy2)
                                
                                # Print control values for debugging
                                print(f"  dx1={dx1:.2f}, dy1={dy1:.2f} | dx2={dx2:.2f}, dy2={dy2:.2f} | dx_out={dx_out:.2f}, dy_out={dy_out:.2f}")

                                # Get ball corners - use detected or cached
                                if ids is not None and len(ids) > 0:
                                    ball_idx = np.where(ids == ball_id)[0]
                                else:
                                    ball_idx = np.array([])  # Empty array when no markers detected
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

                                    # UDP sending with combined control output
                                    next_target = np.array([dy_out, dx_out, compute_theta_send(yaw), angle_b2t])
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
                                
                                # Visualize phi angles in pushing mode
                                phi1_deg = np.degrees(phi1)
                                phi2_deg = np.degrees(phi2)
                                phi_deg = np.degrees(phi)
                                # Draw phi1 arrow (cyan) - from target to ball direction
                                phi_arrow_length = 50
                                phi1_end = (int(target_pixel[0] + phi_arrow_length * np.sin(phi1)),
                                           int(target_pixel[1] - phi_arrow_length * np.cos(phi1)))
                                cv2.arrowedLine(frame, target_pixel, phi1_end, (255, 255, 0), 2, tipLength=0.3)  # Cyan
                                # Draw phi2 arrow (magenta) - required ball position direction
                                phi2_end = (int(target_pixel[0] + phi_arrow_length * np.sin(phi2)),
                                           int(target_pixel[1] - phi_arrow_length * np.cos(phi2)))
                                cv2.arrowedLine(frame, target_pixel, phi2_end, (255, 0, 255), 2, tipLength=0.3)  # Magenta
                                
                                # Display PUSHING MODE phi panel on top right
                                panel_x = img_w - 280
                                panel_y = 10
                                # Draw semi-transparent background rectangle for better visibility (expanded for control equation)
                                cv2.rectangle(frame, (panel_x - 10, panel_y - 5), (img_w - 10, panel_y + 230), (0, 0, 0), -1)
                                cv2.rectangle(frame, (panel_x - 10, panel_y - 5), (img_w - 10, panel_y + 230), (255, 255, 255), 2)
                                # Header
                                cv2.putText(frame, "PUSHING MODE", (panel_x, panel_y + 20), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)  # Yellow header
                                # phi1 - current ball angle (cyan)
                                cv2.putText(frame, f"phi1: {phi1_deg:+7.1f} deg", (panel_x, panel_y + 50), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                                # phi2 - required angle (magenta)
                                cv2.putText(frame, f"phi2: {phi2_deg:+7.1f} deg", (panel_x, panel_y + 75), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                                # phi - difference (green)
                                cv2.putText(frame, f"phi:  {phi_deg:+7.1f} deg", (panel_x, panel_y + 100), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                
                                # Control equation display
                                cv2.putText(frame, "--- Control ---", (panel_x, panel_y + 125), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                                # dx equation
                                cv2.putText(frame, f"dx: ({k1:.1f}*{dx1:+.1f})+({k2:.1f}*{dx2:+.1f})", (panel_x, panel_y + 150), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
                                cv2.putText(frame, f"   = {dx_out:+.2f}", (panel_x, panel_y + 170), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 2)
                                # dy equation
                                cv2.putText(frame, f"dy: ({k1:.1f}*{dy1:+.1f})+({k2:.1f}*{dy2:+.1f})", (panel_x, panel_y + 195), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
                                cv2.putText(frame, f"   = {dy_out:+.2f}", (panel_x, panel_y + 215), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 2)
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
