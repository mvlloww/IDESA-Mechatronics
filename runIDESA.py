# source ./IDESA/bin/activate

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

def draw_fire_alert(frame, target_id=None):
    """Display 'Target X is on fire!' text in red at the top of the video feed."""
    img_h, img_w = frame.shape[:2]
    # Calculate text size for centering horizontally
    if target_id is not None:
        text = f"Target {target_id} is on fire!"
    else:
        text = "FIRE!"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3.0
    thickness = 10
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    # Center horizontally, position at top
    x = (img_w - text_width) // 2
    y = text_height + 50  # 50 pixels from top
    # Draw black outline for visibility
    cv2.putText(frame, text, (x, y), font, font_scale, (0, 0, 0), thickness + 5)
    # Draw the text in red (BGR: 0, 0, 255)
    cv2.putText(frame, text, (x, y), font, font_scale, (0, 0, 255), thickness)

def draw_firefighted_alert(frame):
    """Display large Fire Extinguished! text in green at the top of the video feed."""
    img_h, img_w = frame.shape[:2]
    # Calculate text size for centering horizontally
    text = "Fire Extinguished!"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3.0
    thickness = 10
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    # Center horizontally, position at top
    x = (img_w - text_width) // 2
    y = text_height + 50  # 50 pixels from top
    # Draw black outline for visibility
    cv2.putText(frame, text, (x, y), font, font_scale, (0, 0, 0), thickness + 5)
    # Draw the text in green (BGR: 0, 255, 0)
    cv2.putText(frame, text, (x, y), font, font_scale, (0, 255, 0), thickness)

def send_fire_udp(value, ip, port):
    """Send a boolean byte (0 or 1) over UDP to the fire system."""
    payload = bytes([1 if value else 0])
    try:
        fire_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sent = fire_sock.sendto(payload, (ip, port))
        print(f"Fire UDP: Sent {value} to {ip}:{port} ({sent} bytes)")
        fire_sock.close()
    except socket.error as e:
        print(f"Fire UDP send error: {e}")

def draw_fire_udp_debug(frame, dx, dy, theta_deg, state):
    """Draw debug panel showing UDP values being sent during fire state."""
    img_h, img_w = frame.shape[:2]
    
    # Panel position (top left, below any fire alert text)
    panel_x = 10
    panel_y = 120
    
    # Draw semi-transparent background
    cv2.rectangle(frame, (panel_x - 5, panel_y - 5), (panel_x + 280, panel_y + 120), (0, 0, 0), -1)
    cv2.rectangle(frame, (panel_x - 5, panel_y - 5), (panel_x + 280, panel_y + 120), (0, 165, 255), 2)  # Orange border
    
    # Header
    cv2.putText(frame, "FIRE UDP DEBUG", (panel_x, panel_y + 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)  # Orange
    
    # State
    cv2.putText(frame, f"State: {state}", (panel_x, panel_y + 45), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # dx value
    cv2.putText(frame, f"dx: {dx:+8.1f}", (panel_x, panel_y + 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)  # Light blue
    
    # dy value
    cv2.putText(frame, f"dy: {dy:+8.1f}", (panel_x, panel_y + 95), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 200), 2)  # Light green
    
    # theta value
    cv2.putText(frame, f"theta: {theta_deg:+7.1f} deg", (panel_x, panel_y + 115), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)  # Yellow

def draw_wasd_keys(frame, pressed_key=None, dy=0, dx=0):
    """Draw WASD key visual on bottom right of frame. Highlight pressed key and show dx/dy values."""
    img_h, img_w = frame.shape[:2]
    
    # Key box dimensions
    key_size = 50
    gap = 5
    
    # Position (bottom right)
    base_x = img_w - (3 * key_size + 2 * gap) - 20  # 3 keys wide
    base_y = img_h - (2 * key_size + gap) - 60      # 2 keys tall + space for text
    
    # Key positions: W on top, A-S-D on bottom row
    keys = {
        'W': (base_x + key_size + gap, base_y),
        'A': (base_x, base_y + key_size + gap),
        'S': (base_x + key_size + gap, base_y + key_size + gap),
        'D': (base_x + 2 * (key_size + gap), base_y + key_size + gap)
    }
    
    # Colors
    color_normal = (80, 80, 80)      # Dark gray
    color_pressed = (0, 255, 0)      # Green when pressed
    color_text = (255, 255, 255)     # White text
    
    # Draw each key
    for key_char, (kx, ky) in keys.items():
        # Check if this key is pressed
        is_pressed = (pressed_key == key_char.lower())
        box_color = color_pressed if is_pressed else color_normal
        thickness = -1 if is_pressed else 2  # Filled if pressed, outline if not
        
        # Draw key box
        cv2.rectangle(frame, (kx, ky), (kx + key_size, ky + key_size), box_color, thickness)
        if not is_pressed:
            # Draw filled dark background for non-pressed keys
            cv2.rectangle(frame, (kx + 2, ky + 2), (kx + key_size - 2, ky + key_size - 2), (40, 40, 40), -1)
        
        # Draw key letter
        text_color = (0, 0, 0) if is_pressed else color_text
        text_x = kx + key_size // 2 - 10
        text_y = ky + key_size // 2 + 10
        cv2.putText(frame, key_char, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
    
    # Draw dx/dy values below the keys
    value_y = base_y + 2 * (key_size + gap) + 25
    
    # Show which value is active based on pressed key
    if pressed_key in ['w', 's']:
        value_text = f"dy = {int(dy)}"
        value_color = (0, 255, 255)  # Yellow for dy
    elif pressed_key in ['a', 'd']:
        value_text = f"dx = {int(dx)}"
        value_color = (255, 255, 0)  # Cyan for dx
    else:
        value_text = "dx=0  dy=0"
        value_color = (150, 150, 150)  # Gray when idle
    
    # Draw value text with outline
    cv2.putText(frame, value_text, (base_x, value_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
    cv2.putText(frame, value_text, (base_x, value_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, value_color, 2)
    
    # Draw "MANUAL MODE" label
    label_y = base_y - 15
    cv2.putText(frame, "MANUAL MODE", (base_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
    cv2.putText(frame, "MANUAL MODE", (base_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

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
    return grid
def rotate_dict(d, k=1):
    """
    Rotate endpoint VALUES so each marker gets a new destination.
    k=1 means each marker moves to the next endpoint (shift by 1).
    
    Example with k=1:
    Before: {2:[16,10], 4:[6,30], 6:[16,36]}
    After:  {2:[16,36], 4:[16,10], 6:[6,30]}  (marker 2 now goes where 6 was, etc.)
    """
    keys = list(d.keys())
    values = list(d.values())
    n = len(values)
    k = k % n   # safety for large shifts
    # Rotate values: shift by k positions
    rotated_values = values[-k:] + values[:-k]
    # Reassign rotated values to original keys
    return dict(zip(keys, rotated_values))
def compute_theta_send(theta):
    theta_send = -theta
    return theta_send
def bouncing_ball(dy, dx, ball_start):
    '''bouncing ball function - placeholder for boundary handling'''
    return dy, dx

def get_turret_endpoint(turret_corners, frame, grid_width, grid_height, distance_grid_units):
    """
    Calculate endpoint position that is 'distance_grid_units' north of the turret
    based on its orientation.
    
    'North' is defined as perpendicular to the top edge of the ArUco marker,
    pointing away from the marker (the forward direction).
    
    Parameters:
    - turret_corners: ArUco marker corners [top-left, top-right, bottom-right, bottom-left]
    - frame: current video frame (for dimensions)
    - grid_width, grid_height: grid dimensions
    - distance_grid_units: how many grid squares north of turret
    
    Returns:
    - (endpoint_grid_x, endpoint_grid_y): grid coordinates of the endpoint
    - (endpoint_pixel_x, endpoint_pixel_y): pixel coordinates of the endpoint
    """
    img_h, img_w = frame.shape[:2]
    pts = turret_corners.reshape((4, 2))  # In pixel coordinates
    
    # Get top-left and top-right corners (indices 0 and 1)
    top_left = pts[0]
    top_right = pts[1]
    
    # Calculate center of top edge (in pixels)
    top_center_px = (top_left + top_right) / 2
    
    # Vector from top-left to top-right (top edge direction)
    top_edge = top_right - top_left
    
    # Forward direction is perpendicular to top edge (90° CCW from top edge)
    # Rotate 90° CCW: (x, y) -> (-y, x)
    forward_x = -top_edge[1]
    forward_y = top_edge[0]
    
    # Normalize the forward vector
    forward_length = np.sqrt(forward_x**2 + forward_y**2)
    if forward_length > 0:
        forward_x /= forward_length
        forward_y /= forward_length
    
    # Convert distance from grid units to pixels
    pixels_per_grid_x = img_w / grid_width
    pixels_per_grid_y = img_h / grid_height
    # Use average for distance calculation
    pixels_per_grid = (pixels_per_grid_x + pixels_per_grid_y) / 2
    distance_pixels = distance_grid_units * pixels_per_grid
    
    # Calculate endpoint in pixels
    endpoint_pixel_x = top_center_px[0] + forward_x * distance_pixels
    endpoint_pixel_y = top_center_px[1] + forward_y * distance_pixels
    
    # Convert to grid coordinates
    endpoint_grid_x = int(endpoint_pixel_x / img_w * grid_width)
    endpoint_grid_y = int(endpoint_pixel_y / img_h * grid_height)
    
    # Clip to grid bounds
    endpoint_grid_x = max(0, min(grid_width - 1, endpoint_grid_x))
    endpoint_grid_y = max(0, min(grid_height - 1, endpoint_grid_y))
    
    return (endpoint_grid_x, endpoint_grid_y), (int(endpoint_pixel_x), int(endpoint_pixel_y))

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
# This is a library for roguelike game development (used for pathfinding)
import tcod
import os
import socket   #communicate over the network
import struct
import math


''' ====================== CONFIGURATION CONSTANTS ====================== '''
# === ArUco Marker Settings ===
MARKER_SIZE = 40                    # ArUco marker size in mm
BALL_ID = 9                         # Ball ArUco marker ID
TURRET_ID = 10                      # Water turret ArUco marker ID
TURRET_ENDPOINT_DISTANCE = 4        # Grid squares north of turret for endpoint
END_POINTS = {1:[15,8], 2:[5,20], 3:[15,34]}  # Target endpoints {marker_id: [y, x]}
#END_POINTS = {1:[18,8], 2:[5,17], 3:[15,34], 4:[10,12], 5:[5,23], 6:[12,30]}

# === Grid Settings ===
GRID_HEIGHT = 40                    # Grid rows
GRID_WIDTH = 80                     # Grid columns
OBSTACLE_RADIUS = 4               # Radius around obstacles (grid units)

# === Camera Settings ===
CAMERA_WIDTH = 1920                 # Camera resolution width
CAMERA_HEIGHT = 1080                # Camera resolution height
INITIAL_EXPOSURE = -10              # Initial exposure value (-13 to 0)

# === UDP Communication ===
UDP_IP = "138.38.229.206"           # Raspberry Pi IP address
UDP_PORT = 50000                    # UDP port number
FIRE_UDP_IP = "138.38.226.213"      # Fire system UDP IP address
FIRE_UDP_PORT = 50001               # Fire system UDP port

# === Fire Mode Timing ===
FIRE_INITIAL_WAIT_MIN = 5        # Minimum seconds to wait before starting fire response
FIRE_INITIAL_WAIT_MAX = 30        # Maximum seconds to wait before starting fire response
FIRE_WAIT_BEFORE_ON = 5.0           # Seconds to wait before sending UDP 1 (turret on)
FIRE_WAIT_BEFORE_OFF = 2.0          # Seconds after UDP 1 before sending UDP 0 (turret off)
FIRE_WAIT_AFTER_OFF = 3.0           # Seconds after UDP 0 before showing "Fire Extinguished!"
FIRE_TARGET_TOLERANCE = 1           # Grid cells tolerance for reaching turret endpoint

# === Ball Tracking ===
BALL_CACHE_TIMEOUT = 2           # Seconds before ball cache expires
TARGET_CACHE_TIMEOUT = 2          # Seconds before target cache expires

# === Pathfinding & Control ===
BALL_TARGET_DISTANCE = 4            # Distance threshold for pushing mode
PHI_THRESHOLD_DEG = 60              # Phi alignment threshold (degrees)
ATTACK_DISTANCE = 4                 # Distance ball approaches from opposite side
K1_PUSH_GAIN = 0.8                  # Gain for pushing direction
K2_PHI_GAIN = 1.2                   # Gain for phi correction
PUSHING_MODE_DEBOUNCE = 2         # Seconds to lock into pushing mode before allowing exit
PATH_LENGTH_MULTIPLIER = 1.0 
DIAGONAL_VAR = 14       # Multiplier for path length speed scaling (1.0 = use raw path length)

# === Manual Mode ===
MANUAL_KEY_TIMEOUT = 0.5            # Key hold timeout (seconds)
MANUAL_MOVE_SPEED = 50              # Movement magnitude for WASD controls

# === ArUco Detection Tuning ===
ARUCO_MIN_PERIMETER_RATE = 0.01     # Lower = detect smaller markers (default 0.03)
ARUCO_MAX_PERIMETER_RATE = 4.0
ARUCO_THRESH_WIN_MIN = 3
ARUCO_THRESH_WIN_MAX = 23
ARUCO_THRESH_WIN_STEP = 10
ARUCO_THRESH_CONSTANT = 7
ARUCO_CORNER_REFINE_WIN = 5
ARUCO_CORNER_REFINE_ITER = 30
ARUCO_ERROR_CORRECTION = 0.6
''' ===================================================================== '''


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

# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# Define the aruco detection parameters (tuned for distant markers)
parameters = aruco.DetectorParameters()
parameters.minMarkerPerimeterRate = ARUCO_MIN_PERIMETER_RATE
parameters.maxMarkerPerimeterRate = ARUCO_MAX_PERIMETER_RATE
parameters.adaptiveThreshWinSizeMin = ARUCO_THRESH_WIN_MIN
parameters.adaptiveThreshWinSizeMax = ARUCO_THRESH_WIN_MAX
parameters.adaptiveThreshWinSizeStep = ARUCO_THRESH_WIN_STEP
parameters.adaptiveThreshConstant = ARUCO_THRESH_CONSTANT
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = ARUCO_CORNER_REFINE_WIN
parameters.cornerRefinementMaxIterations = ARUCO_CORNER_REFINE_ITER
parameters.errorCorrectionRate = ARUCO_ERROR_CORRECTION

# CAP_DSHOW
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Exposure control settings
exposure_value = INITIAL_EXPOSURE
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Disable auto-exposure
cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
print(f"Initial exposure set to: {exposure_value}")

#Create window
cv2.namedWindow("frame-image", cv2.WINDOW_NORMAL)
#Position the window at x=0,y=100 on the computer screen
cv2.moveWindow("frame-image",0,100)

'''Initiate grid for pathfinding'''
# Create grid all initialized to 1
grid = np.ones((GRID_HEIGHT, GRID_WIDTH), np.int8)
grid_height, grid_width = grid.shape

# Precompute grid coordinates for masking
y_coords, x_coords = np.ogrid[:grid_height, :grid_width]

# Working copies of config (can be modified at runtime)
end_points = END_POINTS.copy()
ball_id = BALL_ID
radius = OBSTACLE_RADIUS

# Ball detection temporal smoothing
ball_last_corners = None  # Last known ball corners
ball_last_center = None   # Last known ball center (grid coords)
ball_last_yaw = 0.0       # Last known ball yaw angle
ball_last_seen_time = None  # Timestamp when ball was last detected
ball_cache_timeout = BALL_CACHE_TIMEOUT

# Target detection temporal smoothing
target_last_center = None      # Last known target center (grid coords)
target_last_corners = None     # Last known target corners
target_last_seen_time = None   # Timestamp when target was last detected
target_cache_timeout = TARGET_CACHE_TIMEOUT

'''Temporary variables'''
quit_flag = False
mode = 'auto'  # can be 'auto' or 'manual'
isFire = False  # Fire alert state - becomes true after pending timer completes
fire_pending = False  # Fire is pending (waiting for random timer to complete)

# Fire mode state variables
fire_target_id = None              # ID of target that is "on fire"
fire_target_reached = False        # Has the fire target reached turret endpoint
fire_reached_time = None           # Time when fire target reached turret endpoint
fire_sequence_state = 'inactive'   # States: 'inactive', 'initial_wait', 'active', 'waiting_5s', 'sent_1', 'sent_0', 'firefighted'
fire_turret_endpoint_grid = None   # Cached turret endpoint in grid coords
fire_turret_endpoint_pixel = None  # Cached turret endpoint in pixel coords
fire_initial_wait_duration = 0.0   # Random duration for initial wait (30-60s)
fire_initial_wait_start = None     # Time when initial wait started

''' Setup UDP communication '''
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Crop Rotation True/False
In_range = {}

''' Main loop '''
while True:
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
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  
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
        if key == ord('f'):
            if not isFire and not fire_pending:
                # Start fire pending mode - wait random time before activating
                import random
                fire_pending = True
                fire_target_id = None
                fire_target_reached = False
                fire_reached_time = None
                fire_turret_endpoint_grid = None
                fire_turret_endpoint_pixel = None
                fire_sequence_state = 'inactive'
                # Set random initial wait duration
                fire_initial_wait_duration = random.uniform(FIRE_INITIAL_WAIT_MIN, FIRE_INITIAL_WAIT_MAX)
                fire_initial_wait_start = time.perf_counter()
                # Select random target from available endpoints
                available_targets = list(end_points.keys())
                if available_targets:
                    fire_target_id = random.choice(available_targets)
                    print(f"Fire pending: Target ID {fire_target_id} will catch fire in {fire_initial_wait_duration:.1f} seconds...")
                else:
                    print("Fire pending: No targets available!")
                    fire_pending = False
            elif isFire or fire_pending:
                # Cancel fire mode
                isFire = False
                fire_pending = False
                fire_target_id = None
                fire_target_reached = False
                fire_reached_time = None
                fire_sequence_state = 'inactive'
                fire_turret_endpoint_grid = None
                fire_turret_endpoint_pixel = None
                fire_initial_wait_start = None
                print(f"Fire alert: DEACTIVATED")
        if key == ord('=') or key == ord('+'):  # Increase exposure (brighter)
            exposure_value = min(0, exposure_value + 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
            print(f"Exposure: {exposure_value}")
        if key == ord('-') or key == ord('_'):  # Decrease exposure (darker)
            exposure_value = max(-13, exposure_value - 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
            print(f"Exposure: {exposure_value}")

        # If markers are detected, draw them, estimate pose and overlay axes
        if ids is not None and len(ids) > 0:
            try:
                print("DEBUG: Detected IDs:", ids.flatten().tolist())
            except Exception:
                print("DEBUG: Detected IDs present, but could not print list.")
            # Draw detected markers on the frame
            out = aruco.drawDetectedMarkers(frame, corners, ids)
            # Calculate the pose of each detected marker
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, CM, dist_coef)

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

                # Get path from target to end point and ball to target
                # tcod.path.path2d expects sequences of (i,j) pairs
                sp = (int(start_point[1]), int(start_point[0])) # Note the (y,x) order for (i,j)
                target_endpoint = get_endpoint_xy(end_points.get(marker_id))
                if target_endpoint is not None:
                    ep = (int(target_endpoint[0]), int(target_endpoint[1]))
                    path_t2e = tcod.path.path2d(cost=grid, start_points=[sp], end_points=[ep], cardinal=10, diagonal=DIAGONAL_VAR)
                    ball_idx = np.where(ids == ball_id)[0]
                    if ball_idx.size > 0:
                        ball_center = get_center(corners[ball_idx[0]], frame, grid_width, grid_height)
                        bs = (int(ball_center[1]), int(ball_center[0]))
                        path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[sp], cardinal=10, diagonal=DIAGONAL_VAR)
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
                # Pushing mode debouncing state
                is_in_pushing_mode = False
                pushing_mode_enter_time = None
                
                while position_status == False and not quit_flag:
                    ret, frame = cap.read()
                    
                    # Skip frame if capture failed
                    if not ret or frame is None:
                        continue

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                    
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
                    if key == ord('f'):
                        if not isFire and not fire_pending:
                            # Start fire pending mode
                            import random
                            fire_pending = True
                            fire_target_id = None
                            fire_target_reached = False
                            fire_reached_time = None
                            fire_turret_endpoint_grid = None
                            fire_turret_endpoint_pixel = None
                            fire_sequence_state = 'inactive'
                            fire_initial_wait_duration = random.uniform(FIRE_INITIAL_WAIT_MIN, FIRE_INITIAL_WAIT_MAX)
                            fire_initial_wait_start = time.perf_counter()
                            available_targets = list(end_points.keys())
                            if available_targets:
                                fire_target_id = random.choice(available_targets)
                                print(f"Fire pending: Target ID {fire_target_id} will catch fire in {fire_initial_wait_duration:.1f} seconds...")
                            else:
                                print("Fire pending: No targets available!")
                                fire_pending = False
                        elif isFire or fire_pending:
                            # Cancel fire mode
                            isFire = False
                            fire_pending = False
                            fire_target_id = None
                            fire_target_reached = False
                            fire_reached_time = None
                            fire_sequence_state = 'inactive'
                            fire_turret_endpoint_grid = None
                            fire_turret_endpoint_pixel = None
                            fire_initial_wait_start = None
                            print(f"Fire alert: DEACTIVATED")

                    # Handle fire pending timer (waiting before fire activates)
                    if fire_pending and fire_initial_wait_start is not None:
                        current_time = time.perf_counter()
                        elapsed = current_time - fire_initial_wait_start
                        if elapsed >= fire_initial_wait_duration:
                            # Timer complete - activate fire!
                            fire_pending = False
                            isFire = True
                            fire_sequence_state = 'active'
                            print(f"Fire alert: ACTIVATED - Target {fire_target_id} is on fire!")

                    # If fire alert is active, handle fire fighting mode
                    if isFire:
                        img_h, img_w = frame.shape[:2]
                        
                        # Track UDP values for debug display
                        fire_udp_dx = 0
                        fire_udp_dy = 0
                        fire_udp_theta_deg = 0
                        
                        # Handle fire sequence state machine (for when target reached turret endpoint)
                        current_time = time.perf_counter()
                        
                        if fire_sequence_state == 'waiting_5s' and fire_reached_time is not None:
                            # Keep sending 0 dx/dy during waiting period
                            if ids is not None and ball_id in ids.flatten():
                                ball_idx_wait = np.where(ids == ball_id)[0][0]
                                yaw_wait = get_2d_angle_from_corners(corners[ball_idx_wait])
                                fire_udp_dx = 0
                                fire_udp_dy = 0
                                fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_wait))
                                sock.sendto(struct.pack('<iif', 0, 0, float(compute_theta_send(yaw_wait))), (UDP_IP, UDP_PORT))
                            if current_time - fire_reached_time >= FIRE_WAIT_BEFORE_ON:
                                # Send UDP 1
                                send_fire_udp(1, FIRE_UDP_IP, FIRE_UDP_PORT)
                                fire_sequence_state = 'sent_1'
                                fire_reached_time = current_time
                        elif fire_sequence_state == 'sent_1' and fire_reached_time is not None:
                            # Keep sending 0 dx/dy during waiting period
                            if ids is not None and ball_id in ids.flatten():
                                ball_idx_wait = np.where(ids == ball_id)[0][0]
                                yaw_wait = get_2d_angle_from_corners(corners[ball_idx_wait])
                                fire_udp_dx = 0
                                fire_udp_dy = 0
                                fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_wait))
                                sock.sendto(struct.pack('<iif', 0, 0, float(compute_theta_send(yaw_wait))), (UDP_IP, UDP_PORT))
                            if current_time - fire_reached_time >= FIRE_WAIT_BEFORE_OFF:
                                # Send UDP 0
                                send_fire_udp(0, FIRE_UDP_IP, FIRE_UDP_PORT)
                                fire_sequence_state = 'sent_0'
                                fire_reached_time = current_time
                        elif fire_sequence_state == 'sent_0' and fire_reached_time is not None:
                            # Keep sending 0 dx/dy during waiting period
                            if ids is not None and ball_id in ids.flatten():
                                ball_idx_wait = np.where(ids == ball_id)[0][0]
                                yaw_wait = get_2d_angle_from_corners(corners[ball_idx_wait])
                                fire_udp_dx = 0
                                fire_udp_dy = 0
                                fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_wait))
                                sock.sendto(struct.pack('<iif', 0, 0, float(compute_theta_send(yaw_wait))), (UDP_IP, UDP_PORT))
                            if current_time - fire_reached_time >= FIRE_WAIT_AFTER_OFF:
                                fire_sequence_state = 'firefighted'
                        elif fire_sequence_state == 'firefighted':
                            # Keep sending 0 dx/dy during firefighted state
                            if ids is not None and ball_id in ids.flatten():
                                ball_idx_wait = np.where(ids == ball_id)[0][0]
                                yaw_wait = get_2d_angle_from_corners(corners[ball_idx_wait])
                                fire_udp_dx = 0
                                fire_udp_dy = 0
                                fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_wait))
                                sock.sendto(struct.pack('<iif', 0, 0, float(compute_theta_send(yaw_wait))), (UDP_IP, UDP_PORT))
                        
                        # Display appropriate text based on state
                        if fire_sequence_state == 'firefighted':
                            draw_firefighted_alert(frame)
                            # Draw debug panel for firefighted state
                            draw_fire_udp_debug(frame, fire_udp_dx, fire_udp_dy, fire_udp_theta_deg, fire_sequence_state)
                            cv2.imshow('frame-image', frame)
                            continue
                        else:
                            draw_fire_alert(frame, fire_target_id)
                        
                        # Check if turret is detected and get its endpoint
                        turret_idx_fire = np.where(ids == TURRET_ID)[0] if ids is not None else np.array([])
                        if turret_idx_fire.size > 0:
                            turret_corners_fire = corners[turret_idx_fire[0]]
                            fire_turret_endpoint_grid, fire_turret_endpoint_pixel = get_turret_endpoint(
                                turret_corners_fire, frame, grid_width, grid_height, TURRET_ENDPOINT_DISTANCE
                            )
                            # Draw turret endpoint
                            cv2.drawMarker(frame, fire_turret_endpoint_pixel, (255, 255, 0), cv2.MARKER_DIAMOND, 30, 3)
                            cv2.line(frame, tuple(turret_corners_fire.reshape((4,2)).mean(axis=0).astype(int)), fire_turret_endpoint_pixel, (255, 255, 0), 2)
                        
                        # Check if fire target and ball are detected
                        fire_target_visible = fire_target_id is not None and ids is not None and fire_target_id in ids.flatten()
                        ball_visible_fire = ids is not None and ball_id in ids.flatten()
                        
                        if fire_target_visible and fire_turret_endpoint_grid is not None:
                            fire_target_idx = np.where(ids == fire_target_id)[0][0]
                            fire_target_center = get_center(corners[fire_target_idx], frame, grid_width, grid_height)
                            fire_target_pixel = (int(fire_target_center[0] * img_w / grid_width), int(fire_target_center[1] * img_h / grid_height))
                            
                            # Draw fire target marker
                            cv2.drawMarker(frame, fire_target_pixel, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 40, 3)
                            cv2.putText(frame, f"FIRE TARGET {fire_target_id}", (fire_target_pixel[0] + 15, fire_target_pixel[1] - 15), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            
                            # Check if fire target reached turret endpoint
                            if fire_sequence_state == 'active':
                                dist_to_turret = abs(fire_target_center[0] - fire_turret_endpoint_grid[0]) + abs(fire_target_center[1] - fire_turret_endpoint_grid[1])
                                if dist_to_turret <= 1:  # Within 1 grid cell
                                    print(f"Fire target {fire_target_id} reached turret endpoint! Starting firefighting sequence...")
                                    fire_target_reached = True
                                    fire_reached_time = current_time
                                    fire_sequence_state = 'waiting_5s'
                                    
                                    # Send 0 dx/dy over UDP to stop movement, keep theta
                                    if ball_visible_fire:
                                        ball_idx_fire_stop = np.where(ids == ball_id)[0][0]
                                        yaw_fire_stop = get_2d_angle_from_corners(corners[ball_idx_fire_stop])
                                        next_target_stop = np.array([0, 0, compute_theta_send(yaw_fire_stop), 0])
                                        sock.sendto(struct.pack('<iif', 0, 0, float(next_target_stop[2])), (UDP_IP, UDP_PORT))
                                        # Update debug display values
                                        fire_udp_dx = 0
                                        fire_udp_dy = 0
                                        fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_fire_stop))
                                        print(f"FIRE: Target reached - sending dx=0, dy=0, theta={np.degrees(yaw_fire_stop):.1f}°")
                                elif ball_visible_fire:
                                    # Do pathfinding and control to push fire target to turret endpoint
                                    ball_idx_fire = np.where(ids == ball_id)[0][0]
                                    ball_center_fire = get_center(corners[ball_idx_fire], frame, grid_width, grid_height)
                                    ball_pixel_fire = (int(ball_center_fire[0] * img_w / grid_width), int(ball_center_fire[1] * img_h / grid_height))
                                    
                                    # Draw ball marker
                                    cv2.drawMarker(frame, ball_pixel_fire, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 40, 2)
                                    
                                    # Reset grid and add obstacles (excluding fire target and ball)
                                    grid.fill(1)
                                    grid = grid_obstacle(ids, corners, fire_target_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)
                                    
                                    # Pathfinding: target to turret endpoint
                                    ts_fire = (max(0, min(grid_height-1, int(fire_target_center[1]))), max(0, min(grid_width-1, int(fire_target_center[0]))))
                                    ep_fire = (max(0, min(grid_height-1, int(fire_turret_endpoint_grid[1]))), max(0, min(grid_width-1, int(fire_turret_endpoint_grid[0]))))
                                    path_t2e_fire = tcod.path.path2d(cost=grid, start_points=[ts_fire], end_points=[ep_fire], cardinal=10, diagonal=DIAGONAL_VAR)
                                    
                                    if len(path_t2e_fire) > 0:
                                        _, diagonaldown_path_fire = simplify_path(path_t2e_fire)
                                        
                                        # Calculate phi for mode decision
                                        phi_for_mode_fire = None
                                        if len(diagonaldown_path_fire) >= 2:
                                            # Get corners for phi calculation
                                            ball_corners_fire_phi = corners[ball_idx_fire]
                                            fire_target_corners_phi = corners[fire_target_idx]
                                            
                                            # Calculate phi1 using pixel coordinates
                                            ball_pts_fire = ball_corners_fire_phi.reshape((4, 2))
                                            ball_center_px_fire = ball_pts_fire.mean(axis=0)
                                            target_pts_fire = fire_target_corners_phi.reshape((4, 2))
                                            target_center_px_fire = target_pts_fire.mean(axis=0)
                                            phi1_fire = get_phi1_angle(target_center_px_fire, ball_center_px_fire)
                                            
                                            # Calculate phi2
                                            next_waypoint_fire = diagonaldown_path_fire[1]
                                            phi2_fire = get_phi2_angle(fire_target_center, next_waypoint_fire)
                                            
                                            # Calculate phi (angular difference)
                                            phi_for_mode_fire = compute_phi(phi1_fire, phi2_fire)
                                        
                                        # Check ball distance to target for mode selection
                                        ball_is_close_fire = abs(ball_center_fire[0] - fire_target_center[0]) <= BALL_TARGET_DISTANCE and abs(ball_center_fire[1] - fire_target_center[1]) <= BALL_TARGET_DISTANCE
                                        phi_is_aligned_fire = phi_for_mode_fire is not None and abs(np.degrees(phi_for_mode_fire)) <= PHI_THRESHOLD_DEG
                                        
                                        if ball_is_close_fire and phi_is_aligned_fire and len(diagonaldown_path_fire) >= 2:
                                            # PUSHING MODE with phi calculations
                                            print(f"FIRE PUSHING MODE - phi: {np.degrees(phi_for_mode_fire):.1f}° (within ±{PHI_THRESHOLD_DEG}°)")
                                            
                                            # Get ball and target corners
                                            ball_corners_fire_push = corners[ball_idx_fire]
                                            fire_target_corners_push = corners[fire_target_idx]
                                            
                                            # Calculate phi1 and phi2
                                            ball_pts_push = ball_corners_fire_push.reshape((4, 2))
                                            ball_center_px_push = ball_pts_push.mean(axis=0)
                                            target_pts_push = fire_target_corners_push.reshape((4, 2))
                                            target_center_px_push = target_pts_push.mean(axis=0)
                                            phi1 = get_phi1_angle(target_center_px_push, ball_center_px_push)
                                            
                                            next_waypoint = diagonaldown_path_fire[1]
                                            phi2 = get_phi2_angle(fire_target_center, next_waypoint)
                                            phi = compute_phi(phi1, phi2)
                                            
                                            # Control system
                                            k1 = K1_PUSH_GAIN
                                            k2 = K2_PHI_GAIN
                                            
                                            # dx1, dy1: Direction to push target towards waypoint
                                            dy1 = diagonaldown_path_fire[1][0] - fire_target_center[1]
                                            dx1 = diagonaldown_path_fire[1][1] - fire_target_center[0]
                                            
                                            # dx2, dy2: Direction to move ball to reduce phi to 0
                                            radius_ball_to_target = np.sqrt((ball_center_fire[0] - fire_target_center[0])**2 + 
                                                                             (ball_center_fire[1] - fire_target_center[1])**2)
                                            ball_rel_x_current = radius_ball_to_target * np.sin(phi1)
                                            ball_rel_y_current = -radius_ball_to_target * np.cos(phi1)
                                            ball_rel_x_desired = radius_ball_to_target * np.sin(phi2)
                                            ball_rel_y_desired = -radius_ball_to_target * np.cos(phi2)
                                            dx2 = ball_rel_x_desired - ball_rel_x_current
                                            dy2 = ball_rel_y_desired - ball_rel_y_current
                                            
                                            # Combine
                                            dx_out = (k1 * dx1) + (k2 * dx2)
                                            dy_out = (k1 * dy1) + (k2 * dy2)
                                            
                                            # Get yaw and send UDP
                                            yaw_fire = get_2d_angle_from_corners(corners[ball_idx_fire])
                                            angle_b2t_fire = np.arctan2(fire_target_center[1] - ball_center_fire[1], fire_target_center[0] - ball_center_fire[0])
                                            
                                            path_length_fire = len(path_t2e_fire)
                                            scaler = math.sqrt(dx_out**2 + dy_out**2)
                                            if scaler > 0:
                                                dx_scaled = dx_out / scaler * path_length_fire
                                                dy_scaled = dy_out / scaler * path_length_fire
                                            else:
                                                dx_scaled = 0
                                                dy_scaled = 0
                                            next_target_fire = np.array([dy_scaled, dx_scaled, compute_theta_send(yaw_fire), angle_b2t_fire])
                                            sock.sendto(struct.pack('<iif', int(next_target_fire[0]), int(next_target_fire[1]), float(next_target_fire[2])), (UDP_IP, UDP_PORT))
                                            # Update debug display values
                                            fire_udp_dx = dx_scaled
                                            fire_udp_dy = dy_scaled
                                            fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_fire))
                                            print(f"FIRE PUSHING: dx={dx_scaled:.1f}, dy={dy_scaled:.1f}, phi={np.degrees(phi):.1f}°")
                                            
                                            # Draw path from target to turret endpoint
                                            path_pixels_fire = []
                                            for pt in diagonaldown_path_fire:
                                                px = int(pt[1] * img_w / grid_width)
                                                py = int(pt[0] * img_h / grid_height)
                                                path_pixels_fire.append([px, py])
                                            if len(path_pixels_fire) > 1:
                                                cv2.polylines(frame, [np.array(path_pixels_fire, dtype=np.int32)], False, (0, 255, 0), 2)
                                            
                                            # Draw phi arrows
                                            phi_arrow_length = 50
                                            phi1_end = (int(fire_target_pixel[0] + phi_arrow_length * np.sin(phi1)),
                                                       int(fire_target_pixel[1] - phi_arrow_length * np.cos(phi1)))
                                            cv2.arrowedLine(frame, fire_target_pixel, phi1_end, (255, 255, 0), 2, tipLength=0.3)
                                            phi2_end = (int(fire_target_pixel[0] + phi_arrow_length * np.sin(phi2)),
                                                       int(fire_target_pixel[1] - phi_arrow_length * np.cos(phi2)))
                                            cv2.arrowedLine(frame, fire_target_pixel, phi2_end, (255, 0, 255), 2, tipLength=0.3)
                                        else:
                                            # BALL-TO-TARGET MODE - move ball to fire target
                                            phi_str = f"{np.degrees(phi_for_mode_fire):.1f}" if phi_for_mode_fire is not None else "N/A"
                                            print(f"FIRE B2T MODE - close: {ball_is_close_fire}, phi: {phi_str}° (threshold: ±{PHI_THRESHOLD_DEG}°)")
                                            
                                            # Set fire target as obstacle
                                            mask_fire = (x_coords - fire_target_center[0])**2 + (y_coords - fire_target_center[1])**2 <= radius**2
                                            grid[mask_fire] = 0
                                            
                                            # Calculate fake target (opposite side of turret endpoint)
                                            if len(path_t2e_fire) >= 2:
                                                vec_fire = (path_t2e_fire[1][1] - fire_target_center[0], path_t2e_fire[1][0] - fire_target_center[1])
                                            else:
                                                vec_fire = (fire_turret_endpoint_grid[0] - fire_target_center[0], fire_turret_endpoint_grid[1] - fire_target_center[1])
                                            vec_len = np.sqrt(vec_fire[0]**2 + vec_fire[1]**2)
                                            if vec_len > 0:
                                                vec_norm = (vec_fire[0] / vec_len, vec_fire[1] / vec_len)
                                                fake_target_fire = (fire_target_center[0] - ATTACK_DISTANCE * vec_norm[0], 
                                                                    fire_target_center[1] - ATTACK_DISTANCE * vec_norm[1])
                                            else:
                                                fake_target_fire = fire_target_center
                                            
                                            # Pathfinding ball to fake target
                                            bs_fire = (max(0, min(grid_height-1, int(ball_center_fire[1]))), max(0, min(grid_width-1, int(ball_center_fire[0]))))
                                            ft_fire = (max(0, min(grid_height-1, int(fake_target_fire[1]))), max(0, min(grid_width-1, int(fake_target_fire[0]))))
                                            path_b2t_fire = tcod.path.path2d(cost=grid, start_points=[bs_fire], end_points=[ft_fire], cardinal=10, diagonal=DIAGONAL_VAR)
                                            
                                            if len(path_b2t_fire) > 0:
                                                _, diagonaldown_b2t_fire = simplify_path(path_b2t_fire)
                                                if len(diagonaldown_b2t_fire) >= 2:
                                                    dy_b2t = diagonaldown_b2t_fire[1][0] - ball_center_fire[1]
                                                    dx_b2t = diagonaldown_b2t_fire[1][1] - ball_center_fire[0]
                                                    
                                                    yaw_fire = get_2d_angle_from_corners(corners[ball_idx_fire])
                                                    angle_b2t_fire = np.arctan2(fire_target_center[1] - ball_center_fire[1], fire_target_center[0] - ball_center_fire[0])
                                                    
                                                    path_len_fire = len(path_b2t_fire)
                                                    scaler = math.sqrt(dx_b2t**2 + dy_b2t**2)
                                                    if scaler > 0:
                                                        dx_scaled = dx_b2t / scaler * path_len_fire
                                                        dy_scaled = dy_b2t / scaler * path_len_fire
                                                    else:
                                                        dx_scaled = 0
                                                        dy_scaled = 0
                                                    next_target_fire = np.array([dy_scaled, dx_scaled, compute_theta_send(yaw_fire), angle_b2t_fire])
                                                    sock.sendto(struct.pack('<iif', int(next_target_fire[0]), int(next_target_fire[1]), float(next_target_fire[2])), (UDP_IP, UDP_PORT))
                                                    # Update debug display values
                                                    fire_udp_dx = dx_scaled
                                                    fire_udp_dy = dy_scaled
                                                    fire_udp_theta_deg = np.degrees(compute_theta_send(yaw_fire))
                                                    print(f"FIRE B2T: dx={dx_scaled:.1f}, dy={dy_scaled:.1f}")
                                                    
                                                    # Draw path
                                                    path_pixels_fire = []
                                                    for pt in diagonaldown_b2t_fire:
                                                        px = int(pt[1] * img_w / grid_width)
                                                        py = int(pt[0] * img_h / grid_height)
                                                        path_pixels_fire.append([px, py])
                                                    if len(path_pixels_fire) > 1:
                                                        cv2.polylines(frame, [np.array(path_pixels_fire, dtype=np.int32)], False, (255, 0, 0), 2)
                                            
                                            # Draw fake target
                                            fake_pixel_fire = (int(fake_target_fire[0] * img_w / grid_width), int(fake_target_fire[1] * img_h / grid_height))
                                            cv2.drawMarker(frame, fake_pixel_fire, (255, 0, 255), cv2.MARKER_DIAMOND, 30, 2)
                        
                        # Draw fire UDP debug panel
                        draw_fire_udp_debug(frame, fire_udp_dx, fire_udp_dy, fire_udp_theta_deg, fire_sequence_state)
                        cv2.imshow('frame-image', frame)
                        continue

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
                        # Ball detected - update tracking and timestamp
                        ball_last_seen_time = time.perf_counter()
                        ball_idx_temp = np.where(ids == ball_id)[0][0]
                        ball_last_corners = corners[ball_idx_temp].copy()
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, CM, dist_coef)
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
                    elif target_position_available and ball_last_center is not None and ball_last_seen_time is not None and (time.perf_counter() - ball_last_seen_time) < ball_cache_timeout:
                        # Ball NOT detected but cache is still valid - use last known position
                        time_since_ball_seen = time.perf_counter() - ball_last_seen_time
                        print(f"Ball not visible - using cached position ({time_since_ball_seen:.2f}s / {ball_cache_timeout}s)")
                        # Only draw markers if there are any detected
                        if ids is not None and len(ids) > 0:
                            out = aruco.drawDetectedMarkers(frame, corners, ids)
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, CM, dist_coef)
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
                        # Ball not detected and cache expired or unavailable
                        if ball_last_seen_time is not None:
                            time_since_ball_seen = time.perf_counter() - ball_last_seen_time
                            print(f"Ball cache expired ({time_since_ball_seen:.2f}s) - waiting for ball detection...")
                        else:
                            print(f"Ball not detected - no cached position available...")
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

                        # Only update obstacles if we have detected markers
                        if ids is not None and len(ids) > 0:
                            grid = grid_obstacle(ids, corners, keys, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)

                        if center_ball is not None and center_key is not None:
                            ball_start = center_ball
                            target_start = center_key

                            # pathfinding target to end point
                            # Clip coordinates to grid bounds
                            ts = (max(0, min(grid_height-1, int(target_start[1]))), max(0, min(grid_width-1, int(target_start[0])))) # (x,y) -> (y,x) format for tcod
                            endpoint_xy = get_endpoint_xy(end_points.get(keys))  # Returns (y, x) since end_points stores [y, x]
                            ep = (max(0, min(grid_height-1, int(endpoint_xy[0]))), max(0, min(grid_width-1, int(endpoint_xy[1])))) # Already (y,x), keep as (y,x) for tcod
                            print(f"DEBUG: Target start (y,x): {ts}, Endpoint (y,x): {ep}")
                            path_t2e = tcod.path.path2d(cost=grid, start_points=[ts], end_points=[ep], cardinal=10, diagonal=DIAGONAL_VAR)
                            
                            # ===== CALCULATE PHI BEFORE MODE DECISION =====
                            # Simplify path for phi2 calculation
                            if len(path_t2e) > 0:
                                _, diagonaldown_path_t2e_pre = simplify_path(path_t2e)
                            else:
                                diagonaldown_path_t2e_pre = []
                            
                            # Calculate phi to determine mode
                            phi_for_mode = None
                            if len(diagonaldown_path_t2e_pre) >= 2:
                                # Get ball corners for phi1 calculation
                                if ids is not None and len(ids) > 0:
                                    ball_idx_pre = np.where(ids == ball_id)[0]
                                else:
                                    ball_idx_pre = np.array([])
                                if ball_idx_pre.size > 0:
                                    ball_corners_pre = corners[ball_idx_pre[0]]
                                elif ball_last_corners is not None:
                                    ball_corners_pre = ball_last_corners
                                else:
                                    ball_corners_pre = None
                                
                                # Get target corners for phi1 calculation
                                if target_visible:
                                    target_corners_pre = corners[np.where(ids == keys)[0][0]]
                                elif target_last_corners is not None:
                                    target_corners_pre = target_last_corners
                                else:
                                    target_corners_pre = None
                                
                                # Calculate phi1 using pixel coordinates
                                if ball_corners_pre is not None and target_corners_pre is not None:
                                    ball_pts_pre = ball_corners_pre.reshape((4, 2))
                                    ball_center_px_pre = ball_pts_pre.mean(axis=0)
                                    target_pts_pre = target_corners_pre.reshape((4, 2))
                                    target_center_px_pre = target_pts_pre.mean(axis=0)
                                    phi1_pre = get_phi1_angle(target_center_px_pre, ball_center_px_pre)
                                else:
                                    phi1_pre = get_phi1_angle(
                                        (target_start[0] * frame.shape[1] / grid_width, target_start[1] * frame.shape[0] / grid_height),
                                        (ball_start[0] * frame.shape[1] / grid_width, ball_start[1] * frame.shape[0] / grid_height)
                                    )
                                
                                # Calculate phi2
                                next_waypoint_pre = diagonaldown_path_t2e_pre[1]
                                phi2_pre = get_phi2_angle(target_start, next_waypoint_pre)
                                
                                # Calculate phi (angular difference)
                                phi_for_mode = compute_phi(phi1_pre, phi2_pre)
                            
                            # Check if ball is close enough AND phi is within threshold for pushing mode
                            ball_is_close = abs(ball_start[0] - target_start[0]) <= BALL_TARGET_DISTANCE and abs(ball_start[1] - target_start[1]) <= BALL_TARGET_DISTANCE
                            phi_is_aligned = phi_for_mode is not None and abs(np.degrees(phi_for_mode)) <= PHI_THRESHOLD_DEG
                            
                            # Pushing mode debouncing logic
                            should_be_in_pushing_mode = ball_is_close and phi_is_aligned
                            
                            if should_be_in_pushing_mode and not is_in_pushing_mode:
                                # Enter pushing mode and start debounce timer
                                is_in_pushing_mode = True
                                pushing_mode_enter_time = time.perf_counter()
                                print(f"Entering PUSHING MODE - debounce locked for {PUSHING_MODE_DEBOUNCE}s")
                            elif is_in_pushing_mode and not should_be_in_pushing_mode:
                                # Check if we've been in pushing mode long enough to allow exit
                                time_in_pushing_mode = time.perf_counter() - pushing_mode_enter_time
                                if time_in_pushing_mode >= PUSHING_MODE_DEBOUNCE:
                                    # Debounce time elapsed, allow exit from pushing mode
                                    is_in_pushing_mode = False
                                    pushing_mode_enter_time = None
                                    print(f"Exiting PUSHING MODE - debounce time elapsed ({time_in_pushing_mode:.2f}s)")
                                else:
                                    # Still within debounce period, stay in pushing mode
                                    print(f"PUSHING MODE locked - {PUSHING_MODE_DEBOUNCE - time_in_pushing_mode:.2f}s remaining")
                            
                            # Enter pushing mode only if ball is close AND properly aligned (with debouncing)
                            if not is_in_pushing_mode:
                                # BALL-TO-TARGET MODE - ball needs to get closer or better aligned
                                phi_str = f"{np.degrees(phi_for_mode):.1f}" if phi_for_mode is not None else "N/A"
                                print(f"BALL-TO-TARGET MODE - close: {ball_is_close}, phi: {phi_str}° (threshold: ±{PHI_THRESHOLD_DEG}°)")
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
                                push_distance = ATTACK_DISTANCE

                                # Use endpoint directly if path is too short to get direction
                                # Use endpoint directly if path is too short to get direction
                                if len(path_t2e) >= 2:
                                    # Vector from target to next waypoint (direction to endpoint)
                                    vec_t2e = (path_t2e[1][1] - target_start[0], path_t2e[1][0] - target_start[1])
                                else:
                                    # Path has only 1 point (target at endpoint), use direct vector to endpoint
                                    endpoint_xy = get_endpoint_xy(end_points.get(keys))
                                    vec_t2e = (endpoint_xy[1] - target_start[0], endpoint_xy[0] - target_start[1])
                                
                                # Normalize vector if non-zero, then place fake target opposite
                                vec_length = np.sqrt(vec_t2e[0]**2 + vec_t2e[1]**2)
                                if vec_length > 0:
                                    vec_t2e_norm = (vec_t2e[0] / vec_length, vec_t2e[1] / vec_length)
                                    fake_target = (target_start[0] - push_distance * vec_t2e_norm[0], 
                                                   target_start[1] - push_distance * vec_t2e_norm[1])
                                else:
                                    # Target is exactly at endpoint, no direction to compute
                                    fake_target = target_start
                                
                                # pathfinding ball to fake target
                                # Clip coordinates to grid bounds
                                bs = (max(0, min(grid_height-1, int(ball_start[1]))), max(0, min(grid_width-1, int(ball_start[0]))))
                                ts = (max(0, min(grid_height-1, int(fake_target[1]))), max(0, min(grid_width-1, int(fake_target[0]))))
                                path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[ts], cardinal=10, diagonal=DIAGONAL_VAR)
                                # tcod can return an empty array; guard simplify_path
                                if len(path_b2t) > 0:
                                    _, diagonaldown_path_b2t = simplify_path(path_b2t)
                                else:
                                    diagonaldown_path_b2t = []

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
                                        # Normalize direction and scale by path length
                                        path_length_b2t = len(path_b2t)
                                        scaler = math.sqrt(dx**2 + dy**2)
                                        if scaler > 0:
                                            dx_scaled = dx / scaler * path_length_b2t
                                            dy_scaled = dy / scaler * path_length_b2t
                                        else:
                                            dx_scaled = 0
                                            dy_scaled = 0
                                        # UDP sending
                                        next_target = np.array([dy_scaled, dx_scaled, compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                        sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                        print (f"9. message: {next_target}, path_length_b2t: {path_length_b2t}")
                                        
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
                                # PUSHING MODE - ball is close AND properly aligned (phi within threshold)
                                print(f"PUSHING MODE - phi: {np.degrees(phi_for_mode):.1f}° (within ±{PHI_THRESHOLD_DEG}°)")
                                # Reuse the pre-calculated simplified path
                                diagonaldown_path_t2e = diagonaldown_path_t2e_pre

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
                                k1 = K1_PUSH_GAIN
                                k2 = K2_PHI_GAIN
                                
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

                                    # Normalize direction and scale by path length
                                    path_length_t2e = len(path_t2e)
                                    scaler = math.sqrt(dx_out**2 + dy_out**2)
                                    if scaler > 0:
                                        dx_out_scaled = dx_out / scaler * path_length_t2e
                                        dy_out_scaled = dy_out / scaler * path_length_t2e
                                    else:
                                        dx_out_scaled = 0
                                        dy_out_scaled = 0
                                    # UDP sending with combined control output
                                    next_target = np.array([dy_out_scaled, dx_out_scaled, compute_theta_send(yaw), angle_b2t])
                                    sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                    print (f"13. message: {next_target}, path_length_t2e: {path_length_t2e}")
                                    
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
                
                if quit_flag or mode == 'manual':
                    break

            # Final check for all targets reached - rotate endpoints
            if len(In_range) >= len(end_points) and all(In_range.get(k, False) for k in end_points.keys()):
                print("All targets in position! Rotating endpoints...")
                end_points = rotate_dict(end_points, k=1)  # k=1 shifts each marker to next endpoint
                In_range.clear()  # Reset In_range for new endpoint assignments
                print(f"New endpoints: {end_points}")
        
    # Manual mode key hold state
    manual_last_key = None
    manual_last_key_time = 0
    manual_key_timeout = MANUAL_KEY_TIMEOUT
    
    while mode == 'manual' and not quit_flag:
        
        # Capture current frame from the camera
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        yaw = ball_last_yaw  # Use cached yaw as default
        
        # Only fetch yaw if we see the ball marker
        if ids is not None and len(ids) > 0 and ball_id in ids.flatten():
            out = aruco.drawDetectedMarkers(frame, corners, ids)
            ball_idx = np.where(ids == ball_id)[0]
            if ball_idx.size > 0:
                # Use 2D corner-based angle (consistent with auto mode)
                yaw = get_2d_angle_from_corners(corners[ball_idx[0]])
                ball_last_yaw = yaw  # Update cached yaw
                print ('M1. yaw (2D):', np.degrees(yaw))

        # Check for 'awsd' key press for manual control
        key = cv2.waitKey(1) & 0xFF
        current_time = time.perf_counter()
        
        # Check if a movement key was pressed this frame
        new_key_pressed = None
        if key == ord('w'):
            new_key_pressed = 'w'
        elif key == ord('a'):
            new_key_pressed = 'a'
        elif key == ord('s'):
            new_key_pressed = 's'
        elif key == ord('d'):
            new_key_pressed = 'd'
        elif key == ord('q'):
            quit_flag = True
            break
        elif key == ord('c'):  # c to change back to auto mode
            mode = 'auto'
            print("Auto mode activated: normal operation")
            break
        elif key == ord('f'):
            isFire = not isFire
            if isFire:
                import random
                fire_target_id = None
                fire_target_reached = False
                fire_reached_time = None
                fire_sequence_state = 'active'
                fire_turret_endpoint_grid = None
                fire_turret_endpoint_pixel = None
                available_targets = list(end_points.keys())
                if available_targets:
                    fire_target_id = random.choice(available_targets)
                    print(f"Fire alert: ACTIVATED - Target ID {fire_target_id} is ON FIRE!")
                else:
                    print("Fire alert: ACTIVATED but no targets available!")
                    isFire = False
            else:
                fire_target_id = None
                fire_target_reached = False
                fire_reached_time = None
                fire_sequence_state = 'inactive'
                fire_turret_endpoint_grid = None
                fire_turret_endpoint_pixel = None
                print(f"Fire alert: DEACTIVATED")
        
        # If fire alert is active, display fire screen and skip normal processing
        if isFire:
            # Handle fire sequence state machine
            current_time_fire = time.perf_counter()
            if fire_sequence_state == 'waiting_5s' and fire_reached_time is not None:
                if current_time_fire - fire_reached_time >= 5.0:
                    send_fire_udp(1, FIRE_UDP_IP, FIRE_UDP_PORT)
                    fire_sequence_state = 'sent_1'
                    fire_reached_time = current_time_fire
            elif fire_sequence_state == 'sent_1' and fire_reached_time is not None:
                if current_time_fire - fire_reached_time >= 2.0:
                    send_fire_udp(0, FIRE_UDP_IP, FIRE_UDP_PORT)
                    fire_sequence_state = 'sent_0'
                    fire_reached_time = current_time_fire
            elif fire_sequence_state == 'sent_0' and fire_reached_time is not None:
                if current_time_fire - fire_reached_time >= 3.0:
                    fire_sequence_state = 'firefighted'
            
            if fire_sequence_state == 'firefighted':
                draw_firefighted_alert(frame)
            else:
                draw_fire_alert(frame, fire_target_id)
            cv2.imshow('frame-image', frame)
            continue
        
        # Update held key if new key pressed
        if new_key_pressed is not None:
            manual_last_key = new_key_pressed
            manual_last_key_time = current_time
        
        # Determine active key (use held key if within timeout)
        if manual_last_key is not None and (current_time - manual_last_key_time) < manual_key_timeout:
            pressed_key = manual_last_key
        else:
            pressed_key = None
            manual_last_key = None  # Clear held key after timeout
        
        # Set next_target based on active key
        if pressed_key == 'w':
            print('direction = up')
            next_target = np.array([-MANUAL_MOVE_SPEED, 0, compute_theta_send(yaw), 0])  # dy negative (up)
        elif pressed_key == 'a':
            print('direction = left')
            next_target = np.array([0, -MANUAL_MOVE_SPEED, compute_theta_send(yaw), 0])  # dx negative (left)
        elif pressed_key == 's':
            print('direction = down')
            next_target = np.array([MANUAL_MOVE_SPEED, 0, compute_theta_send(yaw), 0])   # dy positive (down)
        elif pressed_key == 'd':
            print('direction = right')
            next_target = np.array([0, MANUAL_MOVE_SPEED, compute_theta_send(yaw), 0])   # dx positive (right)
        else:
            next_target = np.array([0, 0, compute_theta_send(yaw), 0])
            print('no movement key pressed')

        sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
        print(f"MANUAL UDP: dx={int(next_target[1])}, dy={int(next_target[0])}, theta={np.degrees(next_target[2]):.1f}° (raw yaw={np.degrees(yaw):.1f}°)")
        
        # Draw WASD visual (only in manual mode)
        draw_wasd_keys(frame, pressed_key=pressed_key, dy=next_target[0], dx=next_target[1])
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
