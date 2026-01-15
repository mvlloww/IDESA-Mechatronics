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


    # Calculate ball and target pixel positions for drawing markers (if needed elsewhere, move to main loop)
    # ball_pixel = (int(ball_start[0] * img_w / grid_width), int(ball_start[1] * img_h / grid_height))
    # target_pixel = (int(end_point[0] * img_w / grid_width), int(end_point[1] * img_h / grid_height))
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
def rotate_dict(d, k=2):
    items = list(d.items())
    k = k % len(items)   # safety for large shifts
    rotated = items[-k:] + items[:-k]
    return dict(rotated)
def compute_theta_send(theta):
    if abs(theta) > np.pi / 2:
        theta_send = np.sign(theta) * (np.pi - abs(theta))
    else:
        theta_send = -theta
    return theta_send

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
# Define the aruco detection parameters
parameters = aruco.DetectorParameters()

# CAP_DSHOW to make sure it uses DirectShow backend on Windows (more stable)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
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
end_points = {1:[[15,10],False], 2:[[15,35],False], 3:[[12,39],False], 4:[[5,10],False]}
# Set ball ArUco id
ball_id = 8
# Create id_buffer dictionary
id_buffer = {}

'''Temperary variables'''
IsFire = False
quit_flag = False

''' Setup UDP communication '''
# This is the IP address of the machine that the data will be send to
UDP_IP = "138.38.229.206" #clearpass IP address for RPI 3B Model +
# This is the RENOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
UDP_PORT = 50000
sock = socket.socket(socket.AF_INET,    # Family of addresses, in this case IP type 
                     socket.SOCK_DGRAM) # What protocol to use, in this case UDP (datagram)

In_range = {}
pushing_target_id = 1
target_id = 1

while True:
    print("Loop running...")
    # Capture current frame from the camera
    ret, frame = cap.read()

    # Skip frame if capture failed
    if not ret or frame is None:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray-image', gray)  # Show grayscale image for debugging
    # Detect ArUco markers in the grey image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"DEBUG: ids={ids}, corners={corners}, rejected={rejectedImgPoints}")
    if ids is None:
        print("DEBUG: No markers detected (ids is None)")
    elif len(ids) == 0:
        print("DEBUG: ids is empty array")
    else:
        print(f"DEBUG: Detected {len(ids)} marker(s): {ids.flatten().tolist()}")
    cv2.imshow('frame-image', frame)
    # If markers are detected, draw them, estimate pose and overlay axes
    if ids is not None and len(ids) > 0:
        out = aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
        # Check if target_id and ball_id are detected
        target_indices = np.where(ids == target_id)[0]
        ball_indices = np.where(ids == ball_id)[0]
        print("Detected IDs:", ids.flatten().tolist())
        cv2.imshow('frame-image', frame)
        if len(target_indices) > 0 and len(ball_indices) > 0:
            center_key = get_center(corners[target_indices[0]], frame, grid_width, grid_height)
            center_ball = get_center(corners[ball_indices[0]], frame, grid_width, grid_height)
            # Get angle of ball to target
            angle_b2t = np.arctan2(center_key[1]-center_ball[1], center_key[0]-center_ball[0])
            grid.fill(1)
            ball_start = tuple(map(int, get_center(corners[ball_indices[0]], frame, grid_width, grid_height)))
            ball_start_cache = (ball_start[1], ball_start[0])  # (y, x) for path2d
            grid = grid_obstacle(ids, corners, target_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height)
            end_point = tuple(map(int, get_center(corners[target_indices[0]], frame, grid_width, grid_height)))
            endpoint_cache = (end_point[1], end_point[0])  # (y, x) for path2d
            path_b2t = tcod.path.path2d(cost=grid, start_points=[ball_start_cache], end_points=[endpoint_cache], cardinal=10, diagonal=14)
            if len(path_b2t) > 0:
                _, diagonaldown_path_b2t = simplify_path(path_b2t)
            else:
                diagonaldown_path_b2t = []
            #print('6. target id', target_id, "path_b2t:", diagonaldown_path_b2t)

            # Check if path has at least 2 points before accessing [1]
            if len(diagonaldown_path_b2t) >= 1:
                # convert to dx, dy instructions for UDP sending
                dy = ball_start[1]-diagonaldown_path_b2t[1][0]
                dx = ball_start[0]-diagonaldown_path_b2t[1][1]
                #print ('7. dx, dy:', dx, dy, ball_start, diagonaldown_path_b2t[1])
        else:
            # One or more required markers not detected; skip this frame
            continue

        # Recalculate ball_idx for current frame
        ball_idx = np.where(ids == ball_id)[0]
        if ball_idx.size > 0:
            rotate_vec = rvecs[ball_idx[0]][0]
            R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
            yaw = np.arctan2(R[1, 0], R[0, 0])
            #print ('8. rotate_vec:', yaw)
            # UDP sending
            next_target = np.array([dy, dx,compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
            sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
            print ("9. message:", next_target)
        else:
            #print("8. Ball not detected, skipping rotation calculation")
            pass
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

        ball_pixel = (int(ball_start[0] * img_w / grid_width), int(ball_start[1] * img_h / grid_height))
        target_pixel = (int(end_point[0] * img_w / grid_width), int(end_point[1] * img_h / grid_height))

        cv2.drawMarker(frame, ball_pixel, (0,255,0), cv2.MARKER_TILTED_CROSS, 40, 2)
        cv2.drawMarker(frame, target_pixel, (0,0,255), cv2.MARKER_TILTED_CROSS, 40, 2)
        
        for k in end_points.keys():
            ep_coords = end_points.get(k)[0]
            ep_pixel = (int(ep_coords[1] * img_w / grid_width), int(ep_coords[0] * img_h / grid_height))
            cv2.drawMarker(frame, ep_pixel, (255,255,0), cv2.MARKER_DIAMOND, 40, 2)
        # Show the frame after all drawing is done
        cv2.imshow('frame-image', frame)
        # Wait for key press and check for quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit_flag = True
            break
# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)
