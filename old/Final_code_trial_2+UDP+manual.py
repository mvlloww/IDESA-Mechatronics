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

    return (x, y)
def grid_obstacle(ids, target_id, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height):
    '''
    generate obstacles on grid function

    input: detected ArUco ids, target id to ignore, x/y coordinate arrays of grid, current grid, obstacle radius
    output: updated grid with obstacles marked for all non-target markers
    '''
    img_h, img_w = frame.shape[:2]
    
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id != int(target_id) and marker_id != int(ball_id):
            # Get corners for this specific marker
            pts = corners[i].reshape((4, 2))
            center_pixel = pts.mean(axis=0)
            
            grid_x = int(center_pixel[0] / img_w * grid_width)
            grid_y = int(center_pixel[1] / img_h * grid_height)
            center_grid = np.clip([grid_x, grid_y], 0, [grid_width - 1, grid_height - 1]).astype(int)
            ox, oy = center_grid

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
marker_size = 87
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
ball_id = 8
# Create id_buffer dictionary
id_buffer = {}

'''Temperary variables'''
IsFire = False
quit_flag = False
mode = 'auto'  # 'auto' or 'manual'

''' Setup UDP communication '''
# This is the IP address of the machine that the data will be send to
UDP_IP = "138.38.229.206" #clearpass IP address for RPI 3B Model +
# This is the RENOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
UDP_PORT = 50000
sock = socket.socket(socket.AF_INET,    # Family of addresses, in this case IP type 
                     socket.SOCK_DGRAM) # What protocol to use, in this case UDP (datagram)

''' Main loop '''
print ("Auto mode activated: normal operation")
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

                # Update grid with obstacles for all non-target markers
                grid = grid_obstacle(ids, marker_id, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height)
                # Get center of target ArUco code on grid
                start_point = get_center(center_3d, rvecs[t], tvecs[t], CM, dist_coef, frame, grid_width, grid_height)
                print("1. Start Point for ID ", marker_id, ": ", start_point)

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
            print("2. Sorted Targets based on path length: ", sorted_targets) 

            for keys, _len in sorted_targets:
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
                        center_key = get_center(center_3d, rvecs[np.where(ids==keys)[0][0]], tvecs[np.where(ids==keys)[0][0]], CM, dist_coef, frame, grid_width, grid_height)
                        center_ball = get_center(center_3d, rvecs[np.where(ids==ball_id)[0][0]], tvecs[np.where(ids==ball_id)[0][0]], CM, dist_coef, frame, grid_width, grid_height)
                        
                        # Get angle of ball to target
                        angle_b2t = np.arctan2(center_key[1]-center_ball[1], center_key[0]-center_ball[0])
                        
                        # cv2.imshow('frame-image', frame)
                        if end_points.get(keys) == center_key:
                            position_status = True
                            print ("3. Target ID ", keys, " reached endpoint. switching to next target.")

                        # compare current position with buffered position
                        for i in range(len(ids)):
                            if ids[i][0] != keys:
                                # Recompute obstacles for current positions of non-target markers
                                grid = grid_obstacle(ids, keys, x_coords, y_coords, grid, radius, center_3d, rvecs, tvecs, CM, dist_coef, frame, grid_width, grid_height)
                                print ("4. Updated grid with obstacles for non-target markers.")

                        # ball to target
                        # ball_idx = np.where(ids == ball_id)[0]
                        # key_idx = np.where(ids == keys)[0]
                        if center_ball is not None and center_key is not None:
                            ball_start = center_ball
                            target_start = center_key

                            # pathfinding target to end point
                            ts = (int(target_start[1]), int(target_start[0])) # (x,y) -> (y,x) format
                            ep = (int(end_points.get(keys)[0]), int(end_points.get(keys)[1])) # (x,y) -> (y,x) format
                            path_t2e = tcod.path.path2d(cost=grid, start_points=[ts], end_points=[ep], cardinal=10, diagonal=14)
                            

                            if abs(ball_start[0] - target_start[0]) > 2 and abs(ball_start[1] - target_start[1]) > 2:
                                # Set target to obstacle
                                mask = (x_coords - target_start[0])**2 + (y_coords - target_start[1])**2 <= radius**2
                                grid[mask] = 0

                                # Get fake target position for ball to target pathfinding
                                fdy = 0.2*(target_start[1] - path_t2e[1][0])
                                fdx = 0.2*(target_start[0] - path_t2e[1][1])
                                fake_target = (target_start[0] + fdx, target_start[1] + fdy)
                                print ("5. Fake target for ball to target pathfinding: ", fake_target)
                                
                                # pathfinding ball to fake target
                                bs = (int(ball_start[1]), int(ball_start[0]))
                                ts = (int(fake_target[1]), int(fake_target[0]))
                                path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[ts], cardinal=10, diagonal=14)
                                # tcod can return an empty array; guard simplify_path
                                if len(path_b2t) > 0:
                                    _, diagonaldown_path_b2t = simplify_path(path_b2t)
                                else:
                                    diagonaldown_path_b2t = []
                                print('6. target id', keys, "path_b2t:", diagonaldown_path_b2t)

                                # convert to dx, dy instructions for UDP sending
                                dy = ball_start[1]-diagonaldown_path_b2t[1][0]
                                dx = ball_start[0]-diagonaldown_path_b2t[1][1]
                                print ('7. dx, dy:', dx, dy, ball_start, diagonaldown_path_b2t[1])

                                # Recalculate ball_idx for current frame
                                ball_idx = np.where(ids == ball_id)[0]
                                if ball_idx.size > 0:
                                    rotate_vec = rvecs[ball_idx[0]][0]
                                    R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                    yaw = np.arctan2(R[1, 0], R[0, 0])
                                    print ('8. rotate_vec:', yaw)
                                    # UDP sending
                                    next_target = np.array([dy, dx,compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                    sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                    print ("9. message:", next_target)
                                else:
                                    print("8. Ball not detected, skipping rotation calculation")


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
                                # ts = (int(target_start[1]), int(target_start[0]))
                                # ep = (int(end_points.get(keys)[0]), int(end_points.get(keys)[1]))
                                # path_t2e = tcod.path.path2d(cost=grid, start_points=[ts], end_points=[ep], cardinal=10, diagonal=14)
                                if len(path_t2e) > 0:
                                    _, diagonaldown_path_t2e = simplify_path(path_t2e)
                                else:
                                    diagonaldown_path_t2e = []
                                print('10. target id', keys, "path_t2e:", diagonaldown_path_t2e)

                                # convert to dx, dy instructions for UDP sending
                                dy = target_start[1]-diagonaldown_path_t2e[1][0]
                                dx = target_start[0]-diagonaldown_path_t2e[1][1]
                                print ('11. dx, dy:', dx, dy, target_start, diagonaldown_path_t2e[1])

                                # Recalculate ball_idx for current frame
                                ball_idx = np.where(ids == ball_id)[0]
                                if ball_idx.size > 0:
                                    rotate_vec = rvecs[ball_idx[0]][0]
                                    R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                    yaw = np.arctan2(R[1, 0], R[0, 0])
                                    print ('12. rotate_vec:', yaw)

                                    # UDP sending
                                    next_target = np.array([dy, dx, compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j),angle) coordinates of next target point
                                    sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                    print ("13. message:", next_target)
                                else:
                                    print("12. Ball not detected, skipping rotation calculation")

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
                                cv2.imshow('frame-image', frame)
                                print('14. target id', keys, "path_t2e:", path_t2e)
                    else:
                        print("1. Ball or target not detected.")
                        break
                
                # Display the frame once after all processing
                #cv2.imshow('frame-image', frame)
                
                if quit_flag or mode == 'manual':
                    break

            # Crop rotation
            end_points = rotate_dict(end_points, k=2)
            print("15. Rotated end points: ", end_points)

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
            next_target = np.array([0, -50, compute_theta_send(yaw), 0])
        elif key == ord('s'):
            print ('direction = down')
            next_target = np.array([-50, 0, compute_theta_send(yaw), 0])

        elif key == ord('d'):
            print ('direction = right')
            next_target = np.array([0, 50, compute_theta_send(yaw), 0])
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
        cv2.imshow('frame-image', frame)

    if quit_flag:
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit()
