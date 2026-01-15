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
def grid_obstacle(ids, corners, target_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height, end_points):
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
radius = 2

# Set target end points for pathfinding (can be changed later)
end_points = {1:[[15,10],False], 2:[[15,35],False], 3:[[12,39],False], 4:[[5,10],False]}
# Set ball ArUco id
ball_id = 8

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

prev_ball_start_cache = (0, 0)
prev_fake_target_start_cache = (0, 0)
prev_target_start_cache = (0, 0)
prev_end_points_cache_2 = (0, 0)

''' Main loop '''
while True:
    # Start the performance clock
    start = time.perf_counter()

    while IsFire == False and not quit_flag:
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

            # DEBUG: Print detected marker IDs
            print(f"DEBUG: Detected marker IDs: {ids.flatten().tolist() if ids is not None else 'None'}")
            print(f"DEBUG: Configured end_points keys: {list(end_points.keys())}")

            # For each detected marker
            for t in range(len(ids)):
                marker_id = int(ids[t][0])
                
                # Only process markers that have endpoints configured
                if marker_id not in end_points:
                    continue

                # Update grid with obstacles for all non-target markers
                grid = grid_obstacle(ids, corners, marker_id, x_coords, y_coords, grid, radius, frame, grid_width, grid_height, end_points)
                
                center_cache = get_center(corners[t], frame, grid_width, grid_height)
                if np.linalg.norm(np.array(end_points.get(marker_id)[0]) - np.array(center_cache)) <= 1:
                    end_points[marker_id][1] = True
                    # Convert grid coordinates to pixel coordinates for text display
                    # img_h, img_w = frame.shape[:2]
                    # ep_grid = end_points.get(marker_id)[0]
                    # text_pos = (int(ep_grid[0] * img_w / grid_width), int(ep_grid[1] * img_h / grid_height))
                    # cv2.putText(frame,'True', text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    # Still add to target_dict but with 0 length since already at endpoint
                    continue

                # Get path from target to end point and ball to target
                # tcod.path.path2d expects sequences of (i,j) pairs
                start_point_cache = (int(center_cache[1]), int(center_cache[0])) # Note the (y,x) order for (i,j)
                target_endpoint = end_points.get(marker_id)[0]
                if target_endpoint is not None:
                    end_points_cache = (int(target_endpoint[0]), int(target_endpoint[1]))
                    path_t2e = tcod.path.path2d(cost=grid, start_points=[start_point_cache], end_points=[end_points_cache], cardinal=10, diagonal=14)
                    ball_idx = np.where(ids == ball_id)[0]
                    if ball_idx.size > 0:
                        ball_center = get_center(corners[ball_idx[0]], frame, grid_width, grid_height)
                        bs = (int(ball_center[1]), int(ball_center[0]))
                        path_b2t = tcod.path.path2d(cost=grid, start_points=[bs], end_points=[start_point_cache], cardinal=10, diagonal=14)
                    else:
                        path_b2t = []
                    total_path = np.concatenate((path_b2t, path_t2e)) if len(path_b2t) > 0 else path_t2e
                    # append total path to target_dict 
                    target_dict[marker_id] = len(total_path)
                else:
                    print(f"DEBUG: Marker ID {marker_id} has no endpoint mapping in end_points; skipping.")
            # Sort target_dict based on path length
            sorted_targets = sorted(target_dict.items(), key=lambda item: item[1])
            print("2. Sorted Targets based on path length: ", sorted_targets) 

            if len(sorted_targets) == 0:
                try:
                    print("DEBUG: No targets matched configured end_points keys.")
                    print("DEBUG: Configured end_points keys:", list(end_points.keys()))
                except Exception:
                    pass
                # Show frame even if no targets to process
                cv2.imshow('frame-image', frame)
                # Nothing to process this cycle; continue capturing
                continue

            for keys, _len in sorted_targets:
                if end_points[keys][1] == False:
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

                    # Check for 'q' key press
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        quit_flag = True
                        break

                    if ids is not None and len(ids) > 0 and keys in ids.flatten() and ball_id in ids.flatten():
                        out = aruco.drawDetectedMarkers(frame, corners, ids)
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)
                        center_target = get_center(corners[np.where(ids==keys)[0][0]], frame, grid_width, grid_height)
                        center_ball = get_center(corners[np.where(ids==ball_id)[0][0]], frame, grid_width, grid_height)
                        
                        # Get angle of ball to target
                        angle_b2t = np.arctan2(center_target[1]-center_ball[1], center_target[0]-center_ball[0])
                        
                        grid = grid_obstacle(ids, corners, keys, x_coords, y_coords, grid, radius, frame, grid_width, grid_height, end_points)
                        print ("4. Updated grid with obstacles for non-target markers.")

                        # Check if target has reached endpoint
                        if np.linalg.norm(np.array(end_points.get(keys)[0]) - np.array(center_target)) <= 2:
                            end_points[keys][1] = True
                            position_status = True
                            print ("3. Target ID ", keys, " reached endpoint. switching to next target.")
                            break

                        # pathfinding target to end point
                        target_starting = (int(center_target[1]), int(center_target[0])) # (x,y) -> (y,x) format
                        end_points_cache_2 = (int(end_points.get(keys)[0][0]), int(end_points.get(keys)[0][1])) # (x,y) -> (y,x) format
                        # if target_starting != prev_target_start_cache or end_points_cache_2 != prev_end_points_cache_2:
                        path_t2e = tcod.path.path2d(cost=grid, start_points=[target_starting], end_points=[end_points_cache_2], cardinal=10, diagonal=14)
                        prev_target_start_cache = target_starting
                        prev_end_points_cache_2 = end_points_cache_2

                        if abs(center_ball[0] - center_target[0]) > 2 and abs(center_ball[1] - center_target[1]) > 2:
                            # Get fake target position for ball to target pathfinding
                            if len(path_t2e) >= 2:
                                fdy = 0.5*(center_target[1] - path_t2e[1][0])
                                fdx = 0.5*(center_target[0] - path_t2e[1][1])
                                fake_target = (center_target[0] + fdx, center_target[1] + fdy)
                            else:
                                # Path too short, use target position directly
                                fake_target = center_target
                            print ("5. Fake target for ball to target pathfinding: ", fake_target)
                            
                            # pathfinding ball to fake target
                            ball_start_cache = (int(center_ball[1]), int(center_ball[0]))
                            fake_target_start_cache = (int(fake_target[1]), int(fake_target[0]))
                            # if ball_start_cache != prev_ball_start_cache or fake_target_start_cache != prev_fake_target_start_cache:
                            path_b2t = tcod.path.path2d(cost=grid, start_points=[ball_start_cache], end_points=[fake_target_start_cache], cardinal=10, diagonal=14)
                            prev_ball_start_cache = ball_start_cache
                            prev_fake_target_start_cache = fake_target_start_cache
                            # tcod can return an empty array; guard simplify_path
                            if len(path_b2t) > 0:
                                _, diagonaldown_path_b2t = simplify_path(path_b2t)
                            else:
                                diagonaldown_path_b2t = []
                            # print('6. target id', keys, "path_b2t:", diagonaldown_path_b2t)

                            # Check if path has at least 1 points before accessing [1]
                            if len(diagonaldown_path_b2t) >= 1:
                                # convert to dx, dy instructions for UDP sending
                                dy = center_ball[1]-diagonaldown_path_b2t[1][0]
                                dx = center_ball[0]-diagonaldown_path_b2t[1][1]
                                # print ('7. dx, dy:', dx, dy, center_ball, diagonaldown_path_b2t[1])

                                # Recalculate ball_idx for current frame
                                ball_idx = np.where(ids == ball_id)[0]
                                if ball_idx.size > 0:
                                    rotate_vec = rvecs[ball_idx[0]][0]
                                    R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                    yaw = np.arctan2(R[1, 0], R[0, 0])
                                    print ('8. rotate_vec:', yaw)
                                    # UDP sending
                                    next_target = np.array([dy, dx,compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                    #sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                    print ("9. message:", next_target)
                                else:
                                    print("8. Ball not detected, skipping rotation calculation")
                            else:
                                print("7. Path too short for ball-to-target movement")

                                
                            # Convert grid coordinates to pixel coordinates
                            img_h, img_w = frame.shape[:2]
                            
                            # Grid lines removed for performance
                            # Uncomment if needed:
                            # for i in range(grid_height + 1):
                            #     y = int(round(i * img_h / grid_height))
                            #     cv2.line(frame, (0, y), (img_w, y), (100, 100, 100), 1)
                            # for j in range(grid_width + 1):
                            #     x = int(round(j * img_w / grid_width))
                            #     cv2.line(frame, (x, 0), (x, img_h), (100, 100, 100), 1)
                            
                            # Convert path from grid to pixel coordinates
                            if len(diagonaldown_path_b2t) > 1:
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
                            target_pixel = (int(center_target[0] * img_w / grid_width), int(center_target[1] * img_h / grid_height))
                            fake_pixel = (int(fake_target[0] * img_w / grid_width), int(fake_target[1] * img_h / grid_height))
                            
                            cv2.drawMarker(frame, ball_pixel, (0,255,0), cv2.MARKER_TILTED_CROSS, 40, 2)
                            cv2.drawMarker(frame, target_pixel, (0,0,255), cv2.MARKER_TILTED_CROSS, 40, 2)
                            cv2.drawMarker(frame, fake_pixel, (255,0,255), cv2.MARKER_DIAMOND, 40, 2)
                            for k in end_points.keys():
                                ep_coords = end_points.get(k)[0]
                                ep_pixel = (int(ep_coords[1] * img_w / grid_width), int(ep_coords[0] * img_h / grid_height))
                                cv2.drawMarker(frame, ep_pixel, (255,255,0), cv2.MARKER_DIAMOND, 40, 2)
                            

                        elif abs(center_ball[0] - center_target[0]) <= 2 and abs(center_ball[1] - center_target[1]) <= 2:
                            if len(path_t2e) > 0:
                                _, diagonaldown_path_t2e = simplify_path(path_t2e)
                            else:
                                diagonaldown_path_t2e = []
                            print('10. target id', keys, "path_t2e:", diagonaldown_path_t2e)
                            
                            # Check if path has at least 1 points before accessing [1]
                            if len(diagonaldown_path_t2e) < 1:
                                print("11. Path too short for target movement, skipping")
                                cv2.imshow('frame-image', frame)
                                continue
                            
                            # UDP
                            dy = center_target[1]-diagonaldown_path_t2e[1][0]
                            dx = center_target[0]-diagonaldown_path_t2e[1][1]
                            print ('11. dx, dy:', dx, dy, center_target, diagonaldown_path_t2e[1])

                            # Recalculate ball_idx for current frame
                            ball_idx = np.where(ids == ball_id)[0]
                            if ball_idx.size > 0:
                                rotate_vec = rvecs[ball_idx[0]][0]
                                R, _ = cv2.Rodrigues(rvecs[ball_idx[0]][0])
                                yaw = np.arctan2(R[1, 0], R[0, 0])
                                print ('12. rotate_vec:', yaw)

                                # UDP sending
                                next_target = np.array([dy, dx, compute_theta_send(yaw), angle_b2t])  #example data to send (y,x (i,j)) coordinates of next target point
                                #sock.sendto(struct.pack('<iif', int(next_target[0]), int(next_target[1]), float(next_target[2])), (UDP_IP, UDP_PORT))
                                print ("13. message:", next_target)
                            else:
                                print("12. Ball not detected, skipping rotation calculation")
                            
                            # Convert path from grid to pixel coordinates
                            img_h, img_w = frame.shape[:2]
                            
                            # Grid lines removed for performance
                            # Uncomment if needed:
                            # for i in range(grid_height + 1):
                            #     y = int(round(i * img_h / grid_height))
                            #     cv2.line(frame, (0, y), (img_w, y), (100, 100, 100), 1)
                            # for j in range(grid_width + 1):
                            #     x = int(round(j * img_w / grid_width))
                            #     cv2.line(frame, (x, 0), (x, img_h), (100, 100, 100), 1)
                            
                            if len(diagonaldown_path_t2e) > 1:
                                path_pixels = []
                                for point in diagonaldown_path_t2e:
                                    x_pixel = int(point[1] * img_w / grid_width)
                                    y_pixel = int(point[0] * img_h / grid_height)
                                    path_pixels.append([x_pixel, y_pixel])
                                path_pixels = np.array(path_pixels, dtype=np.int32)
                                cv2.polylines(frame, [path_pixels], False, color=(255,0,0), thickness=2)
                            
                            # Convert markers to pixel coordinates
                            ball_pixel = (int(center_ball[0] * img_w / grid_width), int(center_ball[1] * img_h / grid_height))
                            target_pixel = (int(center_target[0] * img_w / grid_width), int(center_target[1] * img_h / grid_height))
                            for k in end_points.keys():
                                ep_coords = end_points.get(k)[0]
                                ep_pixel = (int(ep_coords[1] * img_w / grid_width), int(ep_coords[0] * img_h / grid_height))
                                cv2.drawMarker(frame, ep_pixel, (255,255,0), cv2.MARKER_DIAMOND, 40, 2)

                            
                            cv2.drawMarker(frame, ball_pixel, (0,255,0), cv2.MARKER_TILTED_CROSS, 40, 2)
                            cv2.drawMarker(frame, target_pixel, (0,0,255), cv2.MARKER_TILTED_CROSS, 40, 2)

                            print('14. target id', keys, "path_t2e:", path_t2e)
                    else:
                        print("1. Ball or target not detected.")
                        break
                
                # Display the frame once after all processing
                cv2.imshow('frame-image', frame)
                
                if quit_flag:
                    break

            # Crop rotation - only run if ball and target were detected
            if ids is not None and len(ids) > 0 and all(v[1] for v in end_points.values()):
                end_points = rotate_dict(end_points, k=2) 
                for k in end_points:
                    end_points[k][1] = False
                print("15. Rotated end points: ", end_points)
        else:
            # No markers detected, just show the frame
            cv2.imshow('frame-image', frame)
    
    if quit_flag:
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit()
