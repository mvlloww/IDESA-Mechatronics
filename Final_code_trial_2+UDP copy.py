''' Complete Optimized Version with All Functions '''

import cv2
import numpy as np
import time
import cv2.aruco as aruco
import tcod
import os
import socket
import struct
from collections import deque

### add predefined functions here ###

class OptimizedPathPlanner:
    def __init__(self, calib_path, grid_size=(20, 40)):
        # Load calibration once
        self.load_calibration(calib_path)
        
        # Initialize once
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        
        # Setup grid once
        self.grid_height, self.grid_width = grid_size
        self.grid = np.ones(grid_size, np.int8)
        
        # Precompute coordinates
        self.y_coords, self.x_coords = np.ogrid[:grid_size[0], :grid_size[1]]
        self.radius = 1
        
        # Marker constants
        self.marker_size = 40
        self.half = self.marker_size / 2
        self.center_3d = np.array([[self.half, self.half, 0]], dtype=np.float32)
        
        # State tracking
        self.ball_id = 8
        self.end_points = {1:[10,39], 2:[11,39], 3:[12,39], 4:[13,39]}
        # self.id_buffer = {}
        
        # Detection cache, avoids repeating ArUco detection
        self.last_detection = None
        self.last_detection_time = 0
        self.detection_cache_duration = 0.1  # Cache for 100ms
        
        # Path cache, avoids repeating pathfinding function
        self.path_cache = {}
        self.cache_valid_for = 0.2  # Cache paths for 200ms
        
        # Visualization
        self.display_cache = None
        self.display_cache_time = 0
        
        # UDP setup
        self.UDP_IP = "138.38.229.206"
        self.UDP_PORT = 50000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def load_calibration(self, calib_path):
        """Load calibration once"""
        Camera = np.load(calib_path)
        self.CM = Camera['CM']
        self.dist_coef = Camera['dist_coef']
    
    def detect_markers_cached(self, gray):
        """Cached marker detection to avoid redundant computations"""
        current_time = time.time()
        
        # Use cache if recent
        if (self.last_detection is not None and current_time - self.last_detection_time < self.detection_cache_duration):
            return self.last_detection
        
        # Perform detection
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        # Cache results
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.CM, self.dist_coef)
            self.last_detection = (corners, ids, rvecs, tvecs)
        # else:
        #     self.last_detection = (None, None, None, None)
        
        self.last_detection_time = current_time
        return self.last_detection
    
    def get_path_cached(self, start, end, grid_state_key):
        """Cached pathfinding with grid state validation"""
        cache_key = (tuple(start), tuple(end), grid_state_key)
        current_time = time.time()
        
        # Check cache
        if cache_key in self.path_cache:
            path, timestamp = self.path_cache[cache_key]
            if current_time - timestamp < self.cache_valid_for:
                return path
        
        # Compute new path
        path = tcod.path.path2d(cost=self.grid, start_points=[start], end_points=[end], cardinal=10, diagonal=14)
        
        # Update cache
        self.path_cache[cache_key] = (path, current_time)
        
        # Clean old cache entries
        if len(self.path_cache) > 50:  # Limit cache size
            # Remove oldest entry
            oldest = min(self.path_cache.items(), key=lambda x: x[1][1])[0]
            del self.path_cache[oldest]
        
        return path
    
    def update_grid_obstacles_batch(self, ids, rvecs, tvecs, frame, ignore_id):
        """Batch update obstacles for all markers at once"""
        # Reset grid
        self.grid.fill(1)
        
        if ids is None:
            return
        
        img_h, img_w = frame.shape[:2]
        
        # Precompute all marker centers
        centers = []
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            if ignore_id is not None and marker_id == ignore_id:
                continue
                
            center_2d, _ = cv2.projectPoints(self.center_3d, rvecs[i], tvecs[i], self.CM, self.dist_coef)
            center_pixel = center_2d[0][0].astype(int)
            
            grid_x = int(center_pixel[0] / img_w * self.grid_width)
            grid_y = int(center_pixel[1] / img_h * self.grid_height)
            
            grid_x = np.clip(grid_x, 0, self.grid_width - 1)
            grid_y = np.clip(grid_y, 0, self.grid_height - 1)
            
            centers.append((grid_x, grid_y))
        
        # Apply obstacles in batch
        for ox, oy in centers:
            # Vectorized obstacle creation
            mask = (self.x_coords - ox)**2 + (self.y_coords - oy)**2 <= self.radius**2
            self.grid[mask] = 0
        
        return self.grid
    
    def get_center_fast(self, rvec, tvec, frame):
        """Optimized center calculation"""
        img_h, img_w = frame.shape[:2]
        
        center_2d, _ = cv2.projectPoints(self.center_3d, rvec, tvec, self.CM, self.dist_coef)
        center_pixel = center_2d[0][0].astype(int)
        
        grid_x = int(center_pixel[0] / img_w * self.grid_width)
        grid_y = int(center_pixel[1] / img_h * self.grid_height)
        
        return np.clip([grid_x, grid_y], 0, [self.grid_width-1, self.grid_height-1]).astype(int)
    
    ### please chnage to desired visual function ###
    def create_visualization(self, paths, highlights):
        """Optimized visualization without Matplotlib overhead"""
        # Create display grid (use float for coloring)
        display = self.grid.astype(float)
        
        # Apply path coloring
        for i, path in enumerate(paths):
            color_value = 0.25 + (i * 0.25)  # Vary colors for different paths
            for point in path:
                if 0 <= point[0] < self.grid_height and 0 <= point[1] < self.grid_width:
                    display[point] = color_value
    
        # Apply highlights
        for point, value in highlights.items():
            if 0 <= point[0] < self.grid_height and 0 <= point[1] < self.grid_width:
                display[point] = value
        
        # Resize for display
        display_resized = cv2.resize(display, (200,400), interpolation=cv2.INTER_NEAREST)
        display_colored = cv2.applyColorMap((display_resized * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
        return display_colored
    
    def process_frame_optimized(self, frame):
        """Main processing function - optimized"""
        # Convert to grayscale once
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Get markers (cached)
        corners, ids, rvecs, tvecs = self.detect_markers_cached(gray)
        
        if ids is None or len(ids) == 0:
            return frame
        
        # Update grid with obstacles
        # Create a unique key (hash) for current grid state
        grid_state_key = hash(tuple(sorted(ids.flatten())))
        self.update_grid_obstacles_batch(ids, rvecs, tvecs, frame, ignore_id=self.ball_id)
        
        # Find ball and targets
        ball_indices = np.where(ids == self.ball_id)[0]
        
        if ball_indices.size == 0:
            return frame
        
        # Get ball position
        ball_center = self.get_center_fast(rvecs[ball_indices[0]], tvecs[ball_indices[0]], frame)
        
        # Find targets
        target_dict = {}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in self.end_points:
                target_center = self.get_center_fast(rvecs[i], tvecs[i], frame)
                
                # Get path from ball to target
                start = (int(ball_center[1]), int(ball_center[0]))
                end = (int(target_center[1]), int(target_center[0]))
                
                path = self.get_path_cached(start, end, grid_state_key)
                target_dict[marker_id] = (len(path), target_center, path)
        
        # Sort by path length
        if not target_dict:
            return frame
        
        sorted_targets = sorted(target_dict.items(), key=lambda x: x[1][0])
        
        # Process closest target
        closest_id, (path_len, target_center, path) = sorted_targets[0]
        
        # Simplify path
        if len(path) > 0:
            _, simplified_path = simplify_path(path)
        else:
            simplified_path = []
        
        # Prepare UDP data
        if len(simplified_path) > 1:
            dy = ball_center[0] - simplified_path[1][0]
            dx = ball_center[1] - simplified_path[1][1]
            
            # Get rotation
            R, _ = cv2.Rodrigues(rvecs[ball_indices[0]][0])
            yaw = np.arctan2(R[1, 0], R[0, 0])
            
            # Send UDP
            data = struct.pack('<iif', int(dy), int(dx), float(compute_theta_send(yaw)))
            self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
        
        # Create visualization
        visualization = self.create_visualization(
            paths=[path, simplified_path] if len(simplified_path) > 0 else [path],
            highlights={
                (int(ball_center[1]), int(ball_center[0])): 1.0,
                (int(target_center[1]), int(target_center[0])): 0.75
            }
        )
        
        # Overlay visualization on frame
        frame[0:200, 0:400] = visualization
        
        # Draw markers on frame
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        return frame

# Main loop using optimized class
def main_optimized():
    # Initialize once
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = os.path.join(script_dir, 'Calibration.npz')
    
    planner = OptimizedPathPlanner(calib_path)
    
    # Setup camera
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    cv2.namedWindow("frame-image", cv2.WINDOW_NORMAL)
    cv2.moveWindow("frame-image", 0, 100)
    
    # FPS tracking
    fps_counter = deque(maxlen=30)
    last_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Process frame
        processed_frame = planner.process_frame_optimized(frame)
        
        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - last_time)
        fps_counter.append(fps)
        avg_fps = sum(fps_counter) / len(fps_counter)
        last_time = current_time
        
        # Display FPS
        cv2.putText(processed_frame, f"FPS: {avg_fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('frame-image', processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_optimized()