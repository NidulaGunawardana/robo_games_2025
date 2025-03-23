import pyrealsense2 as rs
import numpy as np
import cv2
from sklearn.cluster import DBSCAN
from collections import deque

# Initialize RealSense pipeline
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(config)

# Get camera intrinsics
profile = pipe.get_active_profile()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = intrinsics.fx, intrinsics.fy
cx, cy = intrinsics.ppx, intrinsics.ppy

# Variables to store selected pixel coordinates
selected_pixel = None

# Grid parameters
grid_size = 300  # 300x300 cells
cell_size = 0.1  # Each cell represents 0.1 cm

# DBSCAN parameters
dbscan_eps = 30  # Epsilon for clustering (in grid units)
dbscan_min_samples = 20  # Minimum points per cluster

# Bounding box smoothing buffer
bbox_history = deque(maxlen=5)  # Stores last 5 frames for smoothing

def get_real_world_coordinates(x, y, depth):
    Z = depth * 100  # Convert depth to cm
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy  # Compute Y coordinate
    return X, Y, Z - 15  # Projecting onto the X-Z plane

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global selected_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_pixel = (x, y)

try:
    cv2.namedWindow("Depth Image")
    cv2.setMouseCallback("Depth Image", mouse_callback)
    
    while True:
        # Capture frames
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue
        
        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        # Create top-view grid
        grid = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)
        
        # Store valid points for clustering
        points = []
        
        # Iterate over depth image and map to grid
        height, width = depth_image.shape
        step_size = 5  # Increase resolution by reducing step size
        for i in range(0, height, step_size):
            for j in range(0, width, step_size):
                depth_value = depth_frame.get_distance(j, i)
                if depth_value == 0 or depth_value > 3:  # Ignore invalid depth values
                    continue
                
                X, Y, Z = get_real_world_coordinates(j, i, depth_value)
                
                # Ground removal condition (ignore points below -10cm in Y coordinate)
                if Y > 10:
                    continue
                
                grid_x = int((X + (grid_size * cell_size / 2)) / cell_size)
                grid_z = int((Z + (grid_size * cell_size / 2)) / cell_size)
                
                if 0 <= grid_x < grid_size and 0 <= grid_z < grid_size:
                    grid[grid_z, grid_x] = (255, 255, 255)  # Mark occupied cell
                    points.append([grid_x, grid_z])
        
        # Object detection using DBSCAN clustering
        bbox_list = []
        if len(points) > dbscan_min_samples:
            points = np.array(points)
            clustering = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(points)
            labels = clustering.labels_
            unique_labels = set(labels)
            
            for label in unique_labels:
                if label == -1:
                    continue  # Ignore noise points
                
                cluster_points = points[labels == label]
                if len(cluster_points) == 0:
                    continue
                
                # Compute bounding box directly
                x_min, y_min = np.min(cluster_points, axis=0)
                x_max, y_max = np.max(cluster_points, axis=0)
                bbox_list.append((x_min, y_min, x_max, y_max))
        
        # Smooth bounding boxes using past history
        bbox_history.append(bbox_list)
        
        # Draw bounding boxes
        for bbox in bbox_list:
            x_min, y_min, x_max, y_max = map(int, bbox)
            cv2.rectangle(grid, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        
        # Scale grid for visualization
        grid_display = cv2.resize(grid, (300, 300), interpolation=cv2.INTER_NEAREST)
        
        # Display selected pixel coordinates
        if selected_pixel:
            x, y = selected_pixel
            depth_value = depth_frame.get_distance(x, y)
            X, Y, Z = get_real_world_coordinates(x, y, depth_value)
            text = f"X: {X:.3f}cm, Y: {Y:.3f}cm, Z: {Z:.3f}cm"
            cv2.putText(depth_colormap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(depth_colormap, (x, y), 5, (0, 0, 255), -1)
        
        # Display images
        cv2.imshow("Depth Image", depth_colormap)
        cv2.imshow("Top View Grid", grid_display)
        
        # Break loop on key press
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
