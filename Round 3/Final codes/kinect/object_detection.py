import freenect
import numpy as np
import cv2
from sklearn.cluster import DBSCAN
from collections import deque

# Approximate camera intrinsics for Kinect v1 depth
fx, fy = 594.21, 591.04
cx, cy = 339.5, 242.7

# Selected pixel tracking
selected_pixel = None

# Grid parameters
grid_size = 300
cell_size = 1  # Each cell = 0.1 cm

# DBSCAN clustering parameters
dbscan_eps = 30
dbscan_min_samples = 20

# Bounding box smoothing
bbox_history = deque(maxlen=5)

def get_depth_and_rgb():
    depth, _ = freenect.sync_get_depth()
    rgb, _ = freenect.sync_get_video()
    return depth, rgb

def get_real_world_coordinates(x, y, depth):
    Z = depth * 0.1  # Approx: depth in mm → cm
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy
    return X, Y, Z - 15

def mouse_callback(event, x, y, flags, param):
    global selected_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_pixel = (x, y)

# Create window and register mouse
cv2.namedWindow("Depth Image")
cv2.setMouseCallback("Depth Image", mouse_callback)

try:
    while True:
        # Get frames
        depth_raw, rgb = get_depth_and_rgb()
        color_frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Apply colormap for depth display
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)

        # Occupancy grid
        grid = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)

        # Clustering points
        points = []

        height, width = depth_raw.shape
        step_size = 5

        for i in range(0, height, step_size):
            for j in range(0, width, step_size):
                depth_value = depth_raw[i, j]
                if depth_value == 0 or depth_value > 2047:  # Skip invalid data
                    continue

                X, Y, Z = get_real_world_coordinates(j, i, depth_value)

                if Y > 10:  # Remove ground
                    continue

                grid_x = int((X + (grid_size * cell_size / 2)) / cell_size)
                grid_z = int((Z + (grid_size * cell_size / 2)) / cell_size)

                if 0 <= grid_x < grid_size and 0 <= grid_z < grid_size:
                    grid[grid_z, grid_x] = (255, 255, 255)
                    points.append([grid_x, grid_z])

        # DBSCAN clustering
        bbox_list = []
        if len(points) > dbscan_min_samples:
            points = np.array(points)
            clustering = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(points)
            labels = clustering.labels_

            for label in set(labels):
                if label == -1:
                    continue
                cluster = points[labels == label]
                x_min, y_min = np.min(cluster, axis=0)
                x_max, y_max = np.max(cluster, axis=0)
                bbox_list.append((x_min, y_min, x_max, y_max))

        bbox_history.append(bbox_list)

        # Draw bounding boxes
        for bbox in bbox_list:
            x_min, y_min, x_max, y_max = map(int, bbox)
            cv2.rectangle(grid, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Resize for display
        grid_display = cv2.resize(grid, (300, 300), interpolation=cv2.INTER_NEAREST)

        # Selected pixel display
        if selected_pixel:
            x, y = selected_pixel
            if 0 <= x < width and 0 <= y < height:
                depth_value = depth_raw[y, x]
                if depth_value != 0:
                    X, Y, Z = get_real_world_coordinates(x, y, depth_value)
                    text = f"X: {X:.2f}cm, Y: {Y:.2f}cm, Z: {Z:.2f}cm"
                    cv2.putText(depth_colormap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 255, 255), 2)
                    cv2.circle(depth_colormap, (x, y), 5, (0, 0, 255), -1)

        # Show windows
        cv2.imshow("Depth Image", depth_colormap)
        cv2.imshow("Top View Grid", grid_display)

        # Exit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cv2.destroyAllWindows()
