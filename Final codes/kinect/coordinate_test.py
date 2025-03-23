import freenect
import numpy as np
import cv2

# Approximate camera intrinsics for Kinect v1
fx, fy = 594.21, 591.04
cx, cy = 339.5, 242.7

# Variable to store selected pixel coordinates
selected_pixel = None

# Function to get RGB and depth data from the Kinect
def get_depth_and_rgb():
    depth, _ = freenect.sync_get_depth()
    rgb, _ = freenect.sync_get_video()
    return depth, rgb

# Function to convert pixel to real-world coordinates
def get_real_world_coordinates(x, y, depth):
    Z = depth * 0.1  # Convert depth from mm to cm (scale approx)
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy
    return X, Y, Z

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global selected_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_pixel = (x, y)

# Setup window and callback
cv2.namedWindow("Depth Image")
cv2.setMouseCallback("Depth Image", mouse_callback)

try:
    while True:
        # Get depth and RGB frames
        depth_raw, rgb = get_depth_and_rgb()

        # Create colormap for visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_raw, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # Convert RGB to BGR for OpenCV
        color_frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Display 3D coordinates of selected pixel
        if selected_pixel:
            x, y = selected_pixel
            depth_value = depth_raw[y, x]
            if depth_value != 0:
                X, Y, Z = get_real_world_coordinates(x, y, depth_value)
                text = f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: {Z:.1f}cm"
                cv2.putText(depth_colormap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
                cv2.circle(depth_colormap, (x, y), 5, (0, 0, 255), -1)

        # Show depth image
        cv2.imshow("Depth Image", depth_colormap)

        # Exit on 'Esc'
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cv2.destroyAllWindows()
