import numpy as np
import cv2
from pykinect import nui

# Initialize Kinect v1
kinect = nui.Runtime()
kinect.depth_frame_ready += lambda frame: None
kinect.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.Resolution640x480, nui.ImageType.Depth)

# Kinect v1 intrinsics (approximate)
fx, fy = 525.0, 525.0
cx, cy = 320.0, 240.0

# Variable to store selected pixel
selected_pixel = None

# Function to compute real-world coordinates from Kinect depth
def get_real_world_coordinates(x, y, raw_depth):
    Z = raw_depth * 0.1  # Convert to cm (Kinect v1 scale ≈ 0.1 cm per unit)
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy
    return X, Y, Z

# Mouse click handler
def mouse_callback(event, x, y, flags, param):
    global selected_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_pixel = (x, y)

try:
    cv2.namedWindow("Depth Image")
    cv2.setMouseCallback("Depth Image", mouse_callback)

    while True:
        # Get depth frame from Kinect
        frame = kinect.depth_stream.get_frame()
        if not frame:
            continue

        # Convert to numpy array
        depth_data = np.frombuffer(frame.image.bits, dtype=np.uint16).reshape((480, 640))
        depth_image = (depth_data >> 3) & 4095  # 11-bit depth from 13-bit packed data

        # Apply color map for display
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Draw selected pixel and compute 3D
        if selected_pixel:
            x, y = selected_pixel
            raw_depth = depth_image[y, x]
            X, Y, Z = get_real_world_coordinates(x, y, raw_depth)
            text = f"X: {X:.1f}cm, Y: {Y:.1f}cm, Z: {Z:.1f}cm"
            cv2.putText(depth_colormap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(depth_colormap, (x, y), 5, (0, 0, 255), -1)

        # Show the depth image
        cv2.imshow("Depth Image", depth_colormap)

        # Exit on 'Esc'
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    kinect.close()
    cv2.destroyAllWindows()
