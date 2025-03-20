import pyrealsense2 as rs
import numpy as np
import cv2

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

# Function to compute real-world coordinates
def get_real_world_coordinates(x, y, depth):
    Z = depth*100 # Convert depth to meters
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy
    return X, Y, Z

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
        
        # Display selected pixel coordinates
        if selected_pixel:
            x, y = selected_pixel
            depth_value = depth_frame.get_distance(x, y)
            X, Y, Z = get_real_world_coordinates(x, y, depth_value)
            text = f"X: {X:.3f}cm, Y: {Y:.3f}cm, Z: {Z:.3f}cm"
            cv2.putText(depth_colormap, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(depth_colormap, (x, y), 5, (0, 0, 255), -1)
        
        # Display depth image
        cv2.imshow("Depth Image", depth_colormap)
        
        # Break loop on key press
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
