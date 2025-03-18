import pyrealsense2 as rs
import numpy as np
import cv2
import os

# Create output directory
output_dir = "coordinate_images"
os.makedirs(output_dir, exist_ok=True)

# Initialize RealSense pipeline
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(config)

# Get camera intrinsics
profile = pipe.get_active_profile()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

try:
    # Capture a single frame
    frames = pipe.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if depth_frame and color_frame:
        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data()) * 0.001  # Convert to meters

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())  # Original RGB image

        # Get depth dimensions
        height, width = depth_image.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy

        # Generate pixel grid
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))

        # Compute real-world coordinates (X, Y, Z)
        Z = depth_image  # Depth (meters)
        X = (xx - cx) * Z / fx  # X coordinate (Left-Right)
        Y = (yy - cy) * Z / fy  # Y coordinate (Up-Down)

        # Normalize values for image representation
        def normalize_to_8bit(array):
            array = np.nan_to_num(array, nan=0.0)  # Replace NaN values
            min_val, max_val = np.min(array), np.max(array)
            return np.uint8(255 * (array - min_val) / (max_val - min_val))

        # Convert X, Y, and Z to 8-bit grayscale images
        X_image = normalize_to_8bit(X)
        Y_image = normalize_to_8bit(Y)
        Z_image = normalize_to_8bit(Z)

        # Save images
        cv2.imwrite(os.path.join(output_dir, "X_image.png"), X_image)
        cv2.imwrite(os.path.join(output_dir, "Y_image.png"), Y_image)
        cv2.imwrite(os.path.join(output_dir, "Z_image.png"), Z_image)
        cv2.imwrite(os.path.join(output_dir, "Color_image.png"), color_image)

        print("Saved: X_image.png, Y_image.png, Z_image.png, Color_image.png")

        # Display images for comparison
        cv2.imshow("X Coordinate Image", X_image)
        cv2.imshow("Y Coordinate Image", Y_image)
        cv2.imshow("Z Coordinate Image", Z_image)
        cv2.imshow("Original Color Image", color_image)

        cv2.waitKey(0)  # Wait for key press before closing

finally:
    pipe.stop()
    cv2.destroyAllWindows()
