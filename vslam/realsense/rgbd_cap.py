import pyrealsense2 as rs
import numpy as np
import cv2
import os

# Create output directory
output_dir = "captured_images"
os.makedirs(output_dir, exist_ok=True)

# Initialize RealSense pipeline
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipe.start(config)

# Get depth sensor's scale (to convert depth values to meters)
profile = pipe.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create alignment primitive to align depth to color
align = rs.align(rs.stream.color)

# Capture 4 images manually
num_captures = 4
captured_count = 0

print("Press 'SPACE' to capture an image. Press 'Q' to quit.")

try:
    while captured_count < num_captures:
        # Wait for frames and align depth to color
        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Normalize depth image for better visualization (Grayscale)
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=255 / depth_image.max())

        # Show side-by-side preview
        stacked_images = np.hstack((color_image, cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)))
        cv2.imshow("Press 'SPACE' to capture, 'Q' to quit", stacked_images)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # Capture on SPACEBAR press
            color_filename = os.path.join(output_dir, f"color_{captured_count + 1}.png")
            depth_filename = os.path.join(output_dir, f"depth_{captured_count + 1}.png")

            cv2.imwrite(color_filename, color_image)
            cv2.imwrite(depth_filename, depth_gray)

            print(f"Saved: {color_filename} & {depth_filename}")
            captured_count += 1

        elif key == ord('q'):  # Quit on 'Q' press
            print("Exiting...")
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
    