import cv2
import numpy as np
import pyrealsense2 as rs

# Setup RealSense pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipe.start(cfg)

# Create alignment primitive to align depth to color
align = rs.align(rs.stream.color)
colorizer = rs.colorizer()

try:
    while True:
        # Wait for frames and align them
        frameset = pipe.wait_for_frames()
        frameset = align.process(frameset)

        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # Get depth at the center of the image
        width, height = depth_frame.get_width(), depth_frame.get_height()
        center_x, center_y = width // 2, height // 2
        depth_value = depth_frame.get_distance(center_x, center_y)

        # Display depth value on the image
        text = f"Depth at center: {depth_value:.3f} meters"
        cv2.putText(color_image, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Stack color and depth images side by side
        stacked_images = np.hstack((color_image, depth_colormap))

        # Show the images
        cv2.imshow("Color and Depth View", stacked_images)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
