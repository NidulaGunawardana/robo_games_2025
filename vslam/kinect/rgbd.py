import numpy as np
import cv2
import os
from pykinect import nui
import threading

# Create output directory
output_dir = "captured_images"
os.makedirs(output_dir, exist_ok=True)

# Image buffers
color_image = None
depth_image = None
captured_count = 0
num_captures = 4
frame_ready = threading.Event()

# Approximate Kinect intrinsics
depth_scale = 0.1  # raw depth unit ≈ 0.1 cm

# Initialize Kinect
kinect = nui.Runtime()
kinect.video_stream.open(nui.ImageStreamType.Video, 2, nui.ImageResolution.Resolution640x480, nui.ImageType.Color)
kinect.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.Resolution640x480, nui.ImageType.Depth)

# Frame callbacks
def video_frame_handler(frame):
    global color_image
    video = np.frombuffer(frame.image.bits, dtype=np.uint8)
    color_image = video.reshape((480, 640, 4))[..., :3]  # Drop alpha channel
    frame_ready.set()

def depth_frame_handler(frame):
    global depth_image
    raw = np.frombuffer(frame.image.bits, dtype=np.uint16).reshape((480, 640))
    depth_image = (raw >> 3) & 4095  # 11-bit depth
    frame_ready.set()

kinect.video_frame_ready += video_frame_handler
kinect.depth_frame_ready += depth_frame_handler

print("Press 'SPACE' to capture an image. Press 'Q' to quit.")

try:
    cv2.namedWindow("Kinect Capture")

    while captured_count < num_captures:
        frame_ready.wait()
        frame_ready.clear()

        if color_image is None or depth_image is None:
            continue

        # Normalize depth for visualization
        depth_display = cv2.convertScaleAbs(depth_image, alpha=255.0 / depth_image.max())

        # Stack color and depth side-by-side
        preview = np.hstack((color_image, cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)))
        cv2.imshow("Kinect Capture", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            # Save images
            color_filename = os.path.join(output_dir, f"color_{captured_count + 1}.png")
            depth_filename = os.path.join(output_dir, f"depth_{captured_count + 1}.png")

            cv2.imwrite(color_filename, color_image)
            cv2.imwrite(depth_filename, depth_display)

            print(f"Saved: {color_filename} & {depth_filename}")
            captured_count += 1

        elif key == ord('q'):
            print("Exiting...")
            break

finally:
    kinect.close()
    cv2.destroyAllWindows()
