from controller import Robot, RangeFinder, Display
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get range finder and enable it
range_finder = robot.getDevice('range-finder')  # match your device name in Webots
range_finder.enable(timestep)

# Get display to visualize the depth as grayscale
display = robot.getDevice('display')  # match your display name in Webots

# Get properties
width = range_finder.getWidth()
height = range_finder.getHeight()
min_range = range_finder.getMinRange()
max_range = range_finder.getMaxRange()

while robot.step(timestep) != -1:
    # Get range image as 2D float array
    depth_image = np.array(range_finder.getRangeImageArray())

    # Normalize depth to 0 (black) - 255 (white) based on range limits
    normalized = (1.0 - (depth_image - min_range) / (max_range - min_range)) * 255.0
    normalized = np.clip(normalized, 0, 255).astype(np.uint8)

    # Convert to grayscale RGBA image
    rgba = cv2.cvtColor(normalized, cv2.COLOR_GRAY2RGBA)

    # Convert to bytes for Webots display
    image_bytes = rgba.tobytes()
    
    # ✅ Display logic remains unchanged
    ir = display.imageNew(image_bytes, Display.BGRA, width, height)
    display.imagePaste(ir, 0, 0, False)
    display.imageDelete(ir)

