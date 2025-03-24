
"""Movements of the robot"""
# mc.move_forward()
# mc.move_backward()
# mc.turn_right()
# mc.turn_left()
# mc.stop()

from motor import MotorController
from controller import Robot, RangeFinder, Display, Camera
import numpy as np
import cv2
import time

robot = Robot()

mc = MotorController(robot)

mc.init_motor_controller()

timestep = int(robot.getBasicTimeStep())

# Get range finder and enable it
range_finder = robot.getDevice('range-finder')
range_finder.enable(timestep)

# Get display1
display1 = robot.getDevice('display1')

# Get display2
display2 = robot.getDevice('display2')

# Get properties of the range finder
range_width = range_finder.getWidth()
range_height = range_finder.getHeight()
min_range = range_finder.getMinRange()
max_range = range_finder.getMaxRange()

# Get properties of the camera
camera = robot.getDevice('camera')
camera.enable(timestep)
cam_width = camera.getWidth()
cam_height = camera.getHeight()

def wait(robot, duration_sec):
    start_time = robot.getTime()
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        if robot.getTime() - start_time >= duration_sec:
            break
            
def is_corner(left_vals, right_vals, threshold=0.4, diff_margin=0.15):
    left_min = min(left_vals)
    right_min = min(right_vals)
    return (left_min < threshold and right_min < threshold and abs(left_min - right_min) < diff_margin)


while robot.step(timestep) != -1:
    
    # Process the range finder data navigates the robo around the maze by avoiding obstacles
    # Get range image as 2D float array
    depth_image = np.array(range_finder.getRangeImageArray())

    # Normalize depth to 0 (black) - 255 (white)
    normalized = (1.0 - (depth_image - min_range) / (max_range - min_range)) * 255.0
    normalized = np.clip(normalized, 0, 255).astype(np.uint8)

    # Convert to grayscale RGBA image
    rgba = cv2.cvtColor(normalized, cv2.COLOR_GRAY2RGBA)

    # For ground removal
    rgba[35:] = [0, 0, 0, 255] 

    # Convert to bytes and show on display
    image_bytes = rgba.tobytes()
    ir = display1.imageNew(image_bytes, Display.BGRA, range_width, range_height)
    display1.imagePaste(ir, 0, 0, False)
    display1.imageDelete(ir)
    
    right_distances = depth_image[range_height//2][range_width//2:] 
    left_distances = depth_image[range_height//2][0:range_width//2]
    
    if is_corner(left_distances, right_distances):
        print("Corner detected — backing up")
        mc.move_backward()
        wait(robot, 1)
        mc.stop()
    
        print("Turning right to escape corner")
        mc.turn_right()
        wait(robot, 4)
        mc.stop()
    
    elif (min(right_distances) < 0.25) or (min(left_distances) < 0.25):
        print("Obstacle ahead")
        if min(right_distances) < min(left_distances):
            print("Turning left ")
            mc.turn_left()
            wait(robot, 2)
            mc.stop()
        else:
            print("Turning right ")
            mc.turn_right()
            wait(robot, 2)
            mc.stop()
    else:
        mc.move_forward()
        
    # Process the camera data and show bounding boxes in the display
    # Get camera image
    cam_data = camera.getImage()
    
    if cam_data:
        # Convert raw image to NumPy RGBA
        cam_image = np.frombuffer(cam_data, np.uint8).reshape((cam_height, cam_width, 4))  # RGBA
        cam_image_bgr = cv2.cvtColor(cam_image, cv2.COLOR_RGBA2BGR)  # Original camera feed
        cam_image_hsv = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red, green, blue, and yellow
        lower_red = np.array([0, 200, 200])
        upper_red = np.array([10, 255, 255])
        
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])
        
        lower_blue = np.array([100, 200, 200])
        upper_blue = np.array([255, 255, 255])
        
        lower_yellow = np.array([20, 150, 150])
        upper_yellow = np.array([100, 255, 255])

        # Create color masks
        color_masks = {
            "Blue": cv2.inRange(cam_image_hsv, lower_red, upper_red),
            "Green": cv2.inRange(cam_image_hsv, lower_green, upper_green),
            "Red": cv2.inRange(cam_image_hsv, lower_blue, upper_blue),
            "Yellow": cv2.inRange(cam_image_hsv, lower_yellow, upper_yellow)
        }

        # Draw bounding boxes on original image
        for color_name, mask in color_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 30:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cam_image_bgr, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(cam_image_bgr, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)

        # Convert annotated image back to RGBA for Webots Display
        cam_annotated_rgba = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2RGBA)
        cam_image_bytes = cam_annotated_rgba.tobytes()

        # ✅ Display logic remains unchanged
        cam_ir = display2.imageNew(cam_image_bytes, Display.BGRA, cam_width, cam_height)
        display2.imagePaste(cam_ir, 0, 0, False)
        display2.imageDelete(cam_ir)

    pass


