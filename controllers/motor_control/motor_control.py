from motor import MotorController
from controller import Robot, RangeFinder, Display
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

# Get display
display = robot.getDevice('display')

# Get properties
width = range_finder.getWidth()
height = range_finder.getHeight()
min_range = range_finder.getMinRange()
max_range = range_finder.getMaxRange()

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
    ir = display.imageNew(image_bytes, Display.BGRA, width, height)
    display.imagePaste(ir, 0, 0, False)
    display.imageDelete(ir)
    
    right_distances = depth_image[height//2][width//2:] 
    left_distances = depth_image[height//2][0:width//2]
    
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
    
    """Movements of the robot"""
    # mc.move_forward()
    # mc.move_backward()
    # mc.turn_right()
    # mc.turn_left()
    # mc.stop()
    pass


