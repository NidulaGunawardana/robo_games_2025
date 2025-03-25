from controller import Robot, Camera, Display
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()

# Assuming your simulation has two displays named 'display1' and 'display2'
display1 = robot.getDevice('display1')
display2 = robot.getDevice('display2')

# Tuning parameters for each color separately

# Blue color tuning
blue_lower = np.array([0, 200, 200])
blue_upper = np.array([10, 255, 255])

# Green color tuning
green_lower = np.array([40, 70, 70])
green_upper = np.array([80, 255, 255])

# Bright Red color tuning
b_red_lower = np.array([120, 80, 200])
b_red_upper = np.array([245, 255, 255])

# Dark Red color tuning
d_red_lower = np.array([115, 130, 70])
d_red_upper = np.array([130, 200, 75])

# Yellow color tuning 
yellow_lower = np.array([20, 150, 150])
yellow_upper = np.array([40, 255, 255])

while robot.step(timestep) != -1:
    data = camera.getImage()
    
    if data:
        # Convert raw image to a NumPy array (RGBA)
        image = np.frombuffer(data, np.uint8).reshape((height, width, 4))
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # Create separate masks for each color
        mask_b_red = cv2.inRange(image_hsv, b_red_lower, b_red_upper)
        mask_d_red = cv2.inRange(image_hsv, d_red_lower, d_red_upper)
        mask_green = cv2.inRange(image_hsv, green_lower, green_upper)
        mask_blue = cv2.inRange(image_hsv, blue_lower, blue_upper)
        mask_yellow = cv2.inRange(image_hsv, yellow_lower, yellow_upper)
        
        # Collect masks in a dictionary for easier processing
        masks = {
            "Red1": mask_b_red,
            "Red2": mask_d_red,
            "Green": mask_green,
            "Blue": mask_blue,
            "Yellow": mask_yellow
        }
        
        # Create a copy for drawing bounding boxes
        annotated_image = image_bgr.copy()
        # Prepare a black image for isolated color display
        isolated_image = np.zeros_like(image_bgr)
        
        # Process each color mask
        for color_name, mask in masks.items():
            # Find contours to draw bounding boxes
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 30:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(annotated_image, color_name, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            # Isolate the detected color using bitwise AND
            isolated_color = cv2.bitwise_and(image_bgr, image_bgr, mask=mask)
            isolated_image = cv2.add(isolated_image, isolated_color)
        
        # Convert the annotated image (with bounding boxes) to RGBA and display on display1
        annotated_rgba = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGBA)
        annotated_bytes = annotated_rgba.tobytes()
        ir1 = display1.imageNew(annotated_bytes, Display.BGRA, width, height)
        display1.imagePaste(ir1, 0, 0, False)
        display1.imageDelete(ir1)
        
        # Convert the isolated image (mask applied) to RGBA and display on display2
        isolated_rgba = cv2.cvtColor(isolated_image, cv2.COLOR_BGR2RGBA)
        isolated_bytes = isolated_rgba.tobytes()
        ir2 = display2.imageNew(isolated_bytes, Display.BGRA, width, height)
        display2.imagePaste(ir2, 0, 0, False)
        display2.imageDelete(ir2)
