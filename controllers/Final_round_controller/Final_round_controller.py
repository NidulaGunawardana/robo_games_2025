from controller import Robot, Camera, Display
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()
display = robot.getDevice('display')

while robot.step(timestep) != -1:
    data = camera.getImage()
    
    if data:
        # Convert raw image to NumPy RGBA
        image = np.frombuffer(data, np.uint8).reshape((height, width, 4))  # RGBA
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)  # Original camera feed
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

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
            "Blue": cv2.inRange(image_hsv, lower_red, upper_red),
            "Green": cv2.inRange(image_hsv, lower_green, upper_green),
            "Red": cv2.inRange(image_hsv, lower_blue, upper_blue),
            "Yellow": cv2.inRange(image_hsv, lower_yellow, upper_yellow)
        }

        # Draw bounding boxes on original image
        for color_name, mask in color_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 30:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(image_bgr, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(image_bgr, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)

        # Convert annotated image back to RGBA for Webots Display
        annotated_rgba = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGBA)
        image_bytes = annotated_rgba.tobytes()

        # ✅ Display logic remains unchanged
        ir = display.imageNew(image_bytes, Display.BGRA, width, height)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)
