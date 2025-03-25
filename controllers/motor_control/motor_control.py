from motor import MotorController
from controller import Robot, RangeFinder, Display, Camera
import numpy as np
import cv2
import time

class RobotNavigator:
    def __init__(self):
        # Initialize the robot and devices
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize motor controller
        self.mc = MotorController(self.robot)
        self.mc.init_motor_controller()
        
        # Range finder setup
        self.range_finder = self.robot.getDevice('range-finder')
        self.range_finder.enable(self.timestep)
        self.range_width = self.range_finder.getWidth()
        self.range_height = self.range_finder.getHeight()
        self.min_range = self.range_finder.getMinRange()
        self.max_range = self.range_finder.getMaxRange()
        
        # Displays for range and camera images
        self.display1 = self.robot.getDevice('display1')
        self.display2 = self.robot.getDevice('display2')
        
        # Camera setup
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        self.cam_width = self.camera.getWidth()
        self.cam_height = self.camera.getHeight()
        
    def wait(self, duration_sec):
        start_time = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            if self.robot.getTime() - start_time >= duration_sec:
                break

    def process_range_finder(self):
        # Get range image as 2D float array
        depth_image = np.array(self.range_finder.getRangeImageArray())
        
        # Normalize depth image to displayable values
        normalized = (1.0 - (depth_image - self.min_range) / (self.max_range - self.min_range)) * 255.0
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)
        
        # Convert to RGBA for display
        rgba = cv2.cvtColor(normalized, cv2.COLOR_GRAY2RGBA)
        # Ground removal for clarity
        rgba[35:] = [0, 0, 0, 255]
        
        # Display image on display1
        image_bytes = rgba.tobytes()
        ir = self.display1.imageNew(image_bytes, Display.BGRA, self.range_width, self.range_height)
        self.display1.imagePaste(ir, 0, 0, False)
        self.display1.imageDelete(ir)
        
        return depth_image

    @staticmethod
    def is_corner(left_vals, right_vals, threshold=0.4, diff_margin=0.15):
        left_min = min(left_vals)
        right_min = min(right_vals)
        return (left_min < threshold and right_min < threshold and abs(left_min - right_min) < diff_margin)
    
    def obstacle_avoidance(self, depth_image):
        # Divide the range finder image into left and right halves at mid-height
        right_distances = depth_image[self.range_height // 2][self.range_width // 2:]
        left_distances = depth_image[self.range_height // 2][:self.range_width // 2]
        
        if self.is_corner(left_distances, right_distances):
            print("Corner detected — backing up")
            self.mc.move_backward()
            self.wait(1)
            self.mc.stop()
            print("Turning right to escape corner")
            self.mc.turn_right()
            self.wait(4)
            self.mc.stop()
        elif (min(right_distances) < 0.25) or (min(left_distances) < 0.25):
            print("Obstacle ahead")
            if min(right_distances) < min(left_distances):
                print("Turning left")
                self.mc.turn_left()
                self.wait(2)
                self.mc.stop()
            else:
                print("Turning right")
                self.mc.turn_right()
                self.wait(2)
                self.mc.stop()
        else:
            self.mc.move_forward()

    def process_camera(self):
        cam_data = self.camera.getImage()
        if not cam_data:
            return
        
        # Convert raw image data into a NumPy array (RGBA)
        cam_image = np.frombuffer(cam_data, np.uint8).reshape((self.cam_height, self.cam_width, 4))
        cam_image_bgr = cv2.cvtColor(cam_image, cv2.COLOR_RGBA2BGR)
        cam_image_hsv = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2HSV)
        
        # RED1 -> Bright Red
        # RED2 -> Dark Red
        
        # Define HSV ranges for color detection
        hsv_ranges = {
            "Red1":(np.array([120, 80, 200]), np.array([245, 255, 255])),
            "Red2":(np.array([115, 130, 70]), np.array([130, 200, 75])),
        }
        # "Green": (np.array([40, 70, 70]), np.array([80, 255, 255])),
        # "Blue":  (np.array([100, 200, 200]), np.array([255, 255, 255])),
        # "Yellow": (np.array([20, 150, 150]), np.array([100, 255, 255]))
        
        # Process each color
        for color_name, (lower, upper) in hsv_ranges.items():
            mask = cv2.inRange(cam_image_hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 30:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cam_image_bgr, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    cv2.putText(cam_image_bgr, color_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)
        
        # Convert annotated image back to RGBA and display on display2
        cam_annotated_rgba = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2RGBA)
        cam_image_bytes = cam_annotated_rgba.tobytes()
        cam_ir = self.display2.imageNew(cam_image_bytes, Display.BGRA, self.cam_width, self.cam_height)
        self.display2.imagePaste(cam_ir, 0, 0, False)
        self.display2.imageDelete(cam_ir)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            depth_image = self.process_range_finder()
            self.obstacle_avoidance(depth_image)
            self.process_camera()
            # Future implementation for approaching and capturing objects
            # can be integrated here, e.g., by checking detected colors
            # and altering the robot's state accordingly.

if __name__ == "__main__":
    navigator = RobotNavigator()
    navigator.run()
