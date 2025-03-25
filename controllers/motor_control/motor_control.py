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
        
        # Parameters for red object detection
        self.red_area_threshold = 80  # Experimentally tuned threshold
        
        # State variables
        self.state = "NAVIGATE"      # Two states: "NAVIGATE" and "APPROACH_CUBE"
        self.cube_info = None        # To store cube bounding box and average depth

    def wait(self, duration_sec):
        start_time = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            if self.robot.getTime() - start_time >= duration_sec:
                break

    def process_range_finder(self):
        # Get range image as 2D float array
        depth_image = np.array(self.range_finder.getRangeImageArray())
        
        # Normalize depth image for display (0 to 255)
        normalized = (1.0 - (depth_image - self.min_range) / (self.max_range - self.min_range)) * 255.0
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)
        
        # Convert to RGBA for display
        rgba = cv2.cvtColor(normalized, cv2.COLOR_GRAY2RGBA)
        # Optionally, remove ground details for clarity:
        # rgba[35:] = [0, 0, 0, 255]
        
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
        # Divide the range finder image into left and right halves at a fixed vertical offset.
        # The offset (here, +23) may be tuned based on your sensor mounting.
        right_distances = depth_image[self.range_height // 2 + 23][self.range_width // 2:]
        left_distances = depth_image[self.range_height // 2 + 23][:self.range_width // 2]
        
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

    def process_camera(self, _depth_image):
        """
        Process the camera image to detect red objects and determine whether they are
        the red placement area or the red cube. If a red cube is detected, its bounding box
        and average depth are stored in self.cube_info.
        """
        cam_data = self.camera.getImage()
        if not cam_data:
            return
        
        # Convert raw image data into a NumPy array (RGBA)
        cam_image = np.frombuffer(cam_data, np.uint8).reshape((self.cam_height, self.cam_width, 4))
        cam_image_bgr = cv2.cvtColor(cam_image, cv2.COLOR_RGBA2BGR)
        cam_image_hsv = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for red objects (two ranges for brightness variations)
        hsv_ranges = {
            "Red1": (np.array([120, 80, 200]), np.array([245, 255, 255])),
            "Red2": (np.array([115, 130, 70]), np.array([130, 200, 75])),
        }
        
        # Reset cube info each frame
        self.cube_info = None
        
        for color_name, (lower, upper) in hsv_ranges.items():
            mask = cv2.inRange(cam_image_hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 30:  # Filter out noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    object_label = color_name  # default label
                    
                    # Use depth data (if available) to estimate the real-world size.
                    if _depth_image is not None:
                        # Map the bounding box from camera space to depth sensor space
                        depth_x = int(x * self.range_width / self.cam_width)
                        depth_y = int(y * self.range_height / self.cam_height)
                        depth_w = int(w * self.range_width / self.cam_width)
                        depth_h = int(h * self.range_height / self.cam_height)
                        
                        # Ensure indices are within bounds
                        depth_x = max(0, min(depth_x, self.range_width - 1))
                        depth_y = max(0, min(depth_y, self.range_height - 1))
                        depth_w = max(1, min(depth_w, self.range_width - depth_x))
                        depth_h = max(1, min(depth_h, self.range_height - depth_y))
                        
                        roi_depth = _depth_image[depth_y:depth_y+depth_h, depth_x:depth_x+depth_w]
                        avg_depth = np.mean(roi_depth) if roi_depth.size > 0 else 1.0
                        
                        # Calculate a depth-corrected size metric:
                        # Here we multiply by avg_depth^2 so that the metric remains roughly constant for a given physical size.
                        corrected_area = area * (avg_depth**2 + 1e-6)
                        
                        # Use threshold to differentiate objects
                        if corrected_area > self.red_area_threshold:
                            object_label = "Red Placement"
                        else:
                            object_label = "Red Cube"
                            # Store cube information (bounding box and depth)
                            # We store the one detected first. You might want to choose the best candidate in a real system.
                            self.cube_info = (x, y, w, h, avg_depth)
                        
                        # Optionally, display debug info on the image.
                        cv2.putText(cam_image_bgr, f"D: {avg_depth:.2f}", (20, y + h),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
                        cv2.putText(cam_image_bgr, f"CA: {corrected_area:.2f}", (20, y + h + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
                    
                    # Draw bounding box and label on the image.
                    cv2.rectangle(cam_image_bgr, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    cv2.putText(cam_image_bgr, object_label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.3, (0, 255, 255), 2)
        
        # Convert annotated image back to RGBA and display it on display2.
        cam_annotated_rgba = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2RGBA)
        cam_image_bytes = cam_annotated_rgba.tobytes()
        cam_ir = self.display2.imageNew(cam_image_bytes, Display.BGRA, self.cam_width, self.cam_height)
        self.display2.imagePaste(cam_ir, 0, 0, False)
        self.display2.imageDelete(cam_ir)

    def approach_cube(self, cube_info):
        """
        Given cube_info as (x, y, w, h, avg_depth), adjust the robot’s heading so that the cube is centered.
        Then drive forward until the cube reaches the target distance.
        """
        x, y, w, h, avg_depth = cube_info
        cube_center_x = x + w/2
        cube_center_y = y + h/2
        image_center_x = self.cam_width / 2
        error = cube_center_x - image_center_x
        threshold_pixels = 15  # acceptable error threshold in pixels
        
        # If cube is off-center, turn the robot
        if abs(error) > threshold_pixels:
            if error > 0:
                print("Cube detected to the right; turning right to center it.")
                self.mc.turn_right()
            else:
                print("Cube detected to the left; turning left to center it.")
                self.mc.turn_left()
            self.wait(0.5)
            self.mc.stop()
        else:
            # Cube is centered: if it's still far away, move forward.
            if cube_center_y < self.cam_height//2 + 25:  # add a small tolerance
                print("Cube centered; moving forward.")
                self.mc.move_forward()
            else:
                print("Cube reached; stopping.")
                self.mc.stop()
                # Optionally, here you could trigger further actions like grasping the cube.
                # Once finished, revert to navigation.
                self.state = "NAVIGATE"
                self.cube_info = None

    def run(self):
        while self.robot.step(self.timestep) != -1:
            depth_image = self.process_range_finder()
            
            if self.state == "NAVIGATE":
                # In navigation, perform obstacle avoidance and process camera.
                self.obstacle_avoidance(depth_image)
                self.process_camera(depth_image)
                # If a cube is detected, transition to the approach state.
                if self.cube_info is not None:
                    print("Red Cube detected; switching to APPROACH_CUBE state.")
                    self.state = "APPROACH_CUBE"
                    
            elif self.state == "APPROACH_CUBE":
                # While approaching, update the camera to get the latest cube info.
                self.process_camera(depth_image)
                if self.cube_info is not None:
                    self.approach_cube(self.cube_info)
                else:
                    print("Cube lost; reverting to NAVIGATE state.")
                    self.state = "NAVIGATE"

if __name__ == "__main__":
    navigator = RobotNavigator()
    navigator.run()
