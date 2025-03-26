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
        
        # Threshold for object detection (area threshold)
        self.area_threshold = 100  # Experimentally tuned threshold
        
        # Sequence order for box placements
        self.order = ["Red", "Yellow", "Green", "Blue"]
        self.current_index = 0
        self.current_target = self.order[self.current_index]
        
        # State variables
        # States: "NAVIGATE" (normal movement), "SEARCH_CUBE", "APPROACH_CUBE",
        # "SEARCH_PLACEMENT", "APPROACH_PLACEMENT", "DONE"
        self.state = "SEARCH_CUBE"      
        self.cube_info = None        # Stores cube bounding box and average depth (for target color)
        self.placement_info = None   # Stores placement area info (for target color)
        self.cube_lost_count = 0     # Counts how many times the cube has been lost

    def wait(self, duration_sec):
        start_time = self.robot.getTime()
        while self.robot.step(self.timestep) != -1:
            if self.robot.getTime() - start_time >= duration_sec:
                break

    def process_range_finder(self):
        # Get range image as a 2D float array
        depth_image = np.array(self.range_finder.getRangeImageArray())
        
        # Normalize depth image for display (0 to 255)
        normalized = (1.0 - (depth_image - self.min_range) / (self.max_range - self.min_range)) * 255.0
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)
        
        # Convert to RGBA for display
        rgba = cv2.cvtColor(normalized, cv2.COLOR_GRAY2RGBA)
        
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
        Process the camera image to detect objects of various colors.
        Determines whether they are cubes or placement areas based on a depth-corrected size.
        Only stores information for objects matching the current target color.
        """
        cam_data = self.camera.getImage()
        if not cam_data:
            return

        # Convert raw image data into a NumPy array (RGBA)
        cam_image = np.frombuffer(cam_data, np.uint8).reshape((self.cam_height, self.cam_width, 4))
        cam_image_bgr = cv2.cvtColor(cam_image, cv2.COLOR_RGBA2BGR)
        cam_image_hsv = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red, green, yellow, and blue.
        hsv_ranges = {
            "Red": [
                (np.array([120, 80, 200]), np.array([245, 255, 255])),
                (np.array([115, 130, 70]), np.array([130, 200, 75]))
            ],
            "Green": [
                (np.array([40, 70, 70]), np.array([80, 255, 255]))
            ],
            "Yellow": [
                (np.array([80, 180, 70]), np.array([90, 240, 220]))
            ],
            "Blue": [
                (np.array([0, 230, 210]), np.array([10, 255, 255])),
                (np.array([0, 200, 90]), np.array([10, 255, 255])),
            ],
        }

        # Reset object information each frame
        self.cube_info = None
        self.placement_info = None

        for color_name, ranges in hsv_ranges.items():
            for lower, upper in ranges:
                mask = cv2.inRange(cam_image_hsv, lower, upper)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 30:  # Filter out noise
                        x, y, w, h = cv2.boundingRect(cnt)
                        object_label = color_name
                        
                        if _depth_image is not None:
                            # Map bounding box from camera space to depth sensor space
                            depth_x = int(x * self.range_width / self.cam_width)
                            depth_y = int(y * self.range_height / self.cam_height)
                            depth_w = int(w * self.range_width / self.cam_width)
                            depth_h = int(h * self.range_height / self.cam_height)

                            depth_x = max(0, min(depth_x, self.range_width - 1))
                            depth_y = max(0, min(depth_y, self.range_height - 1))
                            depth_w = max(1, min(depth_w, self.range_width - depth_x))
                            depth_h = max(1, min(depth_h, self.range_height - depth_y))

                            roi_depth = _depth_image[depth_y:depth_y+depth_h, depth_x:depth_x+depth_w]
                            avg_depth = np.mean(roi_depth) if roi_depth.size > 0 else 1.0

                            # Calculate a depth-corrected size metric
                            corrected_area = area * (avg_depth**2 + 1e-6)

                            # Only store info for objects matching the current target color
                            if color_name == self.current_target:
                                if corrected_area > self.area_threshold:
                                    object_label += " Placement"
                                    self.placement_info = (x, y, w, h, avg_depth)
                                else:
                                    object_label += " Cube"
                                    self.cube_info = (x, y, w, h, avg_depth)

                            # Optional: overlay debug information
                            cv2.putText(cam_image_bgr, f"D: {avg_depth:.2f}", (20, y + h),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
                            cv2.putText(cam_image_bgr, f"CA: {corrected_area:.2f}", (20, y + h + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
                        
                        # Draw bounding box and label on the image.
                        cv2.rectangle(cam_image_bgr, (x, y), (x+w, y+h), (0, 255, 255), 2)
                        cv2.putText(cam_image_bgr, object_label, (x, y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 2)

        # Convert annotated image back to RGBA and display it on display2.
        cam_annotated_rgba = cv2.cvtColor(cam_image_bgr, cv2.COLOR_BGR2RGBA)
        cam_image_bytes = cam_annotated_rgba.tobytes()
        cam_ir = self.display2.imageNew(cam_image_bytes, Display.BGRA, self.cam_width, self.cam_height)
        self.display2.imagePaste(cam_ir, 0, 0, False)
        self.display2.imageDelete(cam_ir)

    def approach_cube(self, cube_info):
        """
        Adjust the robot’s heading to center the cube then drive forward.
        Once the cube is reached (i.e. centered and near), transition to search for the placement area.
        """
        x, y, w, h, avg_depth = cube_info
        cube_center_x = x + w/2
        cube_center_y = y + h/2
        image_center_x = self.cam_width / 2
        error = cube_center_x - image_center_x
        threshold_pixels = 20  # acceptable error threshold
        
        if abs(error) > threshold_pixels:
            if error > 0:
                print(f"{self.current_target} Cube detected to the right; turning right to center it.")
                self.mc.turn_right()
            else:
                print(f"{self.current_target} Cube detected to the left; turning left to center it.")
                self.mc.turn_left()
            self.wait(0.1)
            self.mc.stop()
        else:
            # Cube is centered: if it's still far away, move forward.
            if cube_center_y < self.cam_height // 2 + 23:
                print(f"{self.current_target} Cube centered; moving forward.")
                self.mc.move_forward()
            else:
                print(f"{self.current_target} Cube reached; stopping and switching to placement search.")
                self.mc.stop()
                self.state = "SEARCH_PLACEMENT"
                self.cube_info = None
                # Reset the lost count since the cube was reached
                self.cube_lost_count = 0

    def search_placement(self):
        """
        Rotate the robot (turning continuously) for a full rotation (or until the placement area is detected)
        while processing camera images. If the placement area is found, transition to APPROACH_PLACEMENT.
        """
        print(f"Searching for {self.current_target} placement area...")
        start_time = self.robot.getTime()
        rotation_duration = 20  # approximate duration for a full rotation (tunable)
        while self.robot.step(self.timestep) != -1:
            self.mc.turn_left()
            depth_image = self.process_range_finder()
            self.process_camera(depth_image)
            
            if self.placement_info is not None:
                print(f"{self.current_target} Placement area detected.")
                self.mc.stop()
                self.state = "APPROACH_PLACEMENT"
                return
            if self.robot.getTime() - start_time > rotation_duration:
                print("Full rotation completed. Placement area not detected; reverting to NAVIGATE.")
                self.mc.stop()
                self.state = "NAVIGATE"
                return

    def search_cube(self):
        """
        Rotate the robot (turning continuously) for a full rotation (or until the target cube is detected)
        while processing camera images. If the cube is found, transition to APPROACH_CUBE.
        """
        print(f"Searching for {self.current_target} cube...")
        start_time = self.robot.getTime()
        rotation_duration = 20  # approximate duration for a full rotation (tunable)
        while self.robot.step(self.timestep) != -1:
            self.mc.turn_left()  # or turn_right() as preferred
            depth_image = self.process_range_finder()
            self.process_camera(depth_image)
            
            if self.cube_info is not None:
                print(f"{self.current_target} Cube detected.")
                self.mc.stop()
                self.state = "APPROACH_CUBE"
                return
            
            if self.robot.getTime() - start_time > rotation_duration:
                print("Full rotation completed. Cube not detected; reverting to NAVIGATE.")
                self.mc.stop()
                self.state = "NAVIGATE"
                return

    def approach_placement(self):
        """
        Once the placement area is detected, align the robot, approach it until a target distance is reached,
        then stop and reverse a bit. After placing the box, update the target color and state.
        """
        x, y, w, h, avg_depth = self.placement_info
        placement_center_x = x + w/2
        placement_center_y = y + h/2
        image_center_x = self.cam_width / 2
        error = placement_center_x - image_center_x
        threshold_pixels = 15
        target_distance = 0.4  # Tunable target distance from the placement area
        
        # Align horizontally
        if abs(error) > threshold_pixels:
            if error > 0:
                print(f"{self.current_target} Placement area detected to the right; turning right to center it.")
                self.mc.turn_right()
            else:
                print(f"{self.current_target} Placement area detected to the left; turning left to center it.")
                self.mc.turn_left()
            self.wait(0.5)
            self.mc.stop()
        else:
            # Once aligned, check distance (using avg_depth as a proxy)
            if avg_depth > target_distance:
                print(f"Approaching {self.current_target} placement area.")
                self.mc.move_forward()
            else:
                print(f"Reached {self.current_target} placement area, stopping.")
                self.mc.stop()
                print("Placing box and reversing.")
                self.mc.move_backward()
                self.wait(3)
                self.mc.stop()
                # Update target color or finish mission
                if self.current_target == "Blue":
                    print("Blue box placed. Mission completed.")
                    self.state = "DONE"
                else:
                    # Move to the next color in the sequence and search for its cube
                    self.current_index += 1
                    self.current_target = self.order[self.current_index]
                    print(f"Box placed. Next target is {self.current_target}.")
                    # Reset lost count for next cube
                    self.cube_lost_count = 0
                    self.state = "SEARCH_CUBE"

    def run(self):
        while self.robot.step(self.timestep) != -1:
            depth_image = self.process_range_finder()
            
            if self.state == "NAVIGATE":
                self.obstacle_avoidance(depth_image)
                self.process_camera(depth_image)
                if self.cube_info is not None:
                    print(f"{self.current_target} Cube detected; switching to APPROACH_CUBE state.")
                    self.state = "APPROACH_CUBE"
                    
            elif self.state == "SEARCH_CUBE":
                self.search_cube()
                    
            elif self.state == "APPROACH_CUBE":
                self.process_camera(depth_image)
                if self.cube_info is not None:
                    self.approach_cube(self.cube_info)
                    # Reset lost count if the cube is being tracked
                    self.cube_lost_count = 0
                else:
                    self.cube_lost_count += 1
                    print(f"Cube lost ({self.cube_lost_count} time(s));")
                    if self.cube_lost_count >= 3:
                        print("Cube lost 3 times; reverting to NAVIGATE state.")
                        self.state = "NAVIGATE"
                        self.cube_lost_count = 0
                    else:
                        self.state = "SEARCH_CUBE"
                    
            elif self.state == "SEARCH_PLACEMENT":
                self.search_placement()
                
            elif self.state == "APPROACH_PLACEMENT":
                self.process_camera(depth_image)
                if self.placement_info is not None:
                    self.approach_placement()
                else:
                    print("Placement area lost; reverting to SEARCH_PLACEMENT state.")
                    self.state = "SEARCH_PLACEMENT"
                    
            elif self.state == "DONE":
                print("Mission completed.")
                break

if __name__ == "__main__":
    navigator = RobotNavigator()
    navigator.run()
