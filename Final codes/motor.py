import time
# import KobukiDriver as kd

class MotorController:
    
    def __init__(self, keyboard_control=False):
        self.keyboard_control = keyboard_control
        # self.my_kobuki = kd.Kobuki()
        print("Initialized MotorController and played startup sound.")
    
    def get_encoder(self):
        """Get encoder data from the robot."""
        print("Called get_encoder function.")
        return 0, 0
        
    def move_forward(self):
        print("Called move_forward function.")

    def move_backward(self):
        print("Called move_backward function.")

    def turn_left(self):
        print("Called turn_left function.")

    def turn_right(self):
        print("Called turn_right function.")

    def stop(self):
        print("Called stop function.")
        
    def keyboard_move(self):
        # Play start up sound
        if self.keyboard_control:
            key = input("Enter command: ")
            if key == "w":
                # Move forward
                self.move_forward()
            elif key == "s":
                # Move backward
                self.move_backward()
            elif key == "a":
                # Turn left
                self.turn_left()
            elif key == "d":
                # Turn right
                self.turn_right()
            elif key == "x":
                # Stop
                self.stop()
            elif key == "q":
                # Quit
                self.keyboard_control = False

            # Print sensor data
            print(self.get_encoder())
