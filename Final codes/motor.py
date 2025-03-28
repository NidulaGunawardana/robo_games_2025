import time
import KobukiDriver as kd

class MotorController:
    
    def __init__(self,keyboard_control=False):
        self.keyboard_control = keyboard_control
        self.my_kobuki = kd.Kobuki()
        self.my_kobuki.play_on_sound()
    
    def get_encoder(self):
        """Get encoder data from the robot."""
        encoder_data = self.my_kobuki.encoder_data()
        print(encoder_data)
        return encoder_data[0], encoder_data[1]
        
    def move_forward(self):
        self.my_kobuki.move(200, 200, 0)

    
    def move_backward(self):
        self.my_kobuki.move(-200, -200, 0)

        
    def turn_left(self):
        self.my_kobuki.move(100, -100, 1)

        
    def turn_right(self):
        self.my_kobuki.move(-100, 100, 1)


    def stop(self):
        self.my_kobuki.move(0, 0, 0)
        
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