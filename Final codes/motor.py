import time
import KobukiDriver as kd
import tty, sys, termios

class MotorController:
    
    def __init__(self,keyboard_control=False):
        self.keyboard_control = keyboard_control
        self.my_kobuki = kd.Kobuki()
        self.my_kobuki.play_on_sound()
        self.filedescriptors = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
        self.key = 0
    
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
        
    def keyboard_move(self,turntime):
        # Play start up sound
        if self.keyboard_control:
            key=sys.stdin.read(1)[0]
            print("You pressed", key)
            if key == "w":
                # Move forward
                self.move_forward()
                time.sleep(turntime)
                self.stop()
            elif key == "s":
                # Move backward
                self.move_backward()
                time.sleep(turntime)
                self.stop()
            elif key == "a":
                # Turn left
                self.turn_left()
                time.sleep(turntime)
                self.stop()
            elif key == "d":
                # Turn right
                self.turn_right()
                time.sleep(turntime)
                self.stop()
            elif key == "x":
                # Stop
                self.stop()
            elif key == "q":
                # Quit
                self.keyboard_control = False
    
            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.filedescriptors)