import time


class MotorController:
    ROBOT = None
    left_motor = None
    right_motor = None
    right_position = None
    left_position = None
    
    def __init__(self,robot):
        self.ROBOT = robot
        self.init_motor_controller()
        
    def init_motor_controller(self):
        self.left_motor = self.ROBOT.getDevice('left wheel motor')
        self.right_motor = self.ROBOT.getDevice('right wheel motor')
        self.right_position = self.ROBOT.getDevice('right wheel sensor')
        self.left_position = self.ROBOT.getDevice('left wheel sensor')
        self.right_position.enable(1)
        self.left_position.enable(1)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
    def get_left_position(self):
        return self.left_position.getValue()
    
    def get_right_position(self):
        return self.right_position.getValue()
        
    def move_forward(self):
        self.left_motor.setVelocity(-10.0)
        self.right_motor.setVelocity(-10.0)
        current = self.get_left_position()
        while self.ROBOT.step(32) != -1:
            if abs(current - self.get_left_position()) > 23.75:
                self.stop()
                break
    
    def move_backward(self):
        self.left_motor.setVelocity(10.0)
        self.right_motor.setVelocity(10.0)
        current = self.get_left_position()
        while self.ROBOT.step(32) != -1:
            if abs(current - self.get_left_position()) < 23.75:
                self.stop()
                break
        
    def turn_left(self):
        self.left_motor.setVelocity(3.0)
        self.right_motor.setVelocity(-3.0)
        current = self.get_left_position()
        while self.ROBOT.step(32) != -1:
            if abs(current - self.get_left_position()) > 13.6:
                self.stop()
                break
        
    def turn_right(self):
        self.left_motor.setVelocity(-3.0)
        self.right_motor.setVelocity(3.0)
        current = self.get_left_position()
        while self.ROBOT.step(32) != -1:
            if abs(current - self.get_left_position()) > 13.6:
                self.stop()
                break

    def stop(self):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        