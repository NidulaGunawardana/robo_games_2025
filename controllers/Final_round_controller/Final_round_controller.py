"""Final_round_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from motor_controller import MotorController
from depth_vision import CameraControl

import cv2

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
# ds = robot.getDevice('dsname')
# ds.enable(timestep)

mc = MotorController(robot)
mc.init_motor_controller()

cc = CameraControl(robot)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # cc.display_image()
    mc.move_forward()
    mc.turn_right()
    mc.turn_left()
    mc.move_forward()
    mc.turn_right()
    mc.move_forward()
    mc.turn_right()
    mc.move_forward()
    mc.turn_left()
    
    # while timestep <1000:
        # timestep += 1
    # mc.turn_left()
    # mc.move_forward()
    # mc.turn_right()
    # mc.move_forward()
    # mc.turn_right()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
