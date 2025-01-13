"""epuck_controller_robogames_2025 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    max_speed = 6.28
    timestep = int(robot.getBasicTimeStep())
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    #get proximity sensors
    ps = []
    for i in range(8):
        ps_name = 'ps' + str(i)
        ps.append(robot.getDevice(ps_name))
        ps[i].enable(timestep)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        
        # left_motor.setVelocity(3.0)
        # right_motor.setVelocity(3.0)
        
        for i in range(8):
            print('ps' + str(i) + ': ' + str(ps[i].getValue()))

        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass



if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
    

