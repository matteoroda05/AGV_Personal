"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# getthe motor devices
lMotor = robot.getDevice('left wheel motor')
rMotor = robot.getDevice('right wheel motor')
# set the target position of the motors
lMotor.setPosition(float('inf'))
rMotor.setPosition(float('inf'))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # set up the motor speeds at 50% of the MAX_SPEED
    # after first 5s set rMotor speed to 1% => robot starts turning right with nonnull internal wheel speed
    lMotor.setVelocity(0.5 * MAX_SPEED)
    if robot.getTime() < 5.0:
        rMotor.setVelocity(0.5 * MAX_SPEED)
    else:
        rMotor.setVelocity(0.01 * MAX_SPEED)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
