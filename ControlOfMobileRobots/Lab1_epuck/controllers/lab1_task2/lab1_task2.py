"""lab1_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world in msec.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

# Lab1 Task1 declaration of variables
vec_count = 0
time_check = 0
a = [[1,1,5],[2,2,3],[3,2,4],[2,3,4],[7,1,1],[1,7,1],[-1,-1,5],[-2,-2,5],[3,-3,3],[-3,3,3]]
    
current_vector = a[0]
time_check += current_vector[2]

# Lab1 Task2 variables
motion_init_l = 0
motion_init_r = 0
distance_traveled = 0


# Main loop:
# perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    # val = ds.getValue()

    # print simulation time in sec (timesteps are in msec)
    print("Time: " +str(robot.getTime()))
    
    # Process sensor data here.   
    print("Left position sensor: " +str(leftposition_sensor.getValue()))
    print("Right position sensor: " +str(rightposition_sensor.getValue()))

    if(robot.getTime() < time_check):
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        leftMotor.setVelocity(current_vector[0])
        rightMotor.setVelocity(current_vector[1])
    
    else:
        print("Vector Change")
        
        delta_L = leftposition_sensor.getValue() - motion_init_l
        delta_R = rightposition_sensor.getValue() - motion_init_r
        distance_traveled += .8 * (delta_L + delta_R)
        motion_init_l = leftposition_sensor.getValue()
        motion_init_r = rightposition_sensor.getValue()
        
        print("Distance Travelled: " + str(distance_traveled))
        
        vec_count += 1
        if (vec_count < len(a)):
            current_vector=a[vec_count]
            time_check += current_vector[2]
            leftMotor.setVelocity(current_vector[0])
            rightMotor.setVelocity(current_vector[1])
        else:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            print("Time of Stop: " +str(robot.getTime()))
            print("Total inches traversed: " + str(distance_traveled))
            break
    

# Enter here exit cleanup code.
