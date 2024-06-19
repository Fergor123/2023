# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np

#######################################################
# Creates Robot
#######################################################
robot = Robot()


#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()


print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

def setMotion(left_vel, right_vel):
    leftMotor.setVelocity(left_vel)
    rightMotor.setVelocity(right_vel)
def limiter(set_speed):
    if(set_speed > 4.71):
        return 4.71
    elif(set_speed < 0.3925):
        return 0.3925
    else:
        return set_speed
    
def correctionPID(anchor, targetDist, readingDist, k_p, max_v):
    error = (readingDist - targetDist)
    correction = error * k_p
    corrected_v1 = max_v - correction
    corrected_v2 = limiter(corrected_v1)
    print(corrected_v1)
    if(anchor == 0):
        setMotion(corrected_v2, max_v)
    elif(anchor == 1):
        setMotion(max_v, corrected_v2)
    else: #Default follow left
        setMotion(corrected_v2, max_v)
    

    
##############
# Lab vars; Configuration Space
##############
wall_target = 0.075 #+ .0125
front_target = .115
wall_anchor = 0 # 0 = left; 1 = Right;

max_vel = 3.14

k_ps = 5
orientation = 0 # 0 = East; 1 = North; 2 = West; 3 = South
rotation_flag = False
right_bias = False

# Main loop:
# perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:
    # Read the sensors:
    # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
    full_range_image = lidar.getRangeImage()
    # print size of Range Image
    print('#################################################################')
    print("Lidar's Full Range Image Size: ", len(full_range_image))
    # Compare Distance Sensors to Lidar Ranges
    front_dist = frontDistanceSensor.getValue()
    right_dist = rightDistanceSensor.getValue()
    rear_dist = rearDistanceSensor.getValue()
    left_dist = leftDistanceSensor.getValue()

    print("Distance Sensor vs Lidar")
    print("\tFront:\t", front_dist, "\t|", full_range_image[0])
    print("\tRight:\t", right_dist, "\t|", full_range_image[90])
    print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
    print("\tLeft:\t", left_dist, "\t|", full_range_image[270])

   # camera object recognition
    obj_in_view = len(camera.getRecognitionObjects())
    print("Objects in View: ", obj_in_view)

    pos_image =  99999;
    pos_view = 99999;
    if (obj_in_view > 0):
        pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
        pos_view = camera.getRecognitionObjects()[0].getPosition()[0] 
        
        print("\tPosition in Image:\t", pos_image) # in pixels relative to image
        print("\tPosition in View:\t", pos_view) # in meters relative to camera
         
        
    # Enter here functions to send actuator commands, like:
    idle_flag = obj_in_view > 0 and pos_view < .15
    search_centered_flag = pos_image < 42 and pos_image > 38
    not_search_flag = obj_in_view > 0 and search_centered_flag
    
    if(not idle_flag):
        if(not_search_flag):
            setMotion(3.14, 3.14)
            print("Approaching")
        else:
            setMotion(1.57, -1.57)
            print("Searching")
    else:
        setMotion(0, 0)
        print("Idle")
# Enter here exit cleanup code.
