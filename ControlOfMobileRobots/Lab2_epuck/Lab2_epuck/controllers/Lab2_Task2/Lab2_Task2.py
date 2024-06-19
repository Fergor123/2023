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
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)


waypoints = [[15,-15],[15,15],[-10,15],
             [-15,10],[-15,5],[0,5],
             [5,0],[0,-5],[-10,-5],
             [-15,-10],[-15,-15]]
start_point = [-5,-15]
points_pass = 0
time_check = 0
end_point = waypoints[0]
linear_vel_max = 5.024
rotation = 0

t = [5, 10, 20]
t_frame = 0
t_init = 0


# Main loop:
# perform simulation steps until Webots is stopping the controller
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
while robot.step(timestep) != -1:
    
    if(robot.getTime() < time_check):
        if(t_frame < len(t)):
            if(t[t_frame] < robot.getTime()):
                my_roto = imu.getRollPitchYaw()[2]
                if(imu.getRollPitchYaw()[2] >= 3.13 or imu.getRollPitchYaw()[2] <= -3.13):
                    my_x = start_point[0] - 6 * (robot.getTime() - t_init)
                    my_y = start_point[1]
                elif(imu.getRollPitchYaw()[2] <= .01 and imu.getRollPitchYaw()[2] >= -0.01):
                    my_x = start_point[0] + 6 * (robot.getTime() - t_init)
                    my_y = start_point[1]
                elif(imu.getRollPitchYaw()[2] <= 1.57 and imu.getRollPitchYaw()[2] >= 1.53):
                    my_y = start_point[1] + 6 * (robot.getTime() - t_init)
                    my_x = start_point[0]
                else:
                    my_y = start_point[1] - 6 * (robot.getTime() - t_init)
                    my_x = start_point[0]
                print("Estimated position: " + str(my_x) + "," + str(my_y))
                print("My angle: " + str(360 * (my_roto/3.14)) + " degrees")
                t_frame += 1
        continue
    else:
        distance_x = end_point[0] - start_point[0]
        distance_y = end_point[1] - start_point[1]
        if(distance_x == 0 or distance_y == 0):
            #print("Linear Motion")
            if(distance_x != 0):
                if(distance_x < 0):
                    
                    if(imu.getRollPitchYaw()[2] >= 3.13 or imu.getRollPitchYaw()[2] <= -3.13):
                        rotation = 0
                        time_check += abs(distance_x) / linear_vel_max
                        print("Distance of: " + str(abs(distance_x)) + " inches at ~5 inches/per sec")
                        print("Estimated time: " + str(abs(distance_x) / linear_vel_max))
                        leftMotor.setVelocity(6.28)
                        rightMotor.setVelocity(6.28)
                    else:
                        #print("Rotating West")
                        rotation = 1
                        leftMotor.setVelocity(-1.57)
                        rightMotor.setVelocity(1.57)
                        time_check = robot.getTime()
                if(distance_x > 0):
                    
                    if(imu.getRollPitchYaw()[2] <= .01 and imu.getRollPitchYaw()[2] >= -0.01):
                        rotation = 0
                        time_check += abs(distance_x) / linear_vel_max
                        print("Distance of: " + str(abs(distance_x)) + " inches at ~5 inches/per sec")
                        print("Estimated seconds: " + str(abs(distance_x) / linear_vel_max))
                        leftMotor.setVelocity(6.28)
                        rightMotor.setVelocity(6.28)
                    else:
                        #print("Rotating East")
                        rotation = 1
                        leftMotor.setVelocity(-1.57)
                        rightMotor.setVelocity(1.57)
                        time_check = robot.getTime()
                    
            elif(distance_y != 0):
                if(distance_y > 0):
                    if(imu.getRollPitchYaw()[2] <= 1.57 and imu.getRollPitchYaw()[2] >= 1.53):
                        rotation = 0
                        time_check += distance_y / linear_vel_max
                        print("Distance of: " + str(abs(distance_y)) + " inches at ~5 inches/per sec")
                        print("Estimated seconds: " + str(abs(distance_y) / linear_vel_max))
                        leftMotor.setVelocity(6.28)
                        rightMotor.setVelocity(6.28)
                    else:
                        #print("Rotating North")
                        rotation = 1
                        leftMotor.setVelocity(-1.57)
                        rightMotor.setVelocity(1.57)
                        time_check = robot.getTime()
                if(distance_y < 0):
                    if(imu.getRollPitchYaw()[2] >= -1.57 and imu.getRollPitchYaw()[2] <= -1.53):
                        rotation = 0
                        time_check += abs(distance_y) / linear_vel_max
                        print("Distance of: " + str(abs(distance_y)) + " inches at ~5 inches/per sec")
                        print("Estimated seconds: " + str(abs(distance_y) / linear_vel_max))
                        leftMotor.setVelocity(6.28)
                        rightMotor.setVelocity(6.28)
                    else:
                        #print("Rotating South")
                        rotation = 1
                        leftMotor.setVelocity(-1.57)
                        rightMotor.setVelocity(1.57)
                        time_check = robot.getTime()
        else:
            #print("Circular Motion")
            radius = abs(distance_x)
            circ_dist = 1.5708 * radius
            angular_v = 2.5 / (radius + 1.14)
            v_inner = angular_v * (radius - 1.14)
            time_check += (5.024 + v_inner) / 2
            print("Distance of: " + str(circ_dist))
            print("Estimated Time: " + str((5.024 + v_inner) / 2))
            if(imu.getRollPitchYaw()[2] >= 3.13 or imu.getRollPitchYaw()[2] <= -3.13):
                leftMotor.setVelocity(v_inner/.8)
                rightMotor.setVelocity(3.14)
                print("Velocities Left: " + str(v_inner) + "Right: 2.516")
            else:
                leftMotor.setVelocity(3.14)
                rightMotor.setVelocity(v_inner/.8)
                print("Velocities Left: 2.512 Right: " + str(v_inner))
            #break    
        if(rotation == 0):
            points_pass += 1
            if(points_pass == len(waypoints) + 1):
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                print("Path Complete")
                break
            else:
                #print("Next Waypoint")
                start_point = end_point
                if(points_pass < len(waypoints)):
                    end_point = waypoints[points_pass]
                    t_init = robot.getTime()

# Enter here exit cleanup code.
