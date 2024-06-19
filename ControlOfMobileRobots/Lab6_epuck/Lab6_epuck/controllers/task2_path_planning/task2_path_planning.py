import numpy as np
import math
from motionLibrary import *
import pickle

class mapObj:
    def __init__(self, map_array):
        self.innerMap = map_array
    def giveMap(self):
        return self.innerMap

class Cell:
    def __init__(self, e, n, w, s):
        self.eastWall = e
        self.northWall = n
        self.westWall = w
        self.southWall = s
        
    def printCell(self):
        print("East: ", self.eastWall)
        print("North: ", self.northWall)
        print("West: ", self.westWall)
        print("South: ",self.southWall)

def updateOrientation(imuData, prev_orient):
    if(imuData >= 3.13 or imuData <= -3.13):
        return 2
    elif(imuData <= .01 and imuData >= -0.01):
        return 0
    elif(imuData <= 1.58 and imuData >= 1.53):
        return 1
    elif(imuData >= -1.58 and imuData <= -1.53):
        return 3
    else:
        return prev_orient
        
def updateCord(orientation, cell_coord):
    next_coord = [0,0]
    if(orientation == 0):
        next_coord[0] = cell_coord[0] + 1
        next_coord[1] = 0 + cell_coord[1]
    elif(orientation == 1):
        next_coord[0] = 0 + cell_coord[0]
        next_coord[1] = cell_coord[1] - 1
    elif(orientation == 2):
        next_coord[0] = cell_coord[0] - 1
        next_coord[1] = 0 + cell_coord[1]
    elif(orientation == 3):
        next_coord[0] = 0 + cell_coord[0]
        next_coord[1] = cell_coord[1] + 1
    return next_coord
    
def mapPrint():
    for i in my_map:
        for k in i:
            if (not (k == "NULL")):
                print("X", end=" ")
            else:
                print(".", end=" ")
        print("|")

def waveFront(goal, ref_map, wave_map, i_val):
    walls =[ ref_map[goal[1]][goal[0]].eastWall,
             ref_map[goal[1]][goal[0]].northWall,
             ref_map[goal[1]][goal[0]].westWall,
             ref_map[goal[1]][goal[0]].southWall]
    neighbors = []
    next_set = []
    
    #get neighbors
    for i in range(0,4):
        if(not walls[i]):
            neighbors.append(updateCord(i, goal))
    #update neighbors
    for k in neighbors:
        if(wave_map[k[1]][k[0]] == 0 or wave_map[k[1]][k[0]] > i_val + 1):
            wave_map[k[1]][k[0]] = i_val + 1
            next_set.append(k)
    #Propegate it
    for x in next_set:
        waveFront(x, ref_map, wave_map, i_val + 1)
        
def getPath(start_pos, ref_map, wave_map, path_to_take):
    walls =[ ref_map[start_pos[1]][start_pos[0]].eastWall,
             ref_map[start_pos[1]][start_pos[0]].northWall,
             ref_map[start_pos[1]][start_pos[0]].westWall,
             ref_map[start_pos[1]][start_pos[0]].southWall]
    my_score = wave_map[start_pos[1]][start_pos[0]]
    neighbors = []
    
    if(my_score == 1):
        return
    #get neighbors
    for i in range(0,4):
        if(not walls[i]):
            neighbors.append(updateCord(i, start_pos))
            
    for n in neighbors:
        if(wave_map[n[1]][n[0]] < my_score):
            path_to_take.append(n)
            getPath(n, ref_map, wave_map, path_to_take)
            break
#######################################################
# Parameters
#######################################################


# Robot Deminsions in inch
wheel_radius = 1.6/2
axel_length = 2.28

my_cell_cord = [3,3]
my_goal_cord = [0,2]
my_orient = 1

map_load = open('curr_map_file', "rb")
check_map = pickle.load(map_load)
map_load.close()

test_map = check_map.innerMap
       
front_map = [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 0]]

my_path = []
steps_taken = 0

robot = HamBot()

robot.set_pose(0,0)

waveFront(my_goal_cord,test_map, front_map, 1)
getPath(my_cell_cord, test_map, front_map, my_path)
print("My path: ", my_path)
# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.robot.step(robot.timestep) != -1:
    
    fd = min(robot.lidar.getRangeImage()[0:5] + robot.lidar.getRangeImage()[354:359]) * 39.37
    lf = min(robot.lidar.getRangeImage()[260:280]) * 39.37
    rt = min(robot.lidar.getRangeImage()[80:100]) * 39.37
    bk = min(robot.lidar.getRangeImage()[170:190]) * 39.37
   
    cell_infront = updateCord(my_orient, my_cell_cord)
    if (not(cell_infront == my_path[steps_taken])):
        if(lf >4.5):
            robot.rotate(-90)
            my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
            continue
        else:
            robot.rotate(90)
            my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
            continue

    robot.driveD(10)
    steps_taken += 1
    my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
    print("Orientation pre-cord: ", my_orient)
    my_cell_cord = updateCord(my_orient, my_cell_cord)
    print("My current predicted cell is at: ", my_cell_cord)
    if(my_cell_cord == my_goal_cord):
        print("Arrive at goal")
        break
    
    #robot.update_pose()
    #robot.print_pose()

    # Robot will dirve in a straigh line for 10 inches
    # robot.driveD(10)

    # Robot will rotate in place 90 degrees clock wise
    # robot.rotate(90)

    # Robot will rotate in place 90 degrees counter clock wise
    # robot.rotate(-90)
    
    
    

# Enter here exit cleanup code.
