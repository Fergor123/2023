# import numpy as it may be used in future labs
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

def checkMapErr(cell_cord, map):
    virtual_map = [["NULL", "NULL", "NULL", "NULL"],
                   ["NULL", "NULL", "NULL", "NULL"],
                   ["NULL", "NULL", "NULL", "NULL"],
                   ["NULL", "NULL", "NULL", "NULL"]]
    if(cell_cord[0] < 0):
        for i in range(0,4):
            for k in range(1, 4):
                virtual_map[i][k] = map[i][k - 1]
        cell_cord[0] += 1
        return virtual_map
    elif(cell_cord[1] < 0):
        for i in range(1, 4):
            virtual_map[i] = map[i - 1]
        cell_cord[1] += 1
        return virtual_map
    else:
        return map
        
#######################################################
# Parameters
#######################################################


# Robot Deminsions in inch
wheel_radius = 1.6/2
axel_length = 2.28

my_cell_cord = [0,0]
my_orient = 1
mapped_cells = 0 

my_map = [["NULL", "NULL", "NULL", "NULL"],
          ["NULL", "NULL", "NULL", "NULL"],
          ["NULL", "NULL", "NULL", "NULL"],
          ["NULL", "NULL", "NULL", "NULL"]]
       
virtual_map = [["NULL", "NULL", "NULL", "NULL"],
               ["NULL", "NULL", "NULL", "NULL"],
               ["NULL", "NULL", "NULL", "NULL"],
               ["NULL", "NULL", "NULL", "NULL"]]

robot = HamBot()

robot.set_pose(0,0)

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.robot.step(robot.timestep) != -1:
    
    fd = min(robot.lidar.getRangeImage()[0:5] + robot.lidar.getRangeImage()[354:359]) * 39.37
    lf = min(robot.lidar.getRangeImage()[260:280]) * 39.37
    rt = min(robot.lidar.getRangeImage()[80:100]) * 39.37
    bk = min(robot.lidar.getRangeImage()[170:190]) * 39.37
    print(rt < 4.5, fd < 4.5, lf < 4.5, bk < 4.5)
    print("My Orientation: ", my_orient)
    #my_map[my_cell_cord[1]][my_cell_cord[0]] = True
    mapPrint()
    
    #Cell mapper
    if(my_map[my_cell_cord[1]][my_cell_cord[0]] == "NULL"):
        sampler_array = [rt < 4.5, fd < 4.5, lf < 4.5, bk < 4.5]
        corr_array = [0, 0, 0, 0]
        for i in range(0,4):
            corr_array[(my_orient - 1 + i) % 4] = sampler_array[i]
        my_cell = Cell(corr_array[0], corr_array[1], corr_array[2], corr_array[3])
        my_cell.printCell()
        my_map[my_cell_cord[1]][my_cell_cord[0]] = my_cell
        mapped_cells += 1
    
    if fd < 4.5:
        if(lf >4.5):
            robot.rotate(-90)
            my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
            continue
        else:
            robot.rotate(90)
            my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
            continue

    robot.driveD(10)
    my_orient = updateOrientation(robot.imu.getRollPitchYaw()[2], my_orient)
    print("Orientation pre-cord: ", my_orient)
    my_cell_cord = updateCord(my_orient, my_cell_cord)
    print("My current predicted cell is at: ", my_cell_cord)
    my_map = checkMapErr(my_cell_cord, my_map)
    print("My current cell post correction: ", my_cell_cord)
    print("Mapped Cells: ", mapped_cells)
    
    if(mapped_cells == 16):
        print(my_map)
        for i in my_map:
            for k in i:
                
                k.printCell()
        export_map = mapObj(my_map)
        map_file = open('curr_map_file', "ab")
        pickle.dump(export_map, map_file)
        map_file.close()
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
