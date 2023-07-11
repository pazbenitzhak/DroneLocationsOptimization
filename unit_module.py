import surface as surface
import numpy as np
import matplotlib.pyplot as plt
import PIL
import random
import time_module
from scipy.signal import convolve2d
from scipy.ndimage import convolve


avg_vel = 1.388 # 5km/h
vel_dev = 0.5
diff_threshold = 0.5

class unit_module:
    commander_direction = 0
    soldiers = []
    block = []
    comm_x = 0
    comm_y = 0

    def __init__(self, solds,surf):
        self.soldiers = solds
        self.block = surface.surface.getBlocks(surf) #white_block matrix - true/false
        self.white_block_indexes = surface.surface.getWhiteIndexes(surf)
        self.comm_x, self.comm_y = self.placeCommander(self.white_block_indexes)

    def setCommanderLoc(self,time):
        #in second implementation, the block is a 5000X5000 matrix with True/False values
        #according to 'whiteness'
        block = self.getBlock()
        avg_velocity = avg_vel
        velocity_deviation =  vel_dev #chosen so the velocity ranges are between 0-9.5
        velocity_val = random.gauss(avg_velocity,velocity_deviation)
        dist = velocity_val*time_module.time_module.getTimeInterval(time)
        comm_location = self.getCommanderLoc()
        curr_comm_direc = self.commander_direction
        possible_locations = findPossibleLocations(comm_location[0],comm_location[1],block,dist,curr_comm_direc)
        x_poss, y_poss = possible_locations[random.randint(0,len(possible_locations)-1)] #for example, only to calculate quarter
        self.commander_direction = calculateQuarter(x_poss, y_poss,comm_location[0],comm_location[1]) #adjust general quarter function to commander
        locs_len = len(possible_locations)
        index = random.randint(0,locs_len-1)
        self.comm_x, self.comm_y = possible_locations[index]
        return possible_locations[index]


    def getSoldiers(self):
        return self.soldiers
    
    def getSoldiersNum(self):
        return len(self.getSoldiers())

    def getBlock(self):
        return self.block
    
    def getWhiteIndexes(self):
        return self.white_block_indexes

    def placeUnit(self,surf):
        blocks = surface.surface.getBlocks(surf) # clusters of valid areas for units to move on
        blocks_len = len(blocks)
        index = random.randint(0,blocks_len-1)
        return blocks[index] # the chosen block

    def placeCommander(self,block):
        block_len = len(block)
        index = random.randint(0,block_len-1)
        self.commander_direction = random.randint(1,4)
        return block[index] #tuple: x, y
        
    def getCommanderLoc(self):
        return self.comm_x, self.comm_y

def findPossibleLocations(x_0,y_0,block,dist,quarter):
    possible_locations = []
    possible_locations_q_1 = []
    possible_locations_q_2 = []
    possible_locations_q_3 = []
    possible_locations_q_4 = []
    dist+=0.5 # increasing the range a bit
    for x in range(int(np.floor(x_0-dist)),int(np.ceil(x_0+dist))+1):
        for y in range(int(np.floor(y_0-dist)),int(np.ceil(y_0+dist))+1):
            quarter_1 = x>=x_0 and y>y_0
            quarter_2 = x<x_0 and y>y_0
            quarter_3 = x<x_0 and y<=y_0
            quarter_4 = x>=x_0 and y<=y_0
            if not (0<=x<5000 and 0<=y<5000):
                continue
            if quarter_2 or quarter_4: #2nd and 4th quarters
                if np.floor((dist-1)**2) <= (x-x_0)**2 + (y-y_0)**2 <= np.ceil((dist+1)**2):
                #we took an inner ring and outer ring (dist+-1) to get approximate points
                #that would represent the dist passed by the commander
                    #NPV: if (x,y) in block
                    if block[x,y]==1: #True - it's in a white valid pixel
                        if quarter_2:
                            possible_locations_q_2.append((x,y))
                        else: #must be quarter 4
                            possible_locations_q_4.append((x,y))
                        continue
                
                if np.floor((dist-1)**2) <= ((x+1)-x_0)**2 + ((y+1)-y_0)**2 <= np.ceil((dist+1)**2):
                #we took an inner ring and outer ring (dist+-1) to get approximate points
                #that would represent the dist passed by the commander
                    #NPV: if (x,y) in block
                    if block[x,y]==1: #True - it's in a white valid pixel
                        if quarter_2:
                            possible_locations_q_2.append((x,y))
                        else: #must be quarter 4
                            possible_locations_q_4.append((x,y))
                        continue

            else: #1st or 3rd quarter
                if np.floor((dist-1)**2) <= ((x+1)-x_0)**2 + (y-y_0)**2 <= np.ceil((dist+1)**2):
                #we took an inner ring and outer ring (dist+-1) to get approximate points
                #that would represent the dist passed by the commander
                    #NPV: if (x,y) in block
                    if block[x,y]==1: #True - it's in a white valid pixel
                        if quarter_1:
                            possible_locations_q_1.append((x,y))
                        else: #must be quarter 3
                            possible_locations_q_3.append((x,y))
                        continue

                if np.floor((dist-1)**2) <= (x-x_0)**2 + ((y+1)-y_0)**2 <= np.ceil((dist+1)**2):
                #we took an inner ring and outer ring (dist+-1) to get approximate points
                #that would represent the dist passed by the commander
                    #NPV: if (x,y) in block
                    if block[x,y]==1: #True - it's in a white valid pixel
                        if quarter_1:
                            possible_locations_q_1.append((x,y))
                        else: #must be quarter 3
                            possible_locations_q_3.append((x,y))
                        continue
    #need to pick a number: 1-8 for opposite quarter, 9-24 & 25-40 for complementary quarters and 41-100 for the sqme quarter
    while True:
        num = random.randint(1,100)
        #if (41<=num<=100): quarter stays the same
        if (25<=num<=40):
            quarter = (quarter%4)+1
        elif (9<=num<=24):
            quarter = quarter-1
            if quarter==0: # 1-> 4
                quarter += 4
        elif (1<=num<=8):
            quarter = (quarter+2) % 4
            if quarter==0: # 2-> 4
                quarter += 4
        match quarter:
            case 1:
                if (len(possible_locations_q_1)>0):
                    possible_locations = possible_locations_q_1
                    break
            case 2:
                if (len(possible_locations_q_2)>0):
                    possible_locations = possible_locations_q_2
                    break
            case 3:
                if (len(possible_locations_q_3)>0):
                    possible_locations = possible_locations_q_3
                    break
            case 4: 
                if (len(possible_locations_q_4)>0):
                    possible_locations = possible_locations_q_4
                    break
    return possible_locations


def calculateQuarter(x_comm, y_comm, x_sold, y_sold):
    if x_comm==x_sold: #avoid division by zero
        angle = np.pi/2
        if y_comm==y_sold:
            return 4
    else:
        slope = (y_comm-y_sold)/(x_comm-x_sold)
        angle = np.arctan(slope)
    if (0<=angle<=(np.pi/2)): #quarters 1, 3
        if x_comm>=x_sold:
            return 1
        return 3
    else: #quarters 2,4
        if x_comm>=x_sold:
            return 4
        return 2


