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
#TODO make time object accessible to all files
diff_threshold = 0.5

class unit_module:
    commander_direction = 0
    soldiers = []
    block = []
    comm_x = 0
    comm_y = 0

    def __init__(self, solds,surf):
        self.soldiers = solds
        #NPV: self.block = self.placeUnit(surf)
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
        print("commander quarter: "+ str(self.commander_direction))
        locs_len = len(possible_locations)
        #print("locs_len: " + str(locs_len))
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
        # TODO: for phase II - need to remember what blocks are 'taken'
        return blocks[index] # the chosen block

    def placeCommander(self,block):
        block_len = len(block)
        index = random.randint(0,block_len-1)
        self.commander_direction = random.randint(1,4)
        return block[index] #tuple: x, y
        
    def getCommanderLoc(self):
        return self.comm_x, self.comm_y

def findPossibleLocations(x_0,y_0,block,dist,quarter):
    #print("dist: " + str(dist))
    #print("x_0: " +str(x_0) + " y_0: " +str(y_0))
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

            #if (dist-1)**2 <= (x-x_0)**2 + (y-y_0)**2 <= (dist+1)**2:
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
        print("num: "+str(num))
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
        print("chosen qurter: "+str(quarter))
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
    #print("possible_locations: " +str(possible_locations))
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


"""t = surface.surface('dsm_1m_utm18_e_10_104.tif','dtm_1m_utm18_e_10_104.tif',0.5,50)
diffs = surface.surface.getDiffs(t)
PIL.Image.MAX_IMAGE_PIXELS = 100000000
img = np.array(PIL.Image.open("colorhillshade_dsm_1m_utm18_e_10_104.tif"))
img = img[0:5000,5000:]
mask = np.zeros_like(img)
img[:,:] = [255, 255, 255]
img[diffs<=5] = [0,0,0]

# Define the size of the window in each dimension
window_size = (25, 25, 1)  # or any other size you prefer

# Define the kernel to be used for convolution
kernel = np.ones(window_size) / np.prod(window_size)

# Apply the convolution operation to the image
average_image = convolve(img, kernel, mode='constant', cval=0.0)

# Display the resulting average image
print(average_image.shape)
plt.imshow(average_image)
plt.show()
img[:,:] = [255,255,255]
img[diffs<5] = [0,0,0]
plt.imshow(img)
plt.show()
diffs = unit.getCommandDirec(t)
diffs = np.sort(diffs)


summing = np.sum(diffs)
print(summing)
non_zeros = np.count_nonzero(t.getDiffs())
print("non_zeros: " +str(non_zeros))
print("avg height: " +str(summing/non_zeros))

#diffs = (diffs<3).astype(int)
#print(5000**2-np.count_nonzero(diffs))
sec_diff = (t.getDiffs()<2).astype(int)
print("non zeros which are smaller than 2: " +str(np.count_nonzero(sec_diff)-8300000))

dtm = t.getDTM()
new_diffs = (t.getDiffs()<0.2).astype(int)
dtm = (dtm>=0).astype(int)
dtm = dtm*new_diffs
print("zeros which aren't blacks: " +str(np.count_nonzero(dtm)))

summing = np.sum(t.getDTM())
print(summing)
non_zeros = np.count_nonzero(t.getDTM())
print("non_zeros: " +str(non_zeros))
print("DTM avg height: " +str(summing/non_zeros))

summing = np.sum(t.getDSM())
print(summing)
non_zeros = np.count_nonzero(t.getDSM())
print("non_zeros: " +str(non_zeros))
print("DSM avg height: " +str(summing/non_zeros))



PIL.Image.MAX_IMAGE_PIXELS = 100000000
img = np.array(PIL.Image.open("colorhillshade_dsm_1m_utm18_e_10_104.tif"))
img = img[0:5000,5000:]
img[t.getDiffs()<0.5] = [0,0,0]
print(img.shape)
plt.imshow(img)
plt.show()

img2 = plt.imread("colorhillshade_dsm_1m_utm18_e_10_104.tif")
img2 = img2[0:5000,5000:]
print(img2.shape)
print(img2)
plt.imshow(img2)
plt.show()"""

