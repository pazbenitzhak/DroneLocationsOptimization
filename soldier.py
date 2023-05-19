import random
import unit_module
import time_module
import numpy as np


#specification: https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=2507

class soldier:

    def __init__(self,bw, noise,unit,max_dist_from_comm):
        self.unit = unit
        self.user_bw = bw 
        self.user_equip_noise = noise #9 db in specification
        self.x_loc, self.y_loc= self.placeSoldier(max_dist_from_comm)

    def getLocation(self):
        return self.x_loc, self.y_loc

    def updateNewLocation(self,max_dist_from_comm,time): #random
        avg_velocity =  unit_module.avg_vel #m/sec, i.e. 5km/h
        velocity_deviation =  unit_module.vel_dev #chosen so the velocity ranges are between 0-9.5 km/h
        #angle_deviation = 1.18 # chosen so the direction ranges are between commander_dirc+-pi
        velocity_val = random.gauss(avg_velocity,velocity_deviation)
        #velocity_direc = random.gauss(unit_module.getCommandDirec(),angle_deviation)
        dist = velocity_val*time_module.time_module.getTimeInterval(time)
        unit = self.getSoldierUnit()
        block = unit_module.unit_module.getBlock(unit)
        x_0,y_0 = self.getSoldierLocation()
        comm_x, comm_y = unit_module.unit_module.getCommanderLoc(unit)
        quarter = unit_module.calculateQuarter(comm_x, comm_y, x_0, y_0)
        possible_locations = unit_module.findPossibleLocations(x_0,y_0,block,dist,quarter)
        min_dist_from_cmm = np.infty
        backup_loc_min = (0,0) #setup for case where all possible locations are not <= max_dist_from_comm
        is_not_valid = 1
        while(is_not_valid):
            arr_len = len(possible_locations)
            if arr_len == 0: #in first itertation, arr_len will always be greater that 0
                self.setNewLocation(backup_loc_min[0], backup_loc_min[1])
                return backup_loc_min[0], backup_loc_min[1] #if none of possible locations apply demands, then the minimum distance will be taken
            index = random.randint(0,arr_len-1)
            new_loc = possible_locations[index] #tuple: x, y
            new_dist_from_cmm = (new_loc[0]-comm_x)*2 + (new_loc[1]-comm_y)*2 
            if new_dist_from_cmm<= max_dist_from_comm**2:
                is_not_valid = 0
            else:
                if new_dist_from_cmm <= min_dist_from_cmm:
                    min_dist_from_cmm = new_dist_from_cmm
                    backup_loc_min = (possible_locations[index])
                possible_locations.remove(new_loc)
        self.setNewLocation(new_loc[0],new_loc[1])
        return new_loc[0], new_loc[1]

    def placeSoldier(self,max_dist_from_comm):
        block = unit_module.unit_module.getBlock(self.unit)
        comm_x, comm_y = unit_module.unit_module.getCommanderLoc(self.unit)
        soldiers_placements = []
        for i in range(-max_dist_from_comm, max_dist_from_comm):
            for j in range(-max_dist_from_comm, max_dist_from_comm):
                if (0<=i+comm_x<5000 and 0<=j+comm_y<5000) and (i**2+j**2<=max_dist_from_comm**2) \
                                                            and (block[comm_x+i, comm_y+j]==1):
                    #NPV: if (x,y) in block
                    soldiers_placements.append((comm_x+i, comm_y+j))
        placements_num = len(soldiers_placements)
        index = random.randint(0,placements_num-1)
        return soldiers_placements[index][0], soldiers_placements[index][1] #tuple: x, y

    def setNewLocation(self,x,y): #deterministic
        self.x_loc = x
        self.y_loc = y
        return

    def getSoldierLocation(self):
        return self.x_loc, self.y_loc

    def getSoldierBW(self):
        return self.user_bw

    def getSoldierEquipNoise(self):
        return self.user_equip_noise
    
    def getSoldierUnit(self):
        return self.unit

    

