import random
import surface as surface
import numpy as np
import unit_module as unit_module

drone_height = 50 #m

class drone:
    x_loc = 0
    y_loc = 0
    bandwidth = 0 #10MHZ in specification (article), in Hz
    height = 0
    ptx = 0 # power of drone broadcast towards users, Watt
    connected_units = []
    frequency = 0 #in Hz

    def __init__(self,x_unit,y_unit,bw,units,surface_object,ptx,freq,batt):
        self.x_loc, self.y_loc= self.placeDrone(x_unit, y_unit,surface_object)
        self.bandwidth = bw
        self.height = drone_height+surface.surface.getDTM(surface_object)[x_unit,y_unit]
        self.connected_units = units
        self.ptx = ptx
        self.frequency = freq
        self.battery = batt

    def getLocation(self):
        return self.x_loc, self.y_loc

    def getBandwidth(self):
        return self.bandwidth

    def updateHeight(self,x,y,surface_object):
        self.height = drone_height+ surface.surface.getDTM(surface_object)[x,y]

    def getHeight(self):
        return self.height
    
    def getBattery(self):
        return self.battery
    
    def getFreq(self):
        return self.frequency

    def placeDrone(self,comm_x,comm_y,surface_object):
        drone_map = surface.surface.getDroneMap(surface_object)
        if drone_map[comm_x,comm_y] == 1:
            return comm_x, comm_y
        # not a valid point
        count = 0
        while(True):
            count += 1
            new_x = int(random.gauss(comm_x,4))
            new_y = int(random.gauss(comm_y,4))
            if (new_x<0 or new_x>4999) or (new_y<0 or new_y>4999):
                #choose another point
                continue
            if (drone_map[new_x,new_y]==1):
                return new_x, new_y
            if (count==100):
                break
        one_value_indices = np.argwhere(drone_map==1)
        length = len(one_value_indices)
        ind = random.randint(0,length-1)
        return one_value_indices[ind][0], one_value_indices[ind][1]


    def setNewLocation(self,x,y, surf): #deterministic
        self.x_loc = x
        self.y_loc = y
        self.updateHeight(x, y, surf)
        return x,y

    def getUnits(self): 
        return self.connected_units #returns a list object that can be changed
    
    def getTotalSoldServed(self):
        num = 0
        units = self.getUnits()
        for unit in units:
            num += unit_module.unit_module.getSoldiersNum(unit)+1 #adding the commander manually
        return num

    def getPtx(self):
        return self.ptx
    
    def updateBattery(self):
        #we know that the such drones' battery can last for 3 hours
        # that's why in any time interval of one second we need to substract
        # the battery by 1/10800
        return self.battery-(1/10800)
