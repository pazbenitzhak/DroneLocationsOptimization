import random
import surface as surface
import numpy as np
import unit_module as unit_module

drone_height = 50 #m

class drone:
    x_loc = 0
    y_loc = 0
    bandwidth = 0 #10MHZ in specification
    height = 0
    ptx = 0 # power of drone broadcast towards users
    connected_units = []

    def __init__(self,x_unit,y_unit,bw, units,surface_object,ptx):
        self.x_loc, self.y_loc= self.placeDrone(x_unit, y_unit,surface_object)
        self.user_bandwidth = bw
        self.height = drone_height+ surface.surface.getDSM(surface_object)[x_unit,y_unit]
        #MERAV: maybe 50 above dtm and need to avoid collusions with buildings
        self.connected_units = units
        self.ptx = ptx

    def getLocation(self):
        return self.x_loc, self.y_loc

    def getBandwidth(self):
        return self.bandwidth

    def updateHeight(self,x,y,surface_object):
        self.height = drone_height+ surface.surface.getDSM(surface_object)[x,y]

    def getHeight(self):
        return self.height

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


    def setNewLocation(self,x,y): #deterministic
        self.x_loc = x
        self.y_loc = y
        return x,y

    def getUnits(self): 
        return self.connected_units #returns a list object that can be changed
    
    def getTotalSoldServed(self):
        num = 0
        units = self.getUnits()
        for unit in units:
            num += unit_module.unit.getSoldiersNum(unit)
        return num

    def getPtx(self):
        return self.ptx
