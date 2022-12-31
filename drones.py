import random
from unit_module import unit_module
from time_module import time_module
import numpy as np

drone_height = 50

class drone:
    x_loc = 0
    y_loc = 0
    bandwidth = 0
    height = 0
    connected_units = []

    def __init__(self,x,y,bw, units):
        self.x_loc = x
        self.y_loc=y
        self.user_bandwidth = bw
        self.height = drone_height #TODO: h+getDSM(x,y)
        self.connected_units = units

    def getLocation(self):
        return self.x_loc, self.y_loc

    def getBandwidth(self):
        return self.bandwidth

    def updateHeight(self):
        self.height = drone_height #TODO: h+getDSM(x,y)


    def setNewLocation(self,x,y): #deterministic
        self.x_loc = x
        self.y_loc = y
        return

    def getUnits(self): 
        return self.connected_units #returns a list object that can be changed

