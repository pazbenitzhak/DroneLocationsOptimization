import random
from unit_module import unit_module
from time_module import time_module
import numpy as np
class soldier:
    x_loc = 0
    y_loc = 0
    user_bw = 0
    user_equip_noise = 0 #db

    def __init__(self,x,y,bw, noise):
        self.x_loc = x
        self.y_loc=y
        self.user_bw = bw
        self.user_equip_noise = noise

    def getLocation(self):
        return self.x_loc, self.y_loc

    def updateNewLocation(self): #random
        avg_velocity = 1.388 #m/sec, i.e. 5km/h
        velocity_deviation =  0.5 #chosen so the velocity ranges are between 0-9.5
        angle_deviation = 1.18 # chosen so the direction ranges are between commander_dirc+-pi
        velocity_val = random.gauss(avg_velocity,velocity_deviation)
        velocity_direc = random.gauss(unit_module.getCommandDirec(),angle_deviation)
        dist = velocity_val*time_module.getTimeInterval()
        new_x = self.x_loc+dist*np.cos(velocity_direc)
        new_y = self.y_loc+dist*np.sin(velocity_direc)
        self.x_loc = int(new_x)
        self.y_loc = int(new_y)
        #TODO: check cases where new indices are out of bounds with the matrix

    def setNewLocation(self,x,y): #deterministic
        self.x_loc = x
        self.y_loc = y
        return

    def getSoldierBW(self):
        return self.user_bw

    def getSoldierEquipNoise(self):
        return self.user_equip_noise

