import numpy as np
import soldier as soldier
import drones as drones
import unit_module as unit_module
import current_propagation as curr_prop
import surface as surface
import decimal

# calculates the SNR between a drone and a soldier
# uses current_propagation module in order to calculate path loss
# need to distinguish for every soldier and drone whether there's a line of sight or not

avg_soldier_height = 1.5 #carries equipment on the back

def calc_SNR(sold, drone, surf,drone_pos_loc):
    user_equip_noise = soldier.soldier.getSoldierEquipNoise(sold) #in DB
    total_soldiers_served = drones.drone.getTotalSoldServed(drone)
    bu = drones.drone.getBandwidth(drone)/total_soldiers_served #Hz
    Nu = bu*10**(-3)
    print("Nu1: " + str(Nu))
    exp = 10**(5+(-174+user_equip_noise)/10)
    print("exp: "+str(exp))
    Nu = Nu * exp
    print("Nu2: " + str(Nu))
    Nu = Nu/(10**5)
    print("Nu3: " + str(Nu))
    ptx = drones.drone.getPtx(drone) #Watt
    freq = drones.drone.getFreq(drone) #in Hz
    if is_line_of_sight(sold, drone, surf):
        print("in LOS")
        path_loss = curr_prop.los_path_loss(sold,drone,surf,freq,drone_pos_loc)
    else:
        print("in NLOS")
        path_loss = curr_prop.nlos_path_loss(sold,drone,surf,freq,drone_pos_loc)
    print("path loss: " +str(path_loss))
    s_path = (1/total_soldiers_served)*ptx*10**(-path_loss/10) #bu/B = 1/total_soldiers_served
    print("s_path: " +str(s_path))
    print("Nu: " +str(Nu))
    return s_path/Nu



def is_line_of_sight(sold, drone, surf):
    x_drone, y_drone = drones.drone.getLocation(drone)
    z_drone =  drones.drone.getHeight(drone)
    x_sold, y_sold = soldier.soldier.getSoldierLocation(sold)
    z_sold = avg_soldier_height+surface.surface.getDSM(surf)[x_sold, y_sold]
    #DSM & DTM arecd  almost equal, DSM is slightly higher. We chose DSM for optimizations reasons
    #calculate line equation
    slope = ((y_drone-y_sold)/(x_drone-x_sold))
    line = lambda x: slope*(x-x_drone)+y_drone
    #calculate angle of the 3D vector connecting the drone and the soldier
    horizontal_dist = np.sqrt((x_drone-x_sold)**2+(y_drone-y_sold)**2)
    vertical_dist = z_drone-z_sold
    # the angle is not needed for the calculation of los' heights
    proportion = vertical_dist/horizontal_dist
    points = set({})
    if (x_drone<x_sold):
        smaller_x = x_drone
        prev_y = y_drone
        largest_x = x_sold
    else:
        smaller_x = x_sold
        prev_y = y_sold
        largest_x = x_drone
    # iterate through all x's in the range and add the pixels that the line from
    # the drone to the soldier crosses
    for x in range(smaller_x, largest_x+1):
        y = int(np.floor(line(x)))
        if (slope>0):
          if (y>prev_y):
            #
            for y_tag in range(prev_y, y+1):
              #points.add((prev_x,y_tag))
              dist = np.sqrt((prev_x-x_sold)**2+(y_tag-y_sold)**2)
              #get the height of each point on the line and compare it to the line of sight
              if (dist*proportion>=surface.surface.getDSM(surf)[x, y_tag]):
                return False
              prev_y = y_tag
          for y_tag in range(prev_y,y+1):
            dist = np.sqrt((x-x_sold)**2+(y_tag-y_sold)**2)
            #points.add((x,y_tag))
            if (dist*proportion>=surface.surface.getDSM(surf)[x, y_tag]):
                return False
        else: #slope<=0
          #now iterating backwards because y values decrease
          if (y<prev_y):
            for y_tag in range(prev_y, y-1,-1):
              #points.add((prev_x,y_tag))
              dist = np.sqrt((prev_x-x_sold)**2+(y_tag-y_sold)**2)
              if (dist*proportion>=surface.surface.getDSM(surf)[x, y_tag]):
                return False
              prev_y = y_tag
          for y_tag in range(prev_y,y-1,-1):
            dist = np.sqrt((x-x_sold)**2+(y_tag-y_sold)**2)
            #points.add((x,y_tag))
            if (dist*proportion>=surface.surface.getDSM(surf)[x, y_tag]):
                return False
        prev_y = y
        prev_x = x
    # if we got here, there is LOS
    return True




"""def is_line_of_sight(sold, drone, surf):
    x_drone, y_drone, z_drone = drones.drone.getLocation(drone), drones.drone.getHeight(drone)
    x_sold, y_sold = soldier.soldier.getSoldierLocation(sold)
    z_sold = avg_soldier_height+surface.surface.getDSM(surf)[x_sold, y_sold]
    #DSM & DTM arecd  almost equal, DSM is slightly higher. We chose DSM for optimizations reasons
    #calculate line equation
    slope = ((y_drone-y_sold)/(x_drone-x_sold))
    line = lambda x: slope*(x-x_drone)+y_drone
    #calculate angle of the 3D vector connecting the drone and the soldier
    horizontal_dist = np.sqrt((x_drone-x_sold)**2+(y_drone-y_sold)**2)
    vertical_dist = z_drone-z_sold
    proportion = vertical_dist/horizontal_dist
    if (x_drone<x_sold):
        smaller_x = x_drone
        prev_y = y_drone
        largest_x = x_sold
        end_y = y_sold
    else:
        smaller_x = x_sold
        prev_y = y_sold
        largest_x = x_drone
        end_y = y_drone
    for x in range(smaller_x, largest_x+1):
        y = np.floor(line(x))
        for y_tag in range(prev_y,y+1):
            dist = np.sqrt((x-x_sold)**2+(y_tag-y_sold)**2)
            if (dist*proportion>=surface.surface.getDSM(surf)[x, y_tag]):
                return False
        prev_y = y
    # if we got here, there is LOS
    return True
"""