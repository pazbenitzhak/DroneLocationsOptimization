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

avg_soldier_height = 1.5 #assuming soldier carriescommunication equipment on the back
percent = 5 
drone_height = 50

def calc_SNR(sold, drone, surf,drone_pos_loc):
    user_equip_noise = soldier.soldier.getSoldierEquipNoise(sold) #in DB
    total_soldiers_served = drones.drone.getTotalSoldServed(drone)
    bu = drones.drone.getBandwidth(drone)/total_soldiers_served #Hz 
    Nu = bu
    Nu = bu*10**(-3)
    exp = 10**(5+(-174+user_equip_noise)/10)
    Nu = Nu * exp
    Nu = Nu/(10**5)
    ptx = drones.drone.getPtx(drone) #Watt
    freq = drones.drone.getFreq(drone) #in Hz
    if is_line_of_sight(sold, surf, drone_pos_loc):
        path_loss = curr_prop.los_path_loss(sold,surf,freq,drone_pos_loc)
    else:
        path_loss = curr_prop.nlos_path_loss(sold,surf,freq,drone_pos_loc)
    s_path = (1/total_soldiers_served)*ptx*10**(-path_loss/10) #bu/B = 1/total_soldiers_served
    snr = s_path/Nu
    #when converting a quantity that relates to power, the coefficient should be 10
    snr_db = 10*np.log10(snr)
    snr = snr_db
    return snr



def is_line_of_sight_2(sold, drone, surf):
    x_drone, y_drone = drones.drone.getLocation(drone)
    z_drone =  drones.drone.getHeight(drone)
    x_sold, y_sold = soldier.soldier.getSoldierLocation(sold)
    z_sold = avg_soldier_height+surface.surface.getDSM(surf)[x_sold, y_sold]
    #DSM & DTM are  almost equal, DSM is slightly higher. We chose DSM for optimizations reasons
    #calculate line equation
    slope = ((y_drone-y_sold)/(x_drone-x_sold))
    line = lambda x: slope*(x-x_drone)+y_drone
    #calculate angle of the 3D vector connecting the drone and the soldier
    horizontal_dist = np.sqrt((x_drone-x_sold)**2+(y_drone-y_sold)**2)
    vertical_dist = z_drone-z_sold
    # the angle is not needed for the calculation of los' heights
    proportion = vertical_dist/horizontal_dist
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
              dist = np.sqrt((prev_x-x_sold)**2+(y_tag-y_sold)**2)
              #get the height of each point on the line and compare it to the line of sight
              if (dist*proportion<=surface.surface.getDSM(surf)[x, y_tag]):
                return False
              prev_y = y_tag
          for y_tag in range(prev_y,y+1):
            dist = np.sqrt((x-x_sold)**2+(y_tag-y_sold)**2)
            if (dist*proportion<=surface.surface.getDSM(surf)[x, y_tag]):
                return False
        else: #slope<=0
          #now iterating backwards because y values decrease
          if (y<prev_y):
            for y_tag in range(prev_y, y-1,-1):
              dist = np.sqrt((prev_x-x_sold)**2+(y_tag-y_sold)**2)
              if (dist*proportion<=surface.surface.getDSM(surf)[x, y_tag]):
                return False
              prev_y = y_tag
          for y_tag in range(prev_y,y-1,-1):
            dist = np.sqrt((x-x_sold)**2+(y_tag-y_sold)**2)
            if (dist*proportion<=surface.surface.getDSM(surf)[x, y_tag]):
                return False
        prev_y = y
        prev_x = x
    # if we got here, there is LOS
    return True


def is_line_of_sight(sold,surf,drone_pos_loc):
    x_drone, y_drone = drone_pos_loc
    #we changed it so it can fit calculations which are not related to a the drone's current location - in the dynamic and static options 
    dtm = surface.surface.getDTM(surf)
    z_drone =  dtm[drone_pos_loc]+drone_height
    x_sold, y_sold = soldier.soldier.getSoldierLocation(sold)
    z_sold = avg_soldier_height+surface.surface.getDSM(surf)[x_sold, y_sold]
    line_points = bresenham_line(x_drone, y_drone, x_sold, y_sold)
    three_d_direc = (x_drone-x_sold,y_drone-y_sold,z_drone-z_sold)
    #we'll calculate l: the parameteric distribution of the vector between
    #the drone and the soldier
    #l: sold_loc+t*three_d_direc where t is a constant which will be calculated
    #in the for loop
    #for every point we want to calculate its expected line z
    for point in line_points:
        x_direc, y_direc, z_direc = three_d_direc
        x_point, y_point = point
        coeff = 1
        if x_direc==0:
           coeff = 2
           tx = 0
        else: 
            tx = (x_point-x_sold)/x_direc
        if y_direc==0:
           coeff = 2
           ty = 0
        else: 
            ty = (y_point-y_sold)/y_direc
        dsm_val = surface.surface.getDSM(surf)[x_point,y_point] #height in the pixel
        z_point = z_sold+(coeff*(tx+ty)/2)*z_direc
        # case where tx=ty=0 works because 1.5>0
        if (z_point<=(1+(-percent/100))*dsm_val):
           return False
    return True

def bresenham_line(x1, y1, x2, y2):
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x_step = 1 if x1 < x2 else -1
    y_step = 1 if y1 < y2 else -1

    line_points = []

    if dx >= dy:
        error = dx / 2
        y = y1

        for x in range(x1, x2 + x_step, x_step):
            line_points.append((x, y))
            error -= dy

            if error < 0:
                y += y_step
                error += dx
    else:
        error = dy / 2
        x = x1

        for y in range(y1, y2 + y_step, y_step):
            line_points.append((x, y))
            error -= dx

            if error < 0:
                x += x_step
                error += dy

    return line_points

