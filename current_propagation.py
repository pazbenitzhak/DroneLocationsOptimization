
import surface as surface
import soldier as soldier
import drones as drones
import numpy as np

c = 3*10**8 # speed of light

def los_path_loss(sold, drone, surf, freq):
    #distance between soldier and drone
    #hb - height of drone
    #hm - height of soldier
    #lambda - frequency of drone
    #TODO: figure out what the frequency should be - MERAV ANSWER: cellular frequency
    #we assume the wave velocity is at the speed of light
    drone_loc = drones.drone.getLocation(drone)
    hb = drones.drone.getHeight(drone)
    sold_loc = soldier.soldier.getLocation(sold)
    hm = surface.surface.getDSM(surf)[sold_loc]
    vertical_dist = np.square(hb-hm)
    horizontal_dist = np.square(drone_loc[0]-sold_loc[0])+np.square(drone_loc[1]-sold_loc[1])
    dist = np.sqrt(vertical_dist+horizontal_dist)
    wave_length = c/freq
    rbp = (4*hb*hm)/wave_length
    lbp = np.abs(20*np.log10((wave_length**2)/(8*np.pi*hb*hm)))
    if (dist<=rbp):
        coeff = 20
    else:
        coeff = 40
    return lbp+6+coeff*np.log10(dist/rbp)

def nlos_path_loss(sold, drone, surf, freq):
    drone_loc = drones.drone.getLocation(drone)
    hb = drones.drone.getHeight(drone)
    sold_loc = soldier.soldier.getLocation(sold)
    hm = surface.surface.getDSM(surf)[sold_loc]+1.5
    vertical_dist = np.square(hb-hm)
    horizontal_dist = np.square(drone_loc[0]-sold_loc[0])+np.square(drone_loc[1]-sold_loc[1])
    dist = np.sqrt(vertical_dist+horizontal_dist)
    #street_width = surface.surface.
    # our world/map is straightened so we can just take 'regular' arctan
    phi = np.arctan2(np.abs(drone_loc[1]-sold_loc[1]),np.abs(drone_loc[0]-sold_loc[0]))
    if phi>90:
        phi -= 90
    L_bf = 32.4 + 20*np.log10(dist/1000) + 20*np.log10(freq)
    block_num = calcBlock(sold_loc)
    dsm_blocks = surface.surface.getDSMBlocks(surf)
    L_rts = calc_Lrts(street_width, freq)
    L_msd = calc_Lmsd()
    if ((L_msd+L_rts)<=0):
        return L_bf
    return L_bf+L_rts+L_msd

def calc_Lrts(w,f,del_hm,phi):
    L_ori = calc_Lori(phi)
    return -8.2-10*np.log10(w)+10*np.log10(f)+20*np.log10(del_hm)+L_ori

def calc_Lmsd():
    return

def calc_Lori(phi):
    assert(0<=phi<=90)
    if (0<=phi<35):
        return -10*0.354*phi
    elif (35<=phi<55):
        return 2.5+0.075*(phi-35)
    else: # 55<=phi<=90
        return 4-0.114*(phi-55)
    
def calcBlock(sold_loc):
    x,y = sold_loc
    #TODO: MODULO
