import wave_propagation as wp
import soldier as soldier
import surface as surface
import drones as drones
import unit_module
import numpy as np

drone_step_size = 5 #the smallest step
map_size = 5000
max_dist_from_comm = 100

def calculate_next_locs_data(unit,drone,surf):
    #drone snrs with the unit
    unit_snrs = []
    soldiers = unit_module.unit_module.getSoldiers(unit)
    possible_drone_locations = get_poss_drone_locs(drone,surf)
    #creating a commander soldier object only fot the calculations
    comm_x, comm_y = unit_module.unit_module.getCommanderLoc(unit)
    first_sold = soldiers[0]
    soldier_bw = soldier.soldier.getSoldierBW(first_sold)
    noise = soldier.soldier.getSoldierBW(first_sold)
    commander = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
    soldier.soldier.setNewLocation(commander,comm_x, comm_y)

    #we're building 2 lists of the same length: one will contain the drone's possible locations
    #(x,y) points, and the second will have all of the soldiers' snr values in relation to the drone
    #located in the same point so every index within the 2 lists will match
    for location in possible_drone_locations:
        soldiers_snr = []
        #calc commander snr
        soldiers_snr.append(collect_soldier_snr(commander,drone,surf,location))
        for sold in soldiers:
            soldiers_snr.append(collect_soldier_snr(sold,drone,surf,location))
            soldier_bw = soldier.soldier.getSoldierBW(sold)
            noise = soldier.soldier.getSoldierBW(sold)
        unit_snrs.append(soldiers_snr)
    battery = drones.drone.getBattery(drone)
    return unit_snrs, possible_drone_locations, battery

def get_poss_drone_locs(drone, surf):
    valid_locs = surface.surface.getDroneMap(surf)
    x,y = drones.drone.getLocation(drone)
    points = [(x,y)]
    first_drone_step = drone_step_size
    second_drone_step = 2*drone_step_size
    third_drone_step = 4*drone_step_size
    first_step_points = add_locs_for_step_size(x,y,first_drone_step,valid_locs)
    points.extend(first_step_points)
    second_step_points = add_locs_for_step_size(x,y,second_drone_step,valid_locs)
    points.extend(second_step_points)
    third_step_points = add_locs_for_step_size(x,y,third_drone_step,valid_locs)
    points.extend(third_step_points)
    return points


def add_locs_for_step_size(x,y,step_size,valid_locs):
    points_list = []
    diag_step_size = int(drone_step_size*(1/np.sqrt(2)))
    if (x-drone_step_size>=0):
        point = (x-drone_step_size,y)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append((x-drone_step_size,y))
    if (x+drone_step_size<map_size):
        point = (x+drone_step_size,y)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (y-drone_step_size>=0):
        point = (x,y-drone_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (y+drone_step_size<map_size):
        point = (x,y+drone_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (x+diag_step_size<map_size and y+diag_step_size<map_size):
        point = (x+diag_step_size,y+diag_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (x+diag_step_size<map_size and y-diag_step_size>=0):
        point = (x+diag_step_size,y-diag_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (x-diag_step_size>=0 and y-diag_step_size>=0):
        point = (x-diag_step_size,y-diag_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    if (x-diag_step_size>=0 and y+diag_step_size<map_size):
        point = (x-diag_step_size,y+diag_step_size)
        # if 1 then the location is valid due to the construction of the drone map
        if valid_locs[point]==1:
            points_list.append(point)
    return points_list

def collect_soldier_snr(sold,drone,surf,location):
    return wp.calc_SNR(sold, drone, surf,location)