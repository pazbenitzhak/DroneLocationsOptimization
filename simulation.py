
#args for main function: map_path, number of units, number of soldiers in unit, number of drones
import sys
import time_module
import surface
import unit_module
import soldier
import drones
import matplotlib.pyplot as plt
import numpy as np
import drone_manage


#run command: python simulation.py dtm_1m_utm18_e_10_104.tif 1 5 1 200
#u.e. spec: https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=2507

#all distances are in meters
time_interval = 1
drone_threshold = 150 #m
soldier_threshold = 0.5 #m
soldier_bw = 10 #TODO: change it to a proper value
noise = 9 #TODO: change it to a proper value
max_dist_from_comm = 100 #m
drone_bw = 10*10**6 ##10MHZ in specification (article), in Hz
ptx = 0.2511886432 #24 DBm, 0.2511886432 Watt, according to article
initial_drone_freq = 2*10**9 #2000MHz/2GHz

def main():
    dtm_path, units_num, soldiers_in_unit_num, drones_num, end_time = sys.argv[1], sys.argv[2],\
    sys.argv[3], sys.argv[4], sys.argv[5]
    #init time
    time = time_module.time_module(0,time_interval)
    #init surface
    surface_obj = surface.surface(dtm_path, soldier_threshold, drone_threshold)
    #load img for simulation
    """orig_img = plt.imread("colorhillshade_dsm_1m_utm18_e_10_104.tif")
    orig_img = orig_img[0:5000,5000:]
    orig_img_write = np.copy(orig_img)
    orig_img_write[surface_obj.getDiffs()<=0.5] = [255,255,255] #white
    orig_img_write[surface_obj.getDiffs()>0.5] = [0,0,0] #black"""
    orig_img_write = plt.imread("lines.png")
    orig_img_write = np.round(orig_img_write * 255).astype(np.uint8)
    copy_img = np.copy(orig_img_write)
    #init units
    units = [unit_module.unit_module([], surface_obj) for i in range(int(units_num))]
    #init soldiers in units
    block = surface.surface.getBlocks(surface_obj)
    for unit in units:
        print("unit!")
        comm_x, comm_y = unit_module.unit_module.setCommanderLoc(unit, time)
        copy_img[comm_x,comm_y] = [255,165,0] #orange
        units_soldiers = unit_module.unit_module.getSoldiers(unit)
        #TODO: consult about number of soldiers in unit - MERAV - 10-20, maybe decide randomly about number of soldiers
        for i in range(int(soldiers_in_unit_num)):
            print("soldier!")
            sold = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
            s_x, s_y = soldier.soldier.getLocation(sold)
            if i==0:
                copy_img[s_x,s_y] = [255,0,0] #red
            if i==1:
                copy_img[s_x,s_y] = [0,0,255] #blue
            if i==2:
                copy_img[s_x,s_y] = [255,0,255] #magenta
            if i==3:
                copy_img[s_x,s_y] = [0,128,0] #green
            if i==4:
                copy_img[s_x,s_y] = [184,134,11] #dark golden rod
            units_soldiers.append(sold)
    #for Algorithm 0 every unit has only one drone attached to it
    drones_list = []
    drones_static_locations = []
    drones_dynamic_routes = []
    for i in range(int(drones_num)):
        unit_xy = unit_module.unit_module.getCommanderLoc(units[i])
        unit_x, unit_y = unit_xy
        #TODO: add drone frequency
        drones_list.append(drones.drone(unit_x, unit_y, drone_bw, [units[i]], surface_obj, ptx, initial_drone_freq))
        #drones_static_locations.append(get_drone_static_location(drone))
        #drones_dynamic_routes.append(set_dynamic_route(drone))
    while(time_module.time_module.getTime(time)<int(end_time)):
        #TODO: measure current SNR
        avg_SNR_drones = []
        for i in range(int(drones_num)):
            SNR_i = 0
            drone_loc = drones.drone.getLocation(drones_list[i])
            unit_i_solds = unit_module.unit_module.getSoldiers(units[i])
            sold_num = unit_module.unit_module.getSoldiersNum(units[i])
            for sold in unit_i_solds:
                SNR_i += drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drone_loc)
            SNR_i = SNR_i/sold_num
            avg_SNR_drones.append(SNR_i)
        #update soldiers
        for unit in units:
            comm_x, comm_y = unit_module.unit_module.setCommanderLoc(unit, time)
            copy_img[comm_x,comm_y] = [255,165,0] #orange
            unit_soldiers = unit_module.unit_module.getSoldiers(unit)
            i = 0
            for sold in unit_soldiers:
                s_x, s_y = soldier.soldier.updateNewLocation(sold,max_dist_from_comm,time)
                if i==0:
                    copy_img[s_x,s_y] = [255,0,0] #red
                if i==1:
                    copy_img[s_x,s_y] = [0,0,255] #blue
                if i==2:
                    copy_img[s_x,s_y] = [255,0,255] #magenta
                if i==3:
                    copy_img[s_x,s_y] = [0,128,0] #green
                if i==4:
                    copy_img[s_x,s_y] = [184,134,11] #dark golden rod
                i+=1
                #copy_img[sold_x,sold_y] = [255,0,0] #red

        algo_param = []
        for i in range(len(drones_list)):
            drone = drones_list[i]
            algo_param.extend(drone_manage.calculate_next_locs_data(units[i],drone,surface_obj))
        print("SNRS: " +str(algo_param))
        #optimization algorithm
        #locations = opt_alg(algo_param)
        locations = [drones.drone.placeDrone(drones_list[i],unit_module.unit_module.getCommanderLoc(units[i])[0],
                    unit_module.unit_module.getCommanderLoc(units[i])[1],surface_obj) for i in range(len(drones_list))] #TODO: insert op algo
        #update drones (data from op algo)
        for ind in range(len(drones_list)):
            drone = drones_list[i]
            drone_x, drone_y = drones.drone.setNewLocation(drone,locations[ind][0],locations[ind][1])
            copy_img[drone_x,drone_y] = [106,90,205] #purple
        #for i in range(5000):
         #   for j in range(5000):
          #      copy_img[i,j] = [255,0,0]
        #TODO: measure necessary parameters
        #increment time
        time_module.time_module.updateTime(time)
    plt.imshow(copy_img)
    plt.show()
    print("finish")

    #MERAV: static: where DTM is highest, in the area where the group is supposed to be


""" def main1():
    #static drones positioning
    dsm_path, dtm_path, units_num, soldiers_in_unit_num, drones_num, end_time = sys.argv[1], sys.argv[2],\
    sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6]
    #init time
    time = time_module.time_module(0,time_interval)
    #init surface
    surface_obj = surface.surface(dsm_path, dtm_path, soldier_threshold, drone_threshold)
    #init units
    units = [unit_module.unit_module([], surface_obj) for i in range(int(units_num))]
    #init soldiers in units
    for unit in units:
        units_soldiers = unit_module.unit_module.getSoldiers(unit)
        #TODO: consult about number of soldiers in unit
        for i in range(int(soldiers_in_unit_num)):
            sold = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
            units_soldiers.append(sold)
    #for Algorithm 0 every unit has only one drone attached to it
    drones_list = [drones.drone(unit_module.unit_module.getCommanderLoc(unit[i])[0],unit_module.unit_module.getCommanderLoc(unit[i])[1],
                drone_bw, [units[i]],surface_obj) for i in range(int(drones_num))]
    while(time_module.time_module.getTime(time)<end_time):
        #update soldiers
        for unit in units:
            unit_module.unit_module.setCommanderLoc(unit, time)
            unit_soldiers = unit_module.unit_module.getSoldiers(unit)
            for sold in unit_soldiers:
                soldier.soldier.updateNewLocation(sold,max_dist_from_comm)
        #static locations - no need to relocate drones
        #increment time
        time_module.time_module.updateTime(time)
    print("finish") """

if __name__ == "__main__":
    main()


