
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
import random
import opt_algo as op
#dtm_1m_utm18_e_10_104.tif
#run command: python simulation.py dtm_data.npy 1 5 1 200
#u.e. spec: https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=2507

#all distances are in meters
time_interval = 1
drone_threshold = 50 #m
soldier_threshold = 0.5 #m
soldier_bw = 10 #TODO: change it to a proper value
noise = 9 #TODO: change it to a proper value
max_dist_from_comm = 200 #m
drone_bw = 10*10**6 ##10MHZ in specification (article), in Hz
ptx = 0.2511886432 #24 DBm, 0.2511886432 Watt, according to article
initial_drone_freq = 2*10**9 #2000MHz/2GHz
map_size = 5000
ellipse_a = 100
ellipse_b = 200
block_size = 100
init_batt = 50/100

def main():
    dtm_path, units_num, soldiers_in_unit_num, drones_num, end_time = sys.argv[1], sys.argv[2],\
    sys.argv[3], int(sys.argv[4]), sys.argv[5]
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
        comm_x, comm_y = unit_module.unit_module.setCommanderLoc(unit, time)
        copy_img[comm_x,comm_y] = [255,165,0] #orange
        units_soldiers = unit_module.unit_module.getSoldiers(unit)
        #TODO: consult about number of soldiers in unit - MERAV - 10-20, maybe decide randomly about number of soldiers
        for i in range(int(soldiers_in_unit_num)):
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
            if i==5:
                copy_img[s_x,s_y] = [124,252,0] #lawn green
            if i==6:
                copy_img[s_x,s_y] = [143,188,143] #dark sea green
            if i==7:
                copy_img[s_x,s_y] = [47,79,79] #dark slate gray
            if i==8:
                copy_img[s_x,s_y] = [100,149,237] #corn flower blue
            if i==9:
                copy_img[s_x,s_y] = [139,69,19] #saddle brown
            units_soldiers.append(sold)
    #for Algorithm 0 every unit has only one drone attached to it
    drones_list = []
    drones_static_locations = []
    drones_dynamic_routes = []
    for i in range(drones_num):
        unit_xy = unit_module.unit_module.getCommanderLoc(units[i])
        unit_x, unit_y = unit_xy
        #TODO: add drone frequency
        #create static 'drone'
        drones_static_locations.append(get_drone_static_location(surface_obj, units[i]))
        drone_initial_loc = drones_static_locations[i]
        drones_list.append(drones.drone(drone_initial_loc[0], drone_initial_loc[1], drone_bw, [units[i]],\
                                         surface_obj, ptx, initial_drone_freq,init_batt))
        #create a dynamic elliptic route
        ellip_cent_x, ellip_cent_y, theta = adjust_elliptic_param([unit_x,unit_y])
        drones_dynamic_routes.append(create_elliptic_route(ellip_cent_x, ellip_cent_y, ellipse_a, ellipse_b, theta))
        #print("drones_dynamic_routes : " +str(drones_dynamic_routes[i]) )
    curr_time = time_module.time_module.getTime(time)
    cum_avg_algo_SNR = [[] for i in range(drones_num)]
    cum_avg_static_SNR = [[] for i in range(drones_num)]
    cum_avg_dynamic_SNR = [[] for i in range(drones_num)]
    while(curr_time<int(end_time)):
        #TODO: measure current SNR
        print("curr time: " + str(curr_time))
        for i in range(drones_num):
            #update battery
            drones.drone.updateBattery(drones_list[i])
            algo_SNR_i = 0
            static_SNR_i = 0
            dynamic_SNR_i = 0
            drone_loc = drones.drone.getLocation(drones_list[i])
            #here the case is one unit for each drone
            unit_i_solds = unit_module.unit_module.getSoldiers(units[i])
            sold_num = unit_module.unit_module.getSoldiersNum(units[i])+1
            #create commander object for SNR calculations
            comm_x, comm_y = unit_module.unit_module.getCommanderLoc(unit)
            commander = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
            soldier.soldier.setNewLocation(commander,comm_x, comm_y)
            #calculate SNR for commander
            algo_SNR_i += drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drone_loc)
            static_SNR_i += drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drones_static_locations[i])
            dyn_route_len = len(drones_dynamic_routes[i])
            dynamic_SNR_i += drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drones_dynamic_routes[i][curr_time%dyn_route_len])
            #calculate SNR for other soldiers
            for sold in unit_i_solds:
                algo_SNR_i += drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drone_loc)
                #print("drones_algo_locations: "+str(drone_loc))
                #print("drones_static_locations[i]: "+str(drones_static_locations[i]))
                static_SNR_i += drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drones_static_locations[i])
                copy_img[drones_static_locations[i][0],drones_static_locations[i][1]] = [255,20,147] #deep pink
                dyn_route_len = len(drones_dynamic_routes[i])
                copy_img[drones_dynamic_routes[i][curr_time%dyn_route_len][0],drones_dynamic_routes[i][curr_time%dyn_route_len][1]] = [128,0,128] #dark purple
                dynamic_SNR_i += drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drones_dynamic_routes[i][curr_time%dyn_route_len])
            algo_SNR_i = algo_SNR_i/sold_num
            static_SNR_i = static_SNR_i/sold_num
            dynamic_SNR_i = dynamic_SNR_i/sold_num
            cum_avg_algo_SNR[i].append(algo_SNR_i)
            cum_avg_static_SNR[i].append(static_SNR_i)
            cum_avg_dynamic_SNR[i].append(dynamic_SNR_i)
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
                if i==5:
                    copy_img[s_x,s_y] = [124,252,0] #lawn green
                if i==6:
                    copy_img[s_x,s_y] = [143,188,143] #dark sea green
                if i==7:
                    copy_img[s_x,s_y] = [47,79,79] #dark slate gray
                if i==8:
                    copy_img[s_x,s_y] = [100,149,237] #corn flower blue
                if i==9:
                    copy_img[s_x,s_y] = [139,69,19] #saddle brown
                i+=1
                #copy_img[sold_x,sold_y] = [255,0,0] #red

        snr_param = []
        drones_locations = []
        drones_batteries = []
        for i in range(len(drones_list)):
            drone = drones_list[i]
            snrs, locations, battery = drone_manage.calculate_next_locs_data(units[i],drone,surface_obj)
            snr_param.append(snrs)
            drones_locations.append(locations)
            drones_batteries.append(battery)
        #print("SNRS: " +str(snrs[0]))
        if has_nan_value(snrs):
            break
        #optimization algorithm
        locations = op.optimization_algorithm(snr_param,drones_locations,drones_batteries)
        #update drones (data from op algo)
        for ind in range(len(drones_list)):
            drone = drones_list[i]
            drone_x, drone_y = drones.drone.setNewLocation(drone,locations[ind][0],locations[ind][1])
            copy_img[drone_x,drone_y] = [106,90,205] #purple
        #for i in range(5000):
         #   for j in range(5000):
          #      copy_img[i,j] = [255,0,0]
        #increment time
        time_module.time_module.updateTime(time)
        curr_time = time_module.time_module.getTime(time)
    #print("drones_dynamic_routes : " +str(drones_dynamic_routes[0]) )
    print("drones_algo_locations: "+str(drone_loc))    
    plt.imshow(copy_img)
    plt.show()
    times = range(curr_time)
    for i in range(drones_num):
        # Plot the lists on the same graph
        plt.plot(times, cum_avg_algo_SNR[i], label='Algorithm Positioning SNRs')
        plt.plot(times, cum_avg_static_SNR[i], label='Static Positioning SNRs')
        plt.plot(times, cum_avg_dynamic_SNR[i], label='Dynamic Route Positioning SNRs')

        # Add labels and title
        plt.xlabel('Time[sec]')
        plt.ylabel('SNR[DB]')
        plt.title('Average SNR Comparison')

        # Add a legend
        plt.legend()

        # Display the graph
        plt.show()
    print("finish")

    #MERAV: static: where DTM is highest, in the area where the group is supposed to be

def get_drone_static_location(surf,unit):
    x_comm, y_comm = unit_module.unit_module.getCommanderLoc(unit)
    unit_block_num = (x_comm//100)+50*(y_comm//100)
    neighbor_blocks = [unit_block_num]
    is_upper_row = False
    is_lower_row = False
    is_right_column = False
    is_left_column = False
    if (0<=unit_block_num<=49):
        is_upper_row = True
    if (2450<=unit_block_num<=2499):
        is_lower_row = True
    if ((unit_block_num%50)==0):
        is_left_column = True
    if ((unit_block_num%50)==49):
        is_right_column = True
    if not is_upper_row:
        #need to add upper neighbors
        neighbor_blocks.append(unit_block_num-50)
        if not is_left_column:
            neighbor_blocks.append(unit_block_num-51)
        if not is_right_column:
            neighbor_blocks.append(unit_block_num-49)
    if not is_lower_row:
        #need to add upper neighbors
        neighbor_blocks.append(unit_block_num+50)
        if not is_left_column:
            neighbor_blocks.append(unit_block_num+49)
        if not is_right_column:
            neighbor_blocks.append(unit_block_num+51)
    if not is_left_column:
        neighbor_blocks.append(unit_block_num-1)
    if not is_right_column:
        neighbor_blocks.append(unit_block_num+1)
    dtm = surface.surface.getDTM(surf)
    max_dtms = []
    highest_points = []
    for i in range(len(neighbor_blocks)):
        block_num = neighbor_blocks[i]
        block_init_x = 100*(block_num%50)
        block_init_y = 100*(block_num//50)
        block_dtm = dtm[block_init_x:block_init_x+block_size-1,block_init_y:block_init_y+block_size-1]
        max_dtm = np.max(block_dtm)
        highest_point_block_from_top = np.argmax(block_dtm)
        highest_point_relative_to_block_ind = (highest_point_block_from_top%block_size,highest_point_block_from_top//block_size)
        highest_point_block = (highest_point_relative_to_block_ind[0]+block_init_x,highest_point_relative_to_block_ind[1]+block_init_y)
        max_dtms.append(max_dtm)
        highest_points.append(highest_point_block)
    highest_block = np.argmax(np.array(max_dtms))
    return highest_points[highest_block]


def create_elliptic_route(x0, y0, a, b, theta=0):
    perimeter_pixels = []
    width, height = (map_size,map_size)
    
    # Convert degrees to radians for trigonometric functions
    theta_rad = np.deg2rad(theta)
    
    # Calculate the number of points along the perimeter
    num_points = max(width, height)
    
    # Calculate the angle step
    angle_step = 2 * np.pi / num_points
    
    # Iterate over the angles
    for i in range(0,num_points,20):
        # Calculate the angle
        angle = i * angle_step
        
        # Calculate the x and y coordinates of the point on the perimeter
        x = x0 + a * np.cos(angle) * np.cos(theta_rad) - b * np.sin(angle) * np.sin(theta_rad)
        y = y0 + a * np.cos(angle) * np.sin(theta_rad) + b * np.sin(angle) * np.cos(theta_rad)
        
        # Round the coordinates to the nearest integers
        x = int(round(x))
        y = int(round(y))
        
        # Check if the coordinates are within the matrix bounds
        if 0 <= x < width and 0 <= y < height:
            perimeter_pixels.append((x, y))
    
    return perimeter_pixels

def adjust_elliptic_param(comm_loc):
            #if we're in a corner, theta would be zero in any case
    dyn_ell_cent = comm_loc
    theta = random.randint(0,180)
    if (map_size-ellipse_b<=dyn_ell_cent[0]<map_size):
        theta = 90
        dyn_ell_cent[0] = map_size-ellipse_b-10 #we're taking it a little bit more inside
    elif (0<=dyn_ell_cent[0]<=ellipse_b):
        theta = 90
        dyn_ell_cent[0] = ellipse_b+10 #we're taking it a little bit more inside
    if (0<=dyn_ell_cent[1]<=ellipse_a):
        theta = 0
        dyn_ell_cent[1] = ellipse_a+10 #we're taking it a little bit more inside
    elif (map_size-ellipse_a<=dyn_ell_cent[1]<map_size):
        theta = 0
        dyn_ell_cent[1] = map_size-ellipse_a-10 #we're taking it a little bit more inside
    
    return dyn_ell_cent[0], dyn_ell_cent[1], theta


def has_nan_value(lst):
    for sublist in lst:
        for i in sublist:
            if np.isnan(i):
                print("There is a NaS value.")
                return True
    #print("There is no NaS value.")
    return False




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


