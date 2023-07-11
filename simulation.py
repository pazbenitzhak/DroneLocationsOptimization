
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
import timeout_decorator
#dtm_1m_utm18_e_10_104.tif
#run command: python simulation.py dtm_data.npy 1 5 1 200
#run command: python3.11 simulation.py
# cd /Users/raipheterson/Documents/year4/DroneLocationsOptimization
#u.e. spec: https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=2507

#all distances are in meters
time_interval = 1 #define longevity of 1 interval of delta_t
drone_threshold = 50 #define height of the drone, in meters. must be at least 50
soldier_threshold = 0.5 #must be 0.5
soldier_bw = 10 #must be 10
noise = 9 # according to specification
num_of_solds_in_unit = 10 #define number of soldiers per unit
max_dist_from_comm = 100 #define max distance of soldiers from commander in meters
drone_bw = 10*10**6 ##10MHZ in specification (article), in Hz
ptx = 0.2511886432 #24 DBm, 0.2511886432 Watt, according to article
initial_drone_freq = 2*10**9 #2000MHz/2GHz
map_size = 5000 #must be 5000
ellipse_a = 100 #define dynamic route allipse a axis
ellipse_b = 200 #define dynamic route allipse b axis
block_size = 100 #must be 100
communication_th = 8 #define snr_threshold
number_of_runs = 100 #define number of simulations
timeout = 100 #limit long runs
end_time = 300 #define number of iterations per run

def main():
    dtm_path = "dtm_data.npy"
    surface_obj = surface.surface(dtm_path, soldier_threshold, drone_threshold)
    units_num = 1
    drones_num = 1
    battery = [i for i in range(18,101)]
    num_of_timeouts = 0
    soldiers_in_unit_num = [num_of_solds_in_unit for i in range(number_of_runs)]
    algo_avg_snr, stat_avg_snr, dyn_avg_snr, algo_cnt, stat_cnt, dyn_cnt = [], [], [], 0, 0, 0
    algo_std, static_std, dyn_std = [], [], []
    for i in range(number_of_runs):
        print("before run num: " + str(i))
        try:        
            algo_avg_snr_i, stat_avg_snr_i, dyn_avg_snr_i, algo_cnt_i, stat_cnt_i, dyn_cnt_i, std_data_longest_inconnectivity_algo, std_data_longest_inconnectivity_static ,std_data_longest_inconnectivity_dyn  \
                = sim(surface_obj,units_num,soldiers_in_unit_num[i],drones_num,end_time,battery[i%len(battery)]/100)
        except timeout_decorator.TimeoutError:
            print("Function execution timed out")
            num_of_timeouts += 1
            continue
        algo_avg_snr.append(algo_avg_snr_i)
        stat_avg_snr.append(stat_avg_snr_i)
        dyn_avg_snr.append(dyn_avg_snr_i)
        algo_cnt += algo_cnt_i
        stat_cnt += stat_cnt_i
        dyn_cnt += dyn_cnt_i
        algo_std.extend(std_data_longest_inconnectivity_algo)
        static_std.extend(std_data_longest_inconnectivity_static)
        dyn_std.extend(std_data_longest_inconnectivity_dyn)
        print("after run num: " + str(i))
    algo_success_rate = round((algo_cnt/((soldiers_in_unit_num[0]+1)*(number_of_runs-num_of_timeouts)*end_time))*100,2)
    stat_success_rate = round((stat_cnt/((soldiers_in_unit_num[0]+1)*(number_of_runs-num_of_timeouts)*end_time))*100,2)
    dyn_success_rate = round((dyn_cnt/((soldiers_in_unit_num[0]+1)*(number_of_runs-num_of_timeouts)*end_time))*100,2)
    colors =['tab:blue', '#FF8C00', 'forestgreen']
    num_bins = 11
    fig,ax =plt.subplots()
    ax.hist([algo_avg_snr,stat_avg_snr,dyn_avg_snr], bins = num_bins, range=(-5, 50), density=True, histtype='bar',\
                             label=['Algorithm Positioning','Static Positioning','Dynamic Route'], color=colors, zorder=1)
    
    # Set labels and title
    ax.set_xlabel('SNR[dB]')
    ax.set_ylabel('Proportion')
    ax.set_title('SNR Histogram Comparison')

    # Display legend
    ax.legend()
    ax.grid(True, linestyle='--',color='gray',alpha=0.1,zorder=0)
    plt.show()

    fig,ax =plt.subplots()
    ax.hist([algo_std,static_std,dyn_std], bins = 10, range=(0, 50), density=True, histtype='bar',\
                            label=['Algorithm Positioning','Static Positioning','Dynamic Route'], color=colors, zorder=1)
    ax.set_xlabel('Time[sec]')
    ax.set_ylabel('Proportion')
    ax.set_title('Longest Inconnectivity Histogram Comparison')

    # Display legend
    ax.legend()
    ax.grid(True, linestyle='--',color='gray',alpha=0.1,zorder=0)
    plt.show()

    # Three percentage values
    percentages = [algo_success_rate, stat_success_rate, dyn_success_rate]

    # Labels for the bars
    labels = ['Algorithm Positioning', 'Static Positioning', 'Dynamic Route']

    # Create a bar chart
    plt.bar(labels, percentages, color = colors)

    # Set labels and title
    plt.ylabel('Percentage[%]')
    plt.title('Efficiency Comparison')

    plt.tight_layout()
    plt.show()

    print("num of timed out runs: "+ str(num_of_timeouts))
    return



@timeout_decorator.timeout(timeout)
def sim(surface_obj, units_num, soldiers_in_unit_num, drones_num, end_time, battery):

    time = time_module.time_module(0,time_interval)
    #------------------------------------------
    orig_img_write = plt.imread("lines.png")
    orig_img_write = np.round(orig_img_write * 255).astype(np.uint8)
    copy_img = np.copy(orig_img_write)
    #------------------------------------------
    #init units
    units = [unit_module.unit_module([], surface_obj) for i in range(int(units_num))]
    #init soldiers in units
    block = surface.surface.getBlocks(surface_obj)
    for unit in units:
        comm_x, comm_y = unit_module.unit_module.setCommanderLoc(unit, time)
        comm_x_init, comm_y_init = comm_x, comm_y
        #------------------------------------------
        copy_img[comm_x,comm_y] = [255,165,0] #orange
        #------------------------------------------
        units_soldiers = unit_module.unit_module.getSoldiers(unit)
        for i in range(int(soldiers_in_unit_num)):
            sold = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
            s_x, s_y = soldier.soldier.getLocation(sold)
            #------------------------------------------
            if i==0:
                copy_img[s_x,s_y] = [143,188,143] #dark sea green
                if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                    for j in range(-2,3):
                        copy_img[s_x-2,s_y+j] = [143,188,143]
                        copy_img[s_x+2,s_y+j] = [143,188,143]
                        copy_img[s_x+j,s_y-2] = [143,188,143]
                        copy_img[s_x+j,s_y+2] = [143,188,143]
            if i==1:
                copy_img[s_x,s_y] = [255,0,0] #red
                if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                    for j in range(-2,3):
                        copy_img[s_x-2,s_y+j] = [255,0,0]
                        copy_img[s_x+2,s_y+j] = [255,0,0]
                        copy_img[s_x+j,s_y-2] = [255,0,0]
                        copy_img[s_x+j,s_y+2] = [255,0,0]
            if i==2:
                copy_img[s_x,s_y] = [0,0,255] #blue
                if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                    for j in range(-2,3):
                        copy_img[s_x-2,s_y+j] = [0,0,255]
                        copy_img[s_x+2,s_y+j] = [0,0,255]
                        copy_img[s_x+j,s_y-2] = [0,0,255]
                        copy_img[s_x+j,s_y+2] = [0,0,255]
            if i==3:
                copy_img[s_x,s_y] = [0,128,0] #green
                if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                    for j in range(-2,3):
                        copy_img[s_x-2,s_y+j] = [0,128,0]
                        copy_img[s_x+2,s_y+j] = [0,128,0]
                        copy_img[s_x+j,s_y-2] = [0,128,0]
                        copy_img[s_x+j,s_y+2] = [0,128,0]
            if i==4:
                copy_img[s_x,s_y] = [184,134,11] #dark golden rod
                if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                    for j in range(-2,3):
                        copy_img[s_x-2,s_y+j] = [184,134,11]
                        copy_img[s_x+2,s_y+j] = [184,134,11]
                        copy_img[s_x+j,s_y-2] = [184,134,11]
                        copy_img[s_x+j,s_y+2] = [184,134,11]

            #------------------------------------------
            units_soldiers.append(sold)
    drones_list = []
    drones_static_locations = []
    drones_dynamic_routes = []
    for i in range(drones_num):
        unit_xy = unit_module.unit_module.getCommanderLoc(units[i])
        unit_x, unit_y = unit_xy
        #create static 'drone'
        drones_static_locations.append(get_drone_static_location(surface_obj, units[i]))
        drone_initial_loc = drones_static_locations[i]
        drones_list.append(drones.drone(drone_initial_loc[0], drone_initial_loc[1], drone_bw, [units[i]],\
                                         surface_obj, ptx, initial_drone_freq,battery))
        #create a dynamic elliptic route
        ellip_cent_x, ellip_cent_y, theta = adjust_elliptic_param([unit_x,unit_y])
        drones_dynamic_routes.append(create_elliptic_route(ellip_cent_x, ellip_cent_y, ellipse_a, ellipse_b, theta))
    curr_time = time_module.time_module.getTime(time)
    cum_avg_algo_SNR = [[] for i in range(drones_num)]
    cum_avg_static_SNR = [[] for i in range(drones_num)]
    cum_avg_dynamic_SNR = [[] for i in range(drones_num)]
    algo_cnt_i = 0
    stat_cnt_i = 0
    dyn_cnt_i = 0
    collect_longest_inconnectivity_algo = [0 for i in range(soldiers_in_unit_num+1)]
    collect_longest_inconnectivity_static = [0 for i in range(soldiers_in_unit_num+1)]
    collect_longest_inconnectivity_dyn = [0 for i in range(soldiers_in_unit_num+1)]
    
    std_data_longest_inconnectivity_algo = [0 for i in range(soldiers_in_unit_num+1)]
    std_data_longest_inconnectivity_static = [0 for i in range(soldiers_in_unit_num+1)]
    std_data_longest_inconnectivity_dyn = [0 for i in range(soldiers_in_unit_num+1)]

    while(curr_time<int(end_time)):
        for i in range(drones_num):
            #update battery
            drones.drone.updateBattery(drones_list[i])
            algo_SNR_i = 0
            static_SNR_i = 0
            dynamic_SNR_i = 0
            drone_loc = drones.drone.getLocation(drones_list[i])
            unit_i_solds = unit_module.unit_module.getSoldiers(units[i])
            sold_num = unit_module.unit_module.getSoldiersNum(units[i])+1
            #create commander object for SNR calculations
            comm_x, comm_y = unit_module.unit_module.getCommanderLoc(unit)
            if curr_time%10 == 0:
                comm_x_init, comm_y_init = comm_x, comm_y               
            commander = soldier.soldier(soldier_bw,noise,unit,max_dist_from_comm)
            soldier.soldier.setNewLocation(commander,comm_x, comm_y)
            #calculate SNR for commander
            algo_snr_i_comm = drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drone_loc)
            if algo_snr_i_comm>=communication_th:
                algo_cnt_i+=1
            if algo_snr_i_comm<communication_th:
                collect_longest_inconnectivity_algo[0]+=1
            else:
                std_data_longest_inconnectivity_algo[0]=max(collect_longest_inconnectivity_algo[0],std_data_longest_inconnectivity_algo[0])
                collect_longest_inconnectivity_algo[0]=0
            static_snr_i_comm = drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drones_static_locations[i])
            if static_snr_i_comm>=communication_th:
                stat_cnt_i +=1
            if static_snr_i_comm<communication_th:
                collect_longest_inconnectivity_static[0]+=1
            else:
                std_data_longest_inconnectivity_static[0]=max(collect_longest_inconnectivity_static[0],std_data_longest_inconnectivity_static[0])
                collect_longest_inconnectivity_static[0]=0
            dyn_route_len = len(drones_dynamic_routes[i])
            dyn_snr_i_comm = drone_manage.collect_soldier_snr(commander,drones_list[i],surface_obj,drones_dynamic_routes[i][curr_time%dyn_route_len])
            if dyn_snr_i_comm>=communication_th:
                dyn_cnt_i +=1
            if dyn_snr_i_comm<communication_th:
                collect_longest_inconnectivity_dyn[0]+=1
            else:
                std_data_longest_inconnectivity_dyn[0]=max(collect_longest_inconnectivity_dyn[0],std_data_longest_inconnectivity_dyn[0])
                collect_longest_inconnectivity_dyn[0]=0    
            algo_SNR_i += algo_snr_i_comm
            static_SNR_i += static_snr_i_comm
            dynamic_SNR_i += dyn_snr_i_comm
            #calculate SNR for other soldiers
            for sold in unit_i_solds:
                sold_index=unit_i_solds.index(sold)+1
                algo_snr_i_sold = drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drone_loc)
                algo_SNR_i += algo_snr_i_sold
                if algo_snr_i_sold>=communication_th:
                    algo_cnt_i+=1
                if algo_snr_i_sold<communication_th:
                    collect_longest_inconnectivity_algo[sold_index]+=1
                else:
                    std_data_longest_inconnectivity_algo[sold_index]=max(collect_longest_inconnectivity_algo[sold_index],std_data_longest_inconnectivity_algo[sold_index])
                    collect_longest_inconnectivity_algo[sold_index]=0

                static_snr_i_sold = drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drones_static_locations[i])
                static_SNR_i += static_snr_i_sold
                if static_snr_i_sold>=communication_th:
                    stat_cnt_i +=1
                if static_snr_i_sold<communication_th:
                    collect_longest_inconnectivity_static[sold_index]+=1
                else:
                    std_data_longest_inconnectivity_static[sold_index]=max(collect_longest_inconnectivity_static[sold_index],std_data_longest_inconnectivity_static[sold_index])
                    collect_longest_inconnectivity_static[sold_index]=0
                dyn_route_len = len(drones_dynamic_routes[i])
                dyn_snr_i_sold = drone_manage.collect_soldier_snr(sold,drones_list[i],surface_obj,drones_dynamic_routes[i][curr_time%dyn_route_len])
                dynamic_SNR_i += dyn_snr_i_sold

                if dyn_snr_i_sold>=communication_th:
                    dyn_cnt_i +=1
                if dyn_snr_i_sold<communication_th:
                    collect_longest_inconnectivity_dyn[sold_index]+=1
                else:
                    std_data_longest_inconnectivity_dyn[sold_index]=max(collect_longest_inconnectivity_dyn[sold_index],std_data_longest_inconnectivity_dyn[sold_index])
                    collect_longest_inconnectivity_dyn[sold_index]=0   
            algo_SNR_i = algo_SNR_i/sold_num
            static_SNR_i = static_SNR_i/sold_num
            dynamic_SNR_i = dynamic_SNR_i/sold_num
            cum_avg_algo_SNR[i].append(algo_SNR_i)
            cum_avg_static_SNR[i].append(static_SNR_i)
            cum_avg_dynamic_SNR[i].append(dynamic_SNR_i)
        #update soldiers
        for unit in units:
            comm_x, comm_y = unit_module.unit_module.setCommanderLoc(unit, time)
            copy_img[comm_x,comm_y] = [255,165,0] #orange ------------- for image coloring
            for j in range(-2,3):
                copy_img[comm_x-2,comm_y+j] = [255,165,0]
                copy_img[comm_x+2,comm_y+j] = [255,165,0]
                copy_img[comm_x+j,comm_y-2] = [255,165,0]
                copy_img[comm_x+j,comm_y+2] = [255,165,0]
            unit_soldiers = unit_module.unit_module.getSoldiers(unit)
            i = 0
            for sold in unit_soldiers:
                s_x, s_y = soldier.soldier.updateNewLocation(sold,max_dist_from_comm,time)
                if i==0:
                    copy_img[s_x,s_y] = [143,188,143] #dark sea green
                    if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                        for j in range(-2,3):
                            copy_img[s_x-2,s_y+j] = [143,188,143]
                            copy_img[s_x+2,s_y+j] = [143,188,143]
                            copy_img[s_x+j,s_y-2] = [143,188,143]
                            copy_img[s_x+j,s_y+2] = [143,188,143]
                if i==1:
                    copy_img[s_x,s_y] = [255,0,0] #red
                    if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                        for j in range(-2,3):
                            copy_img[s_x-2,s_y+j] = [255,0,0]
                            copy_img[s_x+2,s_y+j] = [255,0,0]
                            copy_img[s_x+j,s_y-2] = [255,0,0]
                            copy_img[s_x+j,s_y+2] = [255,0,0]
                if i==2:
                    copy_img[s_x,s_y] = [0,0,255] #blue
                    if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                        for j in range(-2,3):
                            copy_img[s_x-2,s_y+j] = [0,0,255]
                            copy_img[s_x+2,s_y+j] = [0,0,255]
                            copy_img[s_x+j,s_y-2] = [0,0,255]
                            copy_img[s_x+j,s_y+2] = [0,0,255]
                if i==3:
                    copy_img[s_x,s_y] = [0,128,0] #green
                    if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                        for j in range(-2,3):
                            copy_img[s_x-2,s_y+j] = [0,128,0]
                            copy_img[s_x+2,s_y+j] = [0,128,0]
                            copy_img[s_x+j,s_y-2] = [0,128,0]
                            copy_img[s_x+j,s_y+2] = [0,128,0]
                if i==4:
                    copy_img[s_x,s_y] = [184,134,11] #dark golden rod
                    if (s_x < 4998 and s_y < 4998 and s_x > 1 and s_y > 1):
                        for j in range(-2,3):
                            copy_img[s_x-2,s_y+j] = [184,134,11]
                            copy_img[s_x+2,s_y+j] = [184,134,11]
                            copy_img[s_x+j,s_y-2] = [184,134,11]
                            copy_img[s_x+j,s_y+2] = [184,134,11]

                i+=1
        snr_param = []
        drones_locations = []
        drones_batteries = []
        for i in range(len(drones_list)):
            drone = drones_list[i]
            snrs, locations, battery = drone_manage.calculate_next_locs_data(units[i],drone,surface_obj)
            snr_param.append(snrs)
            drones_locations.append(locations)
            drones_batteries.append(battery)
        if has_nan_value(snrs):
            break
        #optimization algorithm
        locations = op.optimization_algorithm(snr_param,drones_locations,drones_batteries)
        #update drones (data from op algo)
        for ind in range(len(drones_list)):
            drone = drones_list[i]
            drone_x, drone_y = drones.drone.setNewLocation(drone,locations[ind][0],locations[ind][1], surface_obj)
            #------------------------------------------
            copy_img[drone_x,drone_y] = [255,0,255] #magenta
            if (drone_x < 4996 and drone_y < 4996 and drone_x > 3 and drone_y > 3):
                for j in range(2,4):
                    copy_img[drone_x+j,drone_y+j] = [255,0,255]
                    copy_img[drone_x-j,drone_y+j] = [255,0,255]
                    copy_img[drone_x+j,drone_y-j] = [255,0,255]
                    copy_img[drone_x-j,drone_y-j] = [255,0,255]
            #------------------------------------------
        # This section needs to be uncommented in order to show progress of soldiers, up to 5 soldiers in unit + commander
            #------------------------------------------
        #if (curr_time%50==0 and curr_time != 0):
            # plt.imshow(copy_img[comm_x_init-100:comm_x_init+100,comm_y_init-100:comm_y_init+100])
            # plt.show()

        #copy_img = np.copy(orig_img_write)
            #------------------------------------------
        #increment time
        time_module.time_module.updateTime(time)
        curr_time = time_module.time_module.getTime(time)
    algo_avg_snr_i = np.mean(cum_avg_algo_SNR)
    stat_avg_snr_i = np.mean(cum_avg_static_SNR)
    dyn_avg_snr_i = np.mean(cum_avg_dynamic_SNR)
    # This section can be uncommented if a visualization of 1 run is wanted.
    # Unomment is NOT recommended if number_of_runs != 1
    #------------------------------------------
    # plt.imshow(copy_img)
    # plt.show()
    #------------------------------------------
    times = range(curr_time)
        
    return algo_avg_snr_i, stat_avg_snr_i, dyn_avg_snr_i, algo_cnt_i, stat_cnt_i, dyn_cnt_i,std_data_longest_inconnectivity_algo, std_data_longest_inconnectivity_static ,std_data_longest_inconnectivity_dyn 
     

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
                return True
    return False



if __name__ == "__main__":
    main()


