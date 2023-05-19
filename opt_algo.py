import numpy as np

snr_th = 8 #DB
battery_th = 0.15 # %15, we want to be able to arrive to the charging point with this value
max_dist = 1000*np.sqrt(2) #maximum distance from a charging point that can be in the current configuration
seconds_to_pass_max_dist = max_dist/20
upper_battery_th = battery_th+(seconds_to_pass_max_dist*(1/10800)) # from this battery value we want the weight of the distances to be the deciding factor

#we get a list of lists of lists. each list consists of lists of SNR values
def optimization_algorithm(snrs_list,locations,batteries):
    #we need to define SNR thresholds for good, bad and no signal
    #according to the following source: https://www.telcoantennas.com.au/blog/guide-to-mobile-networks/4g-lte-signal-strength-reference-guide/
    #we see that:
    #SNR>=15DB -> excellent signal
    #SNR>=10DB -> good signal
    #SNR>=3DB -> fair to poor signal
    #SNR<=0DB -> no signal
    #we set the threshold to be at 8DB
    chosen_locations = []
    drones_num = len(snrs_list)
    for i in range(drones_num):
        drone_snrs = snrs_list[i]
        drone_pos_locations = locations[i]
        #check if there is a SNR value which indicates no signal - lower than 8DB
        snrs_th_list = []
        for option in drone_snrs:
            snrs_below_th = 0
            for snr in option:
                if snr<snr_th:
                    snrs_below_th+=1
            snrs_th_list.append(snrs_below_th)
        #now for all options, we have a list indicating how many snrs below threshold they have
        min_below_th = min(snrs_th_list)
        option_after_th = []
        optional_locations = []
        for j in range(len(drone_snrs)):
            if snrs_th_list[j] == min_below_th:
                option_after_th.append(drone_snrs[j])
                optional_locations.append(drone_pos_locations[j])
        optional_locations = np.array(optional_locations)
        #now we have all the options after maintaing only a minimal number of values below the threshold
        snr_weights = sigmoid(np.array(option_after_th))
        #need to average snr_weights
        snr_weights_av = np.array([np.mean(weight_list) for weight_list in snr_weights])
        closest_charge_point = np.array([find_closest_charge_point(x,y) for (x,y) in optional_locations])
        dist_from_charge_point = np.sqrt(np.square(optional_locations[:,0]-closest_charge_point[:,0])\
        +np.square(optional_locations[:,0]-closest_charge_point[:,0]))
        dist_weights = batt_weight_by_batt_usage(dist_from_charge_point,batteries[i])
        value_func = snr_weights_av+dist_weights
        max_val_index = np.argmax(value_func)
        chosen_point = optional_locations[max_val_index]
        chosen_locations.append(chosen_point)
    return chosen_locations

def sigmoid(x):
    mask_mat = (x>=snr_th).astype(int)
    x_tan = np.tanh((x-snr_th)/10)
    return mask_mat*x_tan

def find_closest_charge_point(x,y):
    x_charge_point = 0
    y_charge_point = 0
    if (x<500):
        x_charge_point = 1000
    if (x>=4500):
        x_charge_point = 4000
    else: 
        x_charge_point = round(x / 1000) * 1000
    if (y<500):
        y_charge_point = 1000
    if (y>=4500):
        y_charge_point = 4000
    else:
        y_charge_point = round(y / 1000) * 1000
    return (x_charge_point,y_charge_point)

def batt_weight_by_batt_usage(dist,batt):
    dist_copy = np.copy(dist)
    sec_to_reach_charg_point = dist_copy/20
    batt_consump_needed = sec_to_reach_charg_point/10800
    batt_th_added = battery_th+(5/10800)
    #if the condition is satisfied: give high weight to distance
    #10 chosen beacuase expected value for snrs' average s in the range of 0.1-0.9
    #If not: give low weight to distance so the algorithm will only look at the snrs
    batt_weights = (batt-batt_consump_needed<batt_th_added).astype(int)*10
    return batt_weights

def batt_weight_2():
    return
