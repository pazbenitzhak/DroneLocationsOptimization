import wave_propagation as wp
import soldier as soldier
import surface as surface
import unit_module




def calculate_next_locs_data(unit,drone,surf):
    unit_snrs = []
    soldiers = unit_module.unit_module.getSoldiers(unit)
    possible_drone_locations = get_poss_drone_locs(drone,surf)
    for sold in soldier:
        unit_snrs.extends(collect_soldier_snr(sold,drone,possible_drone_locations))



def collect_soldier_snr(sold,drone,points):
    return