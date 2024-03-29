# Drone Locations Optimization
# Introduction
This project deals with an optimization algorithm for drones movement.
The situation includes a randomly-moving ground force in a predifened map of some kind of an urban area,
and the drone should move along to provide the best communication coverage for the force.
The ground force consists of several soldiers and transfers its communications through a single drone.

# Structure
The code can be viewed in three parts:
1. **Simulation Modules** - includes all the modules of the participating objects in the simulation: drones, soldiers and units.
   Other relevant modules that helped the creation of the simulation include the surface and the time.
   It also includes the simulation.py file, which is **the file needed to be run in order to execute the entire
   process** 
3. **Algorithm Calculation Modules** - includes all modules that help collect the data for the algorithm (drone_manage), and all the modules
   which calculate the necessary data for the algorithm: current_propagation, wave_propagation.
   It also uncludes the module which executes the algorithm.
4. **Utils** - includes the blocks module which calculates blocks in the map which a human can walk at.
   This module is not in use in the current implementation of the project.

# The Algorithm
The algorithm considers 2 main factors:
1. **SNR (Signal to Noise Ration) values** - The SNR each soldier has with the drone.
2. **Battery** - The current battery the drone has.

The algorithm receives a list of SNR values the drone would have with each soldier in case of moving to each possible point.
The drone can advance within a varying distance of 5, 10 or 20 meters, and in 8 different directions (each direction is 45 degrees from its neighbors).
It can also stay in its current location. Therefore there are 25 possible points for advancing.
The algorithm has a list of 25 lists of SNR values (the smaller lists are in the length of the unit's size - number of soldiers + 1 for the commander).
The algorith then throws away all lists where the number of SNR values under the threshold is maximal (the threshold is set to 8 dB).

After that, the algorithm passes each list's SNR values in a weight function which returns a number between 0 and 1 and averages those values.
Another weight function is calculated according to the current battery and then summed to the first expression. In short, the second weight function
will be different than zero only if the drone won't have sufficient battery to get to the charging point (we set it to be th pixel (0,0)). In that case,
it returns values which would be around 15 (thus overshadowing the first weight function).

The algorithm returns the point which maximies the expressions' sum.

# How to use
1. **Clone the repository** to its local workstation.
   There is no need in build or downloading anything but the Python packages inside the files.
   Python3.11 is recommended for running the code.

2. **Find a DTM layer**, in particular one in a resolution of 1m.
   Suitable such layers, in the format of a TIFF image, could be found in the **HRDEM** Canadian government database, which consists of different areas in canada and
   is available in the next link:
   https://open.canada.ca/data/en/dataset/957782bf-847c-4644-a757-e383c0057995
   Notice that you whould look for a TIFF image with the terms "1m" and "dtm" in its name.

   Particularly, such files can be found easily here: https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/VILLE_MONTREAL/VILLE_MONTREAL/utm18/
   Choose a file that starts with "dtm_1m_utm18_e_...". A file of size over 200M and above is recommended.

4. **Load the DTM layer** - Open a python interpreter directory where the DTM layer TIFF image is saved and do the following:

   Import numpy:
   ```
   import numpy as np
   ```

   Save the DTM heights map to an npy file:
   
   ```
   np.save("dtm_data.npy", dir)
   ```

   Copy the npy file to the same directory where the simulation.py is in.

5. Run the code:
   ```
   python simulation.py
   ```

6. Get the results, which include an average SNRs histogram, an efficiency graph and a max duration of communication cut histogram.

7. To get a visualiation of the forces and drones movement, please refer to lines 370-390 in the simulation.py file.

**NOTE:** Right now the code is configured to run 100 runs of the simulation, with 10 soldiers in a unit
         and various battery values. In the current configuration, 100 runs are expected to run in approximatley 1hr, with a single run runtime of 35-40 seconds. You can configure that and other parameters in the first lines of the simulation.py file.

   
