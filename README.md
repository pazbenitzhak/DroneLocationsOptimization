# Drone Locations Optimization
# Introduction
This project deals with an optimization algorithm for drones movement.
The situation includes a randomly-moving ground force in a predifened map of some kind of an urban area,
and the drone should move along to provide the best communication coverage for the force.
The ground force consists of several soldiers and transfers its communications through a single drone.

# Structure
The code could be found in 2 main directories inside the src directory:
1. **sim** - includes all the modules of the participating objects in the simulation: drones, soldiers and units.
   Other relevant modules that helped the creation of the simulation include the surface and the time.
   The module also include the simulation.py file, which is **the file needed to be run in order to execute the entire
   process** 
3. **alg_calc** - includes all modules that help collect the data for the algorithm (drone_manage), and all the modules
   which calculate the necessary data for the algorithm: current_propagation, wave_propagation.
   It also uncludes the module which executes the algorithm.
4. **utils** - includes the blocks module which calculates blocks in the map which a human can walk at.
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
   Python3.11 is recommend for running the code.

2. **Find a DTM layer**, in particular one in a resolution of 1m.
   Suitable such layers, in the format of a TIFF image, could be found in the **HRDEM** Canadian government database, which consists of different areas in canada and
   is available in the next link:
   https://open.canada.ca/data/en/dataset/957782bf-847c-4644-a757-e383c0057995
   Notice that you whould look for a TIFF image with the terms "1m" and "dtm" in its name.

4. **Load the DTM layer** - Open a python interpreter directory where the DTM layer TIFF image is saved  and do the following:

   ```import numpy as np
   ```
   ```
   np.save("dtm_data.npy", dir)
   ```

   
