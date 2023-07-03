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

# How to use
First, you must clone the repository to its local workstation.
There is no need in build or downloading anything but the Python packages inside the files.
Python3.11 is recommend for running the code.

The next step is putting
