# Vehicle Routing Problem

This is a solution to a variation of the vehicle routing problem. This goal of 
this variation is to plan delivery routes such that both the number of vehicles 
dispatched and the time traveled by any given vehicle are minimized. There are 
no constraints on the number of drivers/vehicles, but a vehicle can only travel 
for 12 hours in a day, and must start and end at the depot (origin).


## Assumptions

Functionally, time spent traveling is equal to Euclidean distance traveled, 
therefore the time constraint is effectively a distance constraint.

The pickup->delivery step can also be condensed into a single weighted node, since 
it is assumed a driver can only hold one load at a time, hence anything other than 
going straight from pickup to delivery would be inherently sub-optimal due to the 
usage of Euclidean distance.


## Constraints

Distance/time traveled by a vehicle must be less than or equal to 720.


## Cost Function

total_cost = 500*number_of_drivers + total_number_of_driven_minutes


## Algorithm

The nearest neighbor greedy algorithm was used. This method involves 
filling paths by connecting the closest node to the leaf node until no 
more nodes except the depot can be added, which is when the path ends.


## Running

This program was developed with Clang and CMake. To build the program, run the build.sh
script. 

An executable "Main" will be deposited into the build directory. This will read all the problem file provided via command line argument and print a solution.

If not via build script, run these commands.
- `mkdir build`
- `cd build`
- `cmake ..`
- `make`
