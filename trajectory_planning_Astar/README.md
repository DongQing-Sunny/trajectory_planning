# Trajectory-Planning
This repo contains the code for trajectory planning task.

The overall process of the trajectory planning is built in 
class `TrajectoryPlanning` in `trajector_planning.py`. Run this file to start the method. The starting point, target point and map range can be specified.

First, the map and obstacles are initialized in function `init_map`. Actually, the map should be specified before the trajectory planning module. For simplicity, the map is initialized here.

Then, the class for the path search is instantiated and the path search process is built in funciton `do_search`. Interfaces are set aside for applying other algorithms, which can be supplemented later.

Specifically, each grid is treated as a Node class, and the Astar algorithm is used for path search in funciton `astar_search`. 

Finally, the map and the searched path are visualized.