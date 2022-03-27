# Trajectory Planning
[This repo](https://github.com/DongQing-Sunny/trajectory_planning) contains the code for trajectory planning task.

The overall process of the trajectory planning is built in 
class `TrajectoryPlanning` in [`trajector_planning.py`](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/trajectory_planning.py). Run this file to start the trajectory planning task. The starting point, target point and map range can be specified. At present, there is only Astar path finding algorithm in the framework, and other modules and other algorithm needed for trajectory planning task need to be supplemented.

First, the map and obstacles are initialized in function `init_map`. Actually, the map should be specified before the trajectory planning module. For simplicity, the map is initialized here.

Then, the class for the path search is instantiated and the path search process is built in funciton `do_search`. Interfaces are set aside for applying other algorithms, which can be supplemented later.

Specifically, each grid is treated as a `Node` class defined in [node.py](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/node.py), and the Astar algorithm is used for path search in funciton `astar_search` in [path_search.py](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/path_search.py). 

Finally, the map and the searched path are visualized.

This is a basic version of the assignment implemented in python. The c++ version of the assignment is under development and is expected to be submitted in next week's iteration.
