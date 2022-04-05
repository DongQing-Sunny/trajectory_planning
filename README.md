# Trajectory Planning
[This repo](https://github.com/DongQing-Sunny/trajectory_planning) contains the code for trajectory planning task.

The map and obstacles are initialized in  [`map.py`](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/map.py). 

The overall process of the trajectory planning is built in 
class `TrajectoryPlanning` in [`trajector_planning.py`](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/trajectory_planning.py). Run this file to start the trajectory planning task. The starting point, target point and map range can be specified. At present, there are `Astar, RRT, RRTstar and informed RRTstar` path planning algorithm, and other modules and other algorithm needed for trajectory planning task need to be supplemented.

The class for the path search is instantiated and the path planning process is built in funciton `do_path_planning`. Interfaces are set aside for applying other algorithms, which can be supplemented later.

Specifically, each grid in search-based algorithm is treated as a `Node` class defined in [node.py](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/node.py), and each position in sample-based algorithm is treated as a `Node` class. The specific implementation of each algorithm is in the corresponding funciton in [path_search.py](https://github.com/DongQing-Sunny/trajectory_planning/blob/master/path_planning.py). 

This is a basic version of the assignment implemented in python. The c++ version of the assignment is under development and is expected to be submitted in next week's iteration. Thank you for your guidance.