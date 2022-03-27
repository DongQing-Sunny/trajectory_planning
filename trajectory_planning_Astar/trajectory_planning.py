from path_search import PathSearch
from node import Node
import numpy as np
import math
    
class TrajectoryPlanning:
    def __init__(self):
       return
   
    def do_planning(self, method_selected, x_start, y_start, x_target, y_target, map_max_x, map_max_y):
        map_all_node = self.init_map(map_max_x, map_max_y)
        
        pathSearch = PathSearch(method_selected)
        pathSearch.do_search(map_all_node, x_start, y_start, x_target, y_target)
   
    def init_map(self, max_x, max_y):
        all_node = []
        for i in range(0, max_x):
            for j in range(0, max_y):
                all_node.append(Node(i,j))
                    
        for i in range(0, len(all_node)):
            this_node = all_node[i]
                        
            if np.random.rand() > 0.75:
                this_node.set_block(1)
                
            for j in range(0, len(all_node)):
                that_node = all_node[j]
                if math.pow(this_node.x-that_node.x, 2) <= 1 and math.pow(this_node.y-that_node.y, 2) <= 1 and i != j:
                    this_node.set_neighbor(that_node)
        return all_node

if __name__ == '__main__':
    
    method_selected = 'Astar'
    x_start = 0
    y_start = 0
    x_target = 9
    y_target = 9
    map_max_x = 10
    map_max_y = 10 
    t = TrajectoryPlanning()
    t.do_planning(method_selected, x_start, y_start, x_target, y_target, map_max_x, map_max_y)


