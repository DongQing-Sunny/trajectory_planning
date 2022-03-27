import numpy as np
import math

class ObstacleMap:
    def __init__(self, x_start, y_start, x_target, y_target, max_x, max_y, obstacle_ratio):
        self.x_start = x_start
        self.y_start = y_start
        self.x_target = x_target
        self.y_target = y_target        
        self.max_x = max_x
        self.max_y = max_y
        self.obstacle_ratio = obstacle_ratio
        
    def creata_obstacles(self):
        obstacle_map = []
        rand_map = np.random.rand(self.max_x, self.max_y)
        for i in range(0, self.max_x):
            for j in range(0, self.max_y):
                if rand_map[i,j]<self.obstacle_ratio:
                    obstacle_map.append([i,j])
        return obstacle_map
          
    def create_map(self):
        whole_map = []
        whole_map.append([self.x_start, self.y_start])
        whole_map.extend(self.creata_obstacles())
        whole_map.append([self.x_target, self.y_target])
        return whole_map
        
        
def node_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))

def insert_open(exp_node, x_node, y_node):
    new_node = []
    new_node.insert(0,1)
    new_node.insert(1,exp_node[0])
    new_node.insert(2,exp_node[1])
    new_node.insert(3,x_node)
    new_node.insert(4,y_node)
    new_node.insert(5,exp_node[2])
    new_node.insert(6,exp_node[3])
    new_node.insert(7,exp_node[4])
    return new_node

def node_indx(open_list, x_val, y_val):
    n_index = -1
    for i in range(0, len(open_list)):
        if open_list[i][1] == x_val and open_list[i][2] == y_val:
            n_index = i
    
    return n_index

def is_in_closed_list(closed_list, x_val, y_val):
    is_in = False
    for i in range(0,len(closed_list)):
        if closed_list[i][0] == x_val and closed_list[i][1] == y_val:
            is_in = True
    return is_in

def hn_Astar(xTarget, yTarget, exp_x, exp_y):
    return node_distance(xTarget, yTarget, exp_x, exp_y)

def expand_list(node_x, node_y, gn, xTarget, yTarget, closed_list, MAX_X, MAX_Y):
    expand_list = []
    list_order = [1,0,-1]
    for i in list_order:
        for j in list_order:
            expand_node = []
            if i==0 and j==0:
                continue
            exp_x = node_x + i
            exp_y = node_y + j
            if exp_x >=0 and exp_x < MAX_X and exp_y >=0 and exp_y < MAX_Y and (not is_in_closed_list(closed_list, exp_x, exp_y)):
                expand_node.insert(0,exp_x)
                expand_node.insert(1,exp_y)
                expand_node.insert(2,hn_Astar(xTarget, yTarget, exp_x, exp_y)) #hn
                expand_node.insert(3,gn+node_distance(node_x, node_x, exp_x, exp_y)) #gn
                expand_node.insert(4,expand_node[2]+expand_node[3]) #fn
                expand_list.append(expand_node)
            
    return expand_list

def min_fn(open_list):
    fn = 1000
    for i in range(0, len(open_list)):
        if open_list[i][0] == 0:
            continue
        if open_list[i][-1] < fn:
            fn = open_list[i][-1]
            node_selected_index = i
            node_selected = open_list[i]
        open_list[node_selected_index][0] = 0
            
    return node_selected