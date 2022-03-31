import numpy as np
import math
from node import My_Node
from PIL import Image

class My_Map:
    def __init__(self, map_selected, max_x=0, max_y=0, image=None):
        self.map_selected = map_selected
        self.max_x = max_x
        self.max_y = max_y
        self.image = image
        
    def create_map(self):
        if self.map_selected == 'random_grid_map':
            return self.random_grid_map(self.max_x, self.max_y)
        elif self.map_selected == 'image_map':
            return self.image_map()
    
    def random_grid_map(self, max_x, max_y):
        all_node = []
        for i in range(0, max_x):
            for j in range(0, max_y):
                all_node.append(My_Node(i,j))
                    
        for i in range(0, len(all_node)):
            this_node = all_node[i]
                        
            if np.random.rand() > 0.75:
                this_node.set_block(1)
                
            for j in range(0, len(all_node)):
                that_node = all_node[j]
                if math.pow(this_node.x-that_node.x, 2) <= 1 and math.pow(this_node.y-that_node.y, 2) <= 1 and i != j:
                    this_node.set_neighbor(that_node)
        return all_node
    
    def image_map(self):
        x1=89
        x2=343
        y1=306
        y2=459
        x3=648
        x4=741
        y3=234
        y4=593
        obstacle = []       
        obstacle.append(Obstacle(x1, x2, y1, y2))
        obstacle.append(Obstacle(x3, x4, y3, y4))
        
        return obstacle

class Obstacle:
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2        
    