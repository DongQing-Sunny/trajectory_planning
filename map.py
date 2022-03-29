import numpy as np
import math
from node import Node
from PIL import Image

class Map:
    def __init__(self, map_selected, max_x=0, max_y=0, image=None):
        self.map_selected = map_selected
        self.max_x = max_x
        self.max_y = max_y
        self.image = image
        
    def create_map(self):
        if self.map_selected == 'random_grid_map':
            return self.random_grid_map(self.max_x, self.max_y)
        elif self.map_selected == 'image_map':
            return self.image_map(self.image)
    
    def random_grid_map(self, max_x, max_y):
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
    
    def image_map(self, image):
        img=Image.open(image)
        img=img.convert('L')
        img=np.array(img)
        
        return img
        
    