import math
       
class My_Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = None
        self.g = None
        self.parent = None
        self.neighbors = []
        self.is_torched = 0
        self.is_block = 0
        self.isin_queue = 0
        
    def set_h(self, target_node):
        self.h = self.get_distance(target_node)            

    def set_block(self, is_block):
        self.is_block = is_block
            
    def set_neighbor(self, neighbor):
        self.neighbors.append(neighbor)
                
    def set_parent(self, parent):
        self.parent = parent
        self.g = parent.g + self.get_distance(parent)
        
    def set_g(self, g):
        self.g = g
                
    def get_distance(self, other_node):
        return math.sqrt(math.pow(self.x - other_node.x, 2) + math.pow(self.y - other_node.y, 2))
                    
    def set_torched(self):
        self.is_torched = 1
    
    def set_isin_queue(self):
        self.isin_queue = 1
        

    
