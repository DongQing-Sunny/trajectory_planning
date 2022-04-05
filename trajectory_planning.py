from path_planning import PathPlanning
from map import My_Map

    
class TrajectoryPlanning:
    def __init__(self):
       return
   
    def do_planning(self, method_selected, map, sample_method, x_start, y_start, x_target, y_target):
     
        pathSearch = PathPlanning(method_selected)
        pathSearch.do_path_planning(map, sample_method, x_start, y_start, x_target, y_target)
        
        ##TODO: trajectory generation
        

if __name__ == '__main__':
    
    #method_selected = 'Astar'
    #method_selected = 'DFS'
    #method_selected = 'BFS'
    #method_selected = 'RRT'
    method_selected = 'RRTstar'
    
    #sample_method = 'random'
    sample_method = 'ellipse_informed'
    
    #map_selected = 'random_grid_map'
    map_selected = 'image_map'
    
    x_start = 100
    y_start = 200
    x_target = 700
    y_target = 700
    map_max_x = 10
    map_max_y = 10 
    
    #map = My_Map(map_selected, map_max_x, map_max_y)
    map = My_Map(map_selected)
    map = map.create_map()
    
    t = TrajectoryPlanning()
    t.do_planning(method_selected, map, sample_method, x_start, y_start, x_target, y_target)


