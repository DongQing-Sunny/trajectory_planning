from path_planning import PathPlanning
from map import My_Map

    
class TrajectoryPlanning:
    def __init__(self):
       return
   
    def do_planning(self, method_selected, map, x_start, y_start, x_target, y_target):
     
        pathSearch = PathPlanning(method_selected)
        pathSearch.do_path_planning(map, x_start, y_start, x_target, y_target)
        
        ##TODO: trajectory generation
        

if __name__ == '__main__':
    
    #method_selected = 'Astar'
    method_selected = 'DFS'
    #method_selected = 'BFS'
    map_selected = 'random_grid_map'
    #map_selected = 'image_map'
    
    x_start = 0
    y_start = 0
    x_target = 9
    y_target = 9
    map_max_x = 10
    map_max_y = 10 
    
    map = My_Map(map_selected, map_max_x, map_max_y)
    #map = Map(map_selected, image='newmap.png')
    map = map.create_map()
    
    t = TrajectoryPlanning()
    t.do_planning(method_selected, map, x_start, y_start, x_target, y_target)


