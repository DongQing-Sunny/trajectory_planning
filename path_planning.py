from turtle import done
from xml.sax.handler import all_properties
import matplotlib.pyplot as plt
    
class PathPlanning:
    def __init__(self, algo_select):
        self.algo_select = algo_select
        return

    def do_path_planning(self, map, x_start, y_start, x_target, y_target):
        
        if self.algo_select == 'Astar':
            return self.astar_search(map, x_start, y_start, x_target, y_target)
        if self.algo_select == 'RRT':
            return self.RRT_sampling(map, x_start, y_start, x_target, y_target)
    
    def RRT_sampling(self, x_start, y_start, x_target, y_target):  
        pass

    # def BFS_search(self, all_node, x_start, y_start, x_target, y_target):
    #     plt.scatter(x_start, y_start, color='blue')
    #     plt.scatter(x_target, y_target, color='blue')
        
    #     plt.scatter(x_start, y_start, color='blue')
    #     plt.scatter(x_target, y_target, color='blue')

    #     for i in range(0, len(all_node)):
    #         if all_node[i].x == x_start and all_node[i].y == y_start:
    #             all_node[i].set_block(0)
    #             start_node = all_node[i]
    #         if all_node[i].x == x_target and all_node[i].y == y_target:
    #             all_node[i].set_block(0)
    #             target_node = all_node[i]

    #     for i in range(0, len(all_node)):
    #         if all_node[i].is_block == 1:       
    #             plt.scatter(all_node[i].x, all_node[i].y, color='black') 
        
    #     selected_node = [start_node]
    #     start_node.set_torched()                
        
    #     while (not selected_node.empty()) and (not done):
    #         for candi in selected_node
            
            
    def astar_search(self, all_node, x_start, y_start, x_target, y_target):
        plt.scatter(x_start, y_start, color='blue')
        plt.scatter(x_target, y_target, color='blue')

        for i in range(0, len(all_node)):
            if all_node[i].x == x_start and all_node[i].y == y_start:
                all_node[i].set_block(0)
                start_node = all_node[i]
            if all_node[i].x == x_target and all_node[i].y == y_target:
                all_node[i].set_block(0)
                target_node = all_node[i]

        for i in range(0, len(all_node)):
            if all_node[i].is_block == 1:       
                plt.scatter(all_node[i].x, all_node[i].y, color='black')    

        selected_node = [start_node]
        wait_to_see_node = [start_node]
        start_node.set_torched()

        while 1:
            min_fn = 100000

            for this_node in wait_to_see_node: 
                count = 0 
                for candidate_son in this_node.neighbors:
                    if candidate_son.is_block == 1 or candidate_son.is_torched == 1:
                        count +=1
                        continue 
                    print('count='+str(count))
                    print('num_neighbors='+str(len(this_node.neighbors)))
                    if count == len(this_node.neighbors):
                        print(1)
                        wait_to_see_node.remove(this_node)

                    candidate_son_g = this_node.g + this_node.get_distance(candidate_son)
                    candidate_son_fn = candidate_son_g + candidate_son.h
                    if candidate_son_fn < min_fn:
                        min_son = candidate_son
                        min_parent = this_node
                        min_fn = candidate_son_fn 
              
            try:
                min_son
            except:
                print('The agent or the target is surrounded by obstacles and there is no viable path.')
                return
            selected_node.append(min_son)
            wait_to_see_node.append(min_son)
            min_son.set_parent(min_parent)
            min_son.set_g(min_parent)
            min_son.set_torched()
            if min_son == target_node:
                break
            
        son_node = target_node

        while 1:
            plt.scatter(son_node.parent.x, son_node.parent.y, color='red')
            son_node = son_node.parent
            if son_node.parent == start_node:
                break
                        
        ax=plt.gca()
        ax.xaxis.set_major_locator(plt.MultipleLocator(1))
        ax.yaxis.set_major_locator(plt.MultipleLocator(1))       
        plt.xlim(-0.5, 9.5)    
        plt.ylim(-0.5, 9.5)   
        plt.show()  
        