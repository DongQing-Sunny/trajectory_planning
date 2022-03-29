import numpy as np
import matplotlib.pyplot as plt
import queue
    
class PathPlanning:
    def __init__(self, algo_select):
        self.algo_select = algo_select
        return

    def do_path_planning(self, all_node, x_start, y_start, x_target, y_target):
        if self.algo_select == 'Astar':
            return self.astar_search(all_node, x_start, y_start, x_target, y_target)
        #TODO：other path search method
        
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

        # Index as a tie breaker, in case when h and g are both equal
        idx = 0
        
        selected_node = queue.PriorityQueue()
        selected_node.put(((start_node.g + start_node.h, -start_node.g, idx), start_node))
        start_node.set_torched()
        
        done = False
        
        while (not selected_node.empty()) and (not done):
            current_node = selected_node.get()[-1]
            for candidate in current_node.neighbors:
                if candidate.is_block == 1 or candidate.is_torched == 1:
                    continue 
                idx += 1
                candidate_g = current_node.g + current_node.get_distance(candidate)
                candidate_fn = candidate_g + candidate.h

                candidate.set_parent(current_node)
                candidate.set_g(current_node)
                candidate.set_torched()
                if candidate == target_node:
                    done = True
                    break
                assert isinstance(candidate_fn, float)
                selected_node.put(((candidate_fn, -candidate.g, idx), candidate))

        if not done:
            print('No path found!')
            return
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
        