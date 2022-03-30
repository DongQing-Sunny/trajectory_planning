import queue
import matplotlib.pyplot as plt
    
class PathPlanning:
    def __init__(self, algo_select):
        self.algo_select = algo_select
        return

    def do_path_planning(self, map, x_start, y_start, x_target, y_target):
        
        if self.algo_select == 'Astar':
            return self.astar_search(map, x_start, y_start, x_target, y_target, data_structure='priority')
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
    #         current_node = selected_node.get
    #         for candi in selected_node
          
            
    def astar_search(self, all_node, x_start, y_start, x_target, y_target, data_structure='list'):

        for i in range(0, len(all_node)):
            
            if all_node[i].x == x_start and all_node[i].y == y_start:
                all_node[i].set_block(0)
                start_node = all_node[i]
            if all_node[i].x == x_target and all_node[i].y == y_target:
                all_node[i].set_block(0)
                target_node = all_node[i]
            #test no path
            # if (all_node[i].x == 0 and all_node[i].y == 1) or (all_node[i].x == 1 and all_node[i].y == 1) or (all_node[i].x == 1 and all_node[i].y == 0):
            #     all_node[i].set_block(1)               
            
        for i in range(0, len(all_node)):
            all_node[i].set_h(target_node)


        selected_node = [start_node]
        start_node.set_torched()
        start_node.set_g(0)
        selected_node = [start_node]
        open_list = [start_node]
        idx = 0
        
        if data_structure == 'priority':
            
            open_list = queue.PriorityQueue()
            open_list.put(((start_node.g + start_node.h, -start_node.g, idx), start_node))
            
            while 1:
                
                if open_list.empty():
                    print('The agent or the target is surrounded by obstacles and there is no viable path.')
                    return   
                             
                current_node = open_list.get()[-1]
                current_node.set_torched()
                
                if current_node == target_node:
                    break
                
                for candidate_son in current_node.neighbors:
                    if candidate_son.is_block == 1 or candidate_son.is_torched == 1:
                        continue
                    plt.scatter(candidate_son.x, candidate_son.y, color='orange')
                    idx += 1
                    candidate_son_gn_update = current_node.g + current_node.get_distance(current_node)
                    candidate_son_fn_update  = candidate_son_gn_update  + candidate_son.h
                    
                    if (candidate_son.g == None) or candidate_son_gn_update < candidate_son.g:
                        candidate_son.set_parent(current_node)
                        
                        open_list.put(((candidate_son_fn_update, -candidate_son.g, idx), candidate_son))
                
            son_node = target_node

            while 1:
                plt.scatter(son_node.parent.x, son_node.parent.y, color='red')
                son_node = son_node.parent
                if son_node.parent == start_node:
                    break

            for i in range(0, len(all_node)):
                if all_node[i].is_block == 1:       
                    plt.scatter(all_node[i].x, all_node[i].y, color='black')    
                        
            plt.scatter(x_start, y_start, color='blue')
            plt.scatter(x_target, y_target, color='blue')  
                                                        
            ax=plt.gca()
            ax.xaxis.set_major_locator(plt.MultipleLocator(1))
            ax.yaxis.set_major_locator(plt.MultipleLocator(1))       
            plt.xlim(-0.5, 9.5)    
            plt.ylim(-0.5, 9.5)   
            plt.show()                        
                        
        if data_structure == 'list':
            while 1:
                min_fn = 100000

                for this_node in selected_node: 
                    for candidate_son in this_node.neighbors:
                        if candidate_son.is_block == 1 or candidate_son.is_torched == 1:
                            continue 
                        plt.scatter(candidate_son.x, candidate_son.y, color='orange')
                        candidate_son_g = this_node.g + this_node.get_distance(candidate_son)
                        candidate_son_fn = candidate_son_g + candidate_son.h
                        #print(candidate_son.h)
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
                min_son.set_parent(min_parent)
                min_son.set_torched()
                if min_son == target_node:
                    break
                
            son_node = target_node

            while 1:
                plt.scatter(son_node.parent.x, son_node.parent.y, color='red')
                son_node = son_node.parent
                if son_node.parent == start_node:
                    break
                
            for i in range(0, len(all_node)):
                if all_node[i].is_block == 1:       
                    plt.scatter(all_node[i].x, all_node[i].y, color='black')    
                        
            plt.scatter(x_start, y_start, color='blue')
            plt.scatter(x_target, y_target, color='blue')  
                        
            ax=plt.gca()
            ax.xaxis.set_major_locator(plt.MultipleLocator(1))
            ax.yaxis.set_major_locator(plt.MultipleLocator(1))       
            plt.xlim(-0.5, 9.5)    
            plt.ylim(-0.5, 9.5)   
            plt.show()  
        