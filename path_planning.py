from asyncore import loop
import queue
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image

from node import My_Node
    
class PathPlanning:
    def __init__(self, algo_select):
        self.algo_select = algo_select
        return

    def do_path_planning(self, map, x_start, y_start, x_target, y_target):
        if self.algo_select == 'BFS':
            return self.BFS_search(map, x_start, y_start, x_target, y_target)
        if self.algo_select == 'DFS':
            return self.DFS_search(map, x_start, y_start, x_target, y_target)        
        if self.algo_select == 'Astar':
            return self.astar_search(map, x_start, y_start, x_target, y_target, data_structure='PriorityQueue')
        if self.algo_select == 'RRT':
            return self.RRT_sampling(map, x_start, y_start, x_target, y_target)
        if self.algo_select == 'RRTstar':
            return self.RRTstar_sampling(map, x_start, y_start, x_target, y_target)
        
    def PRM_sampling(self, map, x_start, y_start, x_target, y_target, sample_size = 100):  
        x_sample = np.random.randint(0, map.shape[0], size=(sample_size))
        y_sample = np.random.randint(0, map.shape[1], size=(sample_size))
        for i in range(0, sample_size):
            for j in range(0, map.shape[1]):
                if map[x_sample[i]][y_sample[i]] < 255:
                    free_x_sample = np.delete(x_sample, i)
                    free_y_sample = np.delete(y_sample, i)
                    
    def RRT_sampling(self, obstacle, x_start, y_start, x_target, y_target, tolerance = 50):  
        map = mpimg.imread('newmap.png')   
        plt.imshow(map)
        start_node = My_Node(x_start, y_start)
        start_node.set_g(0)
        target_node = My_Node(x_target, y_target)
        tree_node = [start_node]

        while 1:
            x_sample = np.random.randint(0, map.shape[0])
            y_sample = np.random.randint(0, map.shape[1])            
            rand_node = My_Node(x_sample, y_sample)
            min_distance = 10000
            for node in tree_node:
                distance = rand_node.get_distance(node)
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = node
            x_new = nearest_node.x + 0.3*(rand_node.x-nearest_node.x)
            y_new = nearest_node.y + 0.3*(rand_node.y-nearest_node.y)
            
            new_node = My_Node(x_new, y_new)

            if not (self.is_node_obstacle_free(new_node, obstacle) and self.is_edge_obstacle_free(nearest_node, new_node, obstacle)):
                continue
                    
            new_node.set_parent(nearest_node)
            tree_node.append(new_node)
            
            plt.scatter(new_node.x, new_node.y, s=2, color='orange')    
            if math.sqrt(math.pow(new_node.x - target_node.x,2) + math.pow(new_node.y - target_node.y,2)) < tolerance:
                break
        
        son_node = new_node
        #print(str(son_node.x)+','+str(son_node.y))
        x_path = []
        y_path = []

        while 1:
            x_path.append(son_node.x)
            y_path.append(son_node.y)
            son_node = son_node.parent
            if son_node == start_node:
                break
            
        x_path.append(start_node.x)
        y_path.append(start_node.y)         
        plt.plot(x_path, y_path, label='Frist line',linewidth=1,color='r',marker='o', markerfacecolor='blue',markersize=4)
        plt.show()
                                  
    def is_node_obstacle_free(self, new_node, obstacle):
        for i in range(0, len(obstacle)):    
            if new_node.x >= obstacle[i].x1 and new_node.x <= obstacle[i].x2 and new_node.y >= obstacle[i].y1 and new_node.y <= obstacle[i].y2:
                return False
        
        return True
              
    def is_edge_obstacle_free(self, nearest_node, new_node, obstacle):
        # print('nearest_node:'+str(nearest_node.x)+','+str(nearest_node.y))
        # print('new_node:'+str(new_node.x)+','+str(new_node.y))
        for i in range(0, len(obstacle)):
            if not ((min(new_node.x, nearest_node.x) > obstacle[i].x2) or (max(new_node.x, nearest_node.x) < obstacle[i].x1) or (min(new_node.y, nearest_node.y) > obstacle[i].y2) or (max(new_node.y, nearest_node.y) < obstacle[i].y1)):
                if new_node.x-nearest_node.x == 0:
                        return False
                if new_node.x-nearest_node.x != 0:    
                    y_check1 = (obstacle[i].x1-nearest_node.x)/(new_node.x-nearest_node.x)*(new_node.y-nearest_node.y)+nearest_node.y
                    y_check2 = (obstacle[i].x2-nearest_node.x)/(new_node.x-nearest_node.x)*(new_node.y-nearest_node.y)+nearest_node.y
                    if not ((y_check1 > obstacle[i].y2 and y_check2 > obstacle[i].y2) or (y_check1 < obstacle[i].y1 and y_check2 < obstacle[i].y1)):
                        return False
        return True
    
    
    def RRTstar_sampling(self, obstacle, x_start, y_start, x_target, y_target, near_range = 80, tolerance = 100):  
        map = mpimg.imread('newmap.png')   
        plt.imshow(map)
        start_node = My_Node(x_start, y_start)
        start_node.set_g(0)
        target_node = My_Node(x_target, y_target)
        tree_node = [start_node]
        target_node_gn = 10000000.0
        loop = 0
        not_update_num = 0
        
        while 1:
            loop += 1
            if loop % 100 == 1:
                print('loop:'+str(loop))
            x_sample = np.random.randint(0, map.shape[0])
            y_sample = np.random.randint(0, map.shape[1])            
            rand_node = My_Node(x_sample, y_sample)
            min_distance = 10000
            for node in tree_node:
                distance = rand_node.get_distance(node)
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = node
            x_new = nearest_node.x + 0.3*(rand_node.x-nearest_node.x)
            y_new = nearest_node.y + 0.3*(rand_node.y-nearest_node.y)
            new_node = My_Node(x_new, y_new)
            
            if not self.is_node_obstacle_free(new_node, obstacle):
                continue

            near_list = []
            for node in tree_node:
                distance = new_node.get_distance(node)
                if distance < near_range:
                    near_list.append(node)
            
            if len(near_list) == 0:
                continue
            
            min_new_node_gn = 10000000
            #print('near_list_len:'+str(len(near_list)))
            for node in near_list:
                # print('new_node.g:'+str(new_node.g), 'new_node.x:'+str(new_node.x), 'new_node.y:'+str(new_node.y))
                # print('node.g:'+str(node.g), 'node.x:'+str(node.x), 'node.y:'+str(node.y))
                if not self.is_edge_obstacle_free(node, new_node, obstacle):
                    continue
                
                new_node_gn = node.g + new_node.get_distance(node)
                if new_node_gn < min_new_node_gn:
                     
                    min_new_node_gn = new_node_gn
                    min_node = node
                       
                new_node.set_parent(min_node)
                min_node.append_son(new_node) 
            
            if new_node.parent == None:
                continue                 
            tree_node.append(new_node)
              
            for node in near_list:
                if not self.is_edge_obstacle_free(node, new_node, obstacle):
                    continue             
                node_gn_update = new_node.g + node.get_distance(new_node)
                if node_gn_update < node.g: # update through new_node
                    node.set_parent(new_node)
                    new_node.append_son(node) 
                    node.parent.remove_son(node)
            
            updata_son_queue = queue.Queue()
            updata_son_queue.put(new_node)
            
            num = 0
            while not updata_son_queue.empty():
                num += 1
                #print(num)
                current_node = updata_son_queue.get()
                
                for son_node in current_node.son:
                    updata_son_queue.put(son_node)
                    update_g = current_node.g + son_node.get_distance(current_node)
                    son_node.set_g(update_g)           
            
            plt.scatter(new_node.x, new_node.y, s=2, color='orange')    
            #print('new_node.g:'+str(new_node.g), 'new_node.x:'+str(new_node.x), 'new_node.y:'+str(new_node.y))
            if (math.sqrt(math.pow(new_node.x - target_node.x, 2) + math.pow(new_node.y - target_node.y, 2)) < tolerance):
                if target_node.g == None or target_node.g > new_node.g:
                    target_node.parent = new_node.parent
                    target_node.set_g(new_node.g)
            
            #print('target_node.g='+str(target_node.g)) 
            #if target_node.g != None:
                #print('gra='+str(math.pow(target_node.g - target_node_gn, 2)))
            
            if (target_node.g != None) and (math.pow(target_node.g - target_node_gn, 2) > 0.5):  
                not_update_num = 0               
            if (target_node.g != None) and (math.pow(target_node.g - target_node_gn, 2) < 0.5):
                not_update_num += 1
                if not_update_num>1000:
                    break
                
            if target_node.g != None:
                target_node_gn = target_node.g
            
        son_node = target_node
        #print(str(son_node.x)+','+str(son_node.y))
        x_path = []
        y_path = []

        while 1:
            x_path.append(son_node.x)
            y_path.append(son_node.y)
            son_node = son_node.parent
            if son_node == start_node:
                break
            
        x_path.append(start_node.x)
        y_path.append(start_node.y)         
        plt.plot(x_path, y_path, label='Frist line',linewidth=1,color='r',marker='o', markerfacecolor='blue',markersize=4)
        plt.show()       
        
    def DFS_search(self, all_node, x_start, y_start, x_target, y_target):
    
        for i in range(0, len(all_node)):
            all_node[i].set_block(0)
            if all_node[i].x == x_start and all_node[i].y == y_start:
                all_node[i].set_block(0)
                start_node = all_node[i]
            if all_node[i].x == x_target and all_node[i].y == y_target:
                all_node[i].set_block(0)
                target_node = all_node[i]

        for i in range(0, len(all_node)):
            all_node[i].set_h(target_node)
        
        start_node.set_torched()  
        start_node.set_g(0) 
        open_list = queue.LifoQueue()   
        open_list.put(start_node)   
           
        while 1:
            if open_list.empty():
                print('The agent or the target is surrounded by obstacles and there is no viable path.')
                return
            
            current_node = open_list.get()
            current_node.set_torched()
            print('current_node:'+str(current_node.x)+','+str(current_node.y))
            if current_node == target_node:
                break
            
            for candidate_son in current_node.neighbors:
                plt.scatter(candidate_son.x, candidate_son.y, color='orange')
                print(str(candidate_son.x)+','+str(candidate_son.y))
                if candidate_son.is_block == 1 or candidate_son.is_torched == 1 or candidate_son.isin_queue == 1:
                    continue

                candidate_son.set_parent(current_node)
                    
                    
                if candidate_son.isin_queue == 0:
                    open_list.put(candidate_son)
                    candidate_son.isin_queue = 1
                
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
                    
    def BFS_search(self, all_node, x_start, y_start, x_target, y_target):

        for i in range(0, len(all_node)):
            if all_node[i].x == x_start and all_node[i].y == y_start:
                all_node[i].set_block(0)
                start_node = all_node[i]
            if all_node[i].x == x_target and all_node[i].y == y_target:
                all_node[i].set_block(0)
                target_node = all_node[i]

        for i in range(0, len(all_node)):
            all_node[i].set_h(target_node)
        
        start_node.set_torched()  
        start_node.set_g(0) 
        open_list = queue.Queue()   
        open_list.put(start_node)   
           
        while 1:
            if open_list.empty():
                print('The agent or the target is surrounded by obstacles and there is no viable path.')
                return
            
            current_node = open_list.get()
            current_node.set_torched()
            
            if current_node == target_node:
                break
            
            for candidate_son in current_node.neighbors:
                plt.scatter(candidate_son.x, candidate_son.y, color='orange')
                if candidate_son.is_block == 1 or candidate_son.is_torched == 1 or candidate_son.isin_queue == 1:
                    continue

                candidate_son.set_parent(current_node)
                    
                    
                if candidate_son.isin_queue == 0:
                    open_list.put(candidate_son)
                    candidate_son.isin_queue = 1
                
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
        idx = 0
        
        if data_structure == 'PriorityQueue':
            
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
        