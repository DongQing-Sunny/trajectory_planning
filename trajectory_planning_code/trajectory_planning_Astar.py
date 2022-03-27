from utils import ObstacleMap, insert_open, expand_list, min_fn, hn_Astar
import matplotlib.pyplot as plt

x_start = 0
y_start = 0

gn = 0
x_target = 9
y_target= 9
MAX_X = 9
MAX_Y = 9


map = ObstacleMap(x_start, y_start, x_target, y_target, MAX_X, MAX_Y, 0.25)
whole_map = map.create_map()
for i in range(0, len(whole_map)):
    plt.scatter(whole_map[i][0], whole_map[i][1])
plt.show()

closed_list = whole_map
#print(closed_list)
# hn_start = hn_Astar(x_target, y_target, x_node, y_node)
# open_list = [1,0,0,0,0,hn_start,0,hn_start]
open_list = []
x_node = x_start
y_node = y_start

for loop in range(0, 10000):
    #扩展结点：寻找当前节点的儿子，并计算hn，gn和fn=gn+hn
    exp_list = expand_list(x_node, y_node, gn, x_target, y_target, closed_list, MAX_X, MAX_Y)
    #print(exp_list)
    #将新扩展的节点加入open_list
    for i in range(0, len(exp_list)):
        new_node = insert_open(exp_list[i], x_node, y_node)
        open_list.append(new_node)
    
    # if len(open_list)==0:
    #     break
    
        
    #从open_list中选出fn最小的node，将选中的node从open_list中首位标记置0
    node_selected = min_fn(open_list)
    #将选中的node，放入close_list
    closed_list.append([node_selected[1], node_selected[2]])
    x_node = node_selected[1]
    y_node = node_selected[2]
    gn = node_selected[6]
    if node_selected[1]==x_target and node_selected[2]==y_target:
        break

path = [[node_selected[1], node_selected[2]]]  
child_node = node_selected 
for i in range(0,100):

    x_parent = child_node[3]
    y_parent = child_node[4]
    for i in range(0, len(open_list)):
        if x_parent==open_list[i][1] and y_parent==open_list[i][2]:
            path.append([open_list[i][1], open_list[i][2]])
            child_node = open_list[i]
            
    if child_node[1] == x_start and child_node[2] == y_start:
        break
    
for i in range(0, len(whole_map)):
    plt.scatter(whole_map[i][0], whole_map[i][1])
    
# for i in range(0, len(path)):
#     plt.scatter(path[i][0], path[i][1])
plt.plot(path)    
plt.show()
    
        
