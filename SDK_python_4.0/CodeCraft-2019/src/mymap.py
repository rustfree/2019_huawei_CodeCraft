import sys
import numpy as np
cross_visited = {}
cross_XY = {}

def read_file(filename):
    s1 = []
    s2 = []
    s3 = []
    s4 = {}
    
    with open(filename) as f:
        s1 = f.readlines()
        for line in s1:
            s2.append([x.strip('()\n') for x in line.split(',')])
        del s2[0]
        s3 = [[int(col) for col in row] for row in s2]
        for i, tmp in enumerate(s3):
            s4[s3[i][0]] = tmp[1:] 
    return s4

def update_cross(origin_cross_dict, origin_road_dict):
    for i, tmp in origin_cross_dict.items():
        for j in range(4):
            road_id = tmp[j]
            if road_id == -1:
                tmp[j] = (-1, -1, -1, -1)
                continue
            roda_data = origin_road_dict[road_id]
            if i == roda_data[3]:
                tmp[j] = (roda_data[4], road_id, roda_data[0], 0)
            elif i == roda_data[4] and roda_data[5]:
                tmp[j] = (roda_data[3], road_id, roda_data[0], 1)
            else:
                tmp[j] = (-1, -1, -1, -1)


def Set_Cross_Direction(cur_cross_id, cross_dict_input, direction=None, pre_cross_id=None):
    if cross_visited[cur_cross_id]:
        return
    finish_state = 0
    if pre_cross_id != None:
        for i, cross_list in enumerate(cross_dict_input[cur_cross_id]):
            if cross_list[0] == pre_cross_id:
                finish_state = 1
                now_dir = i
                if abs(now_dir - direction) == 2:
                    num_rotation = 0
                elif now_dir - direction == 0:
                    num_rotation = 2
                elif abs(now_dir - direction) == 3:
                    num_rotation = 2 + int((now_dir - direction) < 0) * (-2) + 1
                elif abs(now_dir - direction) == 1:
                    num_rotation = 2 + (direction - now_dir)
                
                
                #num_rotation = ((now_dir + 2) % 4 - direction) % 4
                for k in range(num_rotation):
                    cross_dict_input[cur_cross_id] = [cross_dict_input[cur_cross_id][3], cross_dict_input[cur_cross_id][0],
                                     cross_dict_input[cur_cross_id][1], cross_dict_input[cur_cross_id][2]]
                
                break
            
    if finish_state == 0 and pre_cross_id != None:
        return
    cross_visited[cur_cross_id] = True
            
    for i, cross_list in enumerate(cross_dict_input[cur_cross_id]):
        if cross_list[0] != -1:
            Set_Cross_Direction(cross_list[0], cross_dict_input, i, cur_cross_id)

                    
def Set_Cross_XY(center_cross_id, cross_dict_input, cross_XY_input):
    max_min_x =[0, sys.maxsize]
    max_min_y =[0, sys.maxsize]
    queue = []
    visited = set()
    queue.append(center_cross_id)
    visited.add(center_cross_id)
    
    while len(queue) != 0:
        node = queue.pop(0)
        for direction, tmp in enumerate(cross_dict_input[node]):
            adj_node = tmp[0]
            if adj_node != -1:
                if adj_node not in visited:
                    visited.add(adj_node)
                    queue.append(adj_node)
                    if direction == 0:
                        cross_XY_input[adj_node][0], cross_XY_input[adj_node][1] = cross_XY_input[node][0], cross_XY_input[node][1] - 1
                    elif direction == 2:
                        cross_XY_input[adj_node][0], cross_XY_input[adj_node][1] = cross_XY_input[node][0], cross_XY_input[node][1] + 1
                    elif direction == 1:
                        cross_XY_input[adj_node][0], cross_XY_input[adj_node][1] = cross_XY_input[node][0] + 1, cross_XY_input[node][1]
                    elif direction == 3:
                        cross_XY_input[adj_node][0], cross_XY_input[adj_node][1] = cross_XY_input[node][0] - 1, cross_XY_input[node][1]
                    
                    x = cross_XY_input[adj_node][0]
                    y = cross_XY_input[adj_node][1]
                    if x > max_min_x[0]:
                        max_min_x[0] = x
                    elif x < max_min_x[1]:
                        max_min_x[1] = x
                    
                    if y > max_min_y[0]:
                        max_min_y[0] = y
                    elif y < max_min_y[1]:
                        max_min_y[1] = y
    return max_min_x, max_min_y
                    

def get_cross_map(road_dict,cross_dict):

    update_cross(cross_dict, road_dict)

    for cross_id in cross_dict.keys():
        cross_visited[cross_id] = False
        
    Set_Cross_Direction(list(cross_dict.keys())[0], cross_dict)
    
    for cross_id in cross_dict.keys():
        cross_visited[cross_id] = False
        cross_XY[cross_id] = [0, 0]
        
    x_limit, y_limit = Set_Cross_XY(list(cross_dict.keys())[0], cross_dict, cross_XY)
    map_matrix = np.zeros(((y_limit[0] - y_limit[1] + 1), (x_limit[0] - x_limit[1] + 1))) - 1
    
    for cross_id, XY_loc in cross_XY.items():
        XY_loc[0] -= x_limit[1]
        XY_loc[1] -= y_limit[1]
        map_matrix[XY_loc[1]][XY_loc[0]] = cross_id

    map_list = map_matrix.tolist()#getA().tolist()
    int_map_list = []
    for i in range(len(map_list)):
        for j in range(len(map_list[i])):
            int_map_list.append( int(map_list[i][j]) )
    print('int_map_list:',int_map_list)
    
    cross_map = {}
    for i  in range( len(int_map_list) ):
        if int_map_list [i] != -1:
            cross_map[int_map_list[i]] = i+1
     
    print('cross_map:',cross_map)
    return cross_map