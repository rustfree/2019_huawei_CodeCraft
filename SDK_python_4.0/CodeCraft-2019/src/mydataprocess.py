# coding=utf-8
import time
import copy
import mygraph
import math
import mymap
import random
from collections import deque, namedtuple

random.seed(100)
#mydataprocess.py


node_pair = {}
node_pair_channel = {}
map_speed_increase_time1 ={}
map_speed_increase_time2 ={}
map_id = 0

def mapprocess(info_car,info_cross,info_road,road_dict,cross_dict):
    global map_id
    if info_cross[0][0] == 10:
        map_id = 1
    else:
        map_id = 2

    new_info_car=[]
    new_info_cross =[]
    new_info_road =[]
    cross_map = mymap.get_cross_map(road_dict,cross_dict)
    for cross_item in info_cross:
        cross_id = cross_item[0]
        cross_item[0] = cross_map[cross_id] 
        new_info_cross.append(cross_item)

    for car_item in info_car:
        src = car_item[1]
        dest = car_item[2]
        car_item[1] = cross_map[src]
        car_item[2] = cross_map[dest]
        new_info_car.append(car_item)
        
    for road_item in info_road:
        src = road_item[4]
        dest = road_item[5]
        road_item[4] = cross_map[src]
        road_item[5] = cross_map[dest]
        new_info_road.append(road_item)
    return new_info_car,new_info_cross,new_info_road

#预处理info_road,返回（src,des,len)
def road_process(info_road): 
    path_all_weight = [] 
    path_all = []
    #path_id = []
    for road_item in info_road:
        path_one = [road_item[4],road_item[5],road_item[1]]#/road_item[3] ] #src,des,len/lane_num，加入车道数的约束
        path_all_weight.append(path_one)
        path_all.append((road_item[4],road_item[5]))
        #path_id.append[road_item[0]]
        if road_item[6] == 1: #two direction
            path_one = [road_item[5],road_item[4],road_item[1]]#/road_item[3]] #des,src,len
            path_all_weight.append(path_one)
            path_all.append((road_item[5],road_item[4]))
            #path_id.append[road_item[0]]
    return path_all_weight,path_all #,path_id

    #根据各辆车起点和目标，返回最短路径
def get_car_path(info_car,info_road,info_cross):
    path_all_original ,path_all= road_process(info_road)
    path_all_weight = copy.deepcopy(path_all_original)#深复制

    result_temp = []
    #every car
    #path_src_des = {} #记录各个点对的路径
    #先放入id和计划时间，再附上路径
    #生成各个最短路径并储存到二维字典中
    path_src_dest_first = {}
    path_src_dest_then = {}
    num_src_dest_first ={}
    num_src_dest_then = {}
    for car_item in info_car:
        src = car_item[1]  #源节点
        dest = car_item[2] #目的节点
        if src > dest :
            #节点对加入字典
            if src not in path_src_dest_first.keys():  #添加源节点
                path_src_dest_first[src] = {}
                path_src_dest_first[src][dest] = None#字典，第一维度为src,第二维度为dest,内容为路径队列
            elif dest not in path_src_dest_first[src].keys():   #如果目标节点没有导入
                path_src_dest_first[src][dest] = None
            #各个路径出现次数
            if (src,dest) not in num_src_dest_first.keys():
                num_src_dest_first[(src,dest)] = 1
            else:
                num_src_dest_first[(src,dest)] = num_src_dest_first[(src,dest)] + 1
        elif src < dest :
            #节点对加入字典
            if src not in path_src_dest_then.keys():  #添加源节点
                path_src_dest_then[src] = {}
                path_src_dest_then[src][dest] = None#字典，第一维度为src,第二维度为dest,内容为路径队列
            elif dest not in path_src_dest_then[src].keys():   #如果目标节点没有导入
                path_src_dest_then[src][dest] = None
            #各个(src,dest)出现次数
            if (src,dest) not in num_src_dest_then.keys():
                num_src_dest_then[(src,dest)] = 1
            else:
                num_src_dest_then[(src,dest)] = num_src_dest_then[(src,dest)] + 1

    #到底是正序好还是逆序好有待验证！！！
    src_first = list(path_src_dest_first.keys()) #src比dest大
    src_first.sort() #src正序排序 33， 34 ，35

    src_then = list(path_src_dest_then.keys()) #dest比src小
    src_then.sort(reverse = True) # 32， 31， 30

    print('src_first:',src_first)
    print('src_then:',src_then)

    node_pair_initialize(info_road)#道路车辆数清零
    for src in src_first:
        graph = mygraph.GRAPH(path_all_weight)  #生成有向图
        path = graph.get_path(src,list(path_src_dest_first[src].keys()))#生成某个源节点到各个节点的最短路径
        #print('path:',path)
        path_all_weight = change_path_weight( path, path_all_original,path_all,num_src_dest_first) #更新权重
        for dest in path_src_dest_first[src].keys():
            path_src_dest_first[src][dest] = path[dest]
    print('node_pair1:',node_pair)
    #print('path_src_dest_first,',path_src_dest_first)

    node_pair_initialize(info_road)#道路车辆数清零
    for src in src_then:
        graph = mygraph.GRAPH(path_all_weight)  #生成有向图
        path = graph.get_path(src,list(path_src_dest_then[src].keys()))#生成某个源节点到各个节点的最短路径
        #print('path:',path)
        path_all_weight = change_path_weight( path, path_all_original,path_all,num_src_dest_then) #更新权重
        for dest in path_src_dest_then[src].keys():
            path_src_dest_then[src][dest] = path[dest]
    print('node_pair2:',node_pair)
    #print('path_src_dest_then,',path_src_dest_then)

    #为每一辆车匹配路径
    for car_item in info_car:
        result_item = [car_item[0], car_item[1],car_item[4],car_item[3]]  #  car_id and cross_id,go_time
        src = car_item[1]
        dest = car_item[2]
        if src > dest:
            path_ = path_src_dest_first[src][dest]
        else:
            path_ = path_src_dest_then[src][dest]
        #print('path:',path_)
        for i in range(len(path_)-1):  #遍历每个最短路径，节点序列转为路段序列
            #找回道路id和计划时间
            for road_item in info_road:#遍历每条路段
                if (road_item[4] == path_[i] and road_item[5] ==  path_[i+1]) or (road_item[5] == path_[i] and road_item[4] ==  path_[i+1]):#起始点一样或相反
                    result_item.append(road_item[0]) #路径id
        result_temp.append(result_item)
    #修改实际出发时间
    result_final = plan_start_time(info_car,info_cross,result_temp)
    return result_final 


def plan_start_time(info_car,info_cross,result_temp):
    global map_id
    
    global map_speed_increase_time1
    global map_speed_increase_time2

    map_speed_increase_time1 = {16:0,14:80,12:150,10:220,8:290,6: 360, 4:450} #官网可通过
    map_speed_increase_time2 = {16:700,14:760,12:840,10:900,8:970,6:1050 ,4:1150}

    
    if map_id ==1:
        pre_time = 300 

        priority_speed1 = {16:0,14:2,12:5,10:12,8:20,6: 30, 4:40}
        priority_speed2 = {16:140,14:145,12:150,10:158,8:168,6: 178, 4:188}  
        priority_time = 200
    else:
        pre_time = 200
        priority_speed1 = {16:0,14:2,12:5,10:12,8:20,6: 30, 4:40}
        priority_speed2 = {16:180,14:185,12:190,10:198,8:208,6: 218, 4:228}
        priority_time = 180
        

    result_final = []
    for i in range(len(info_car)):

        if info_car[i][5] == 0:# not prioity
            if info_car[i][2] < info_car[i][1]: #dest<src先走
                speed = info_car[i][3]
                result_temp[i][2] = max(map_speed_increase_time1[speed],result_temp[i][2]) + pre_time + priority_time
                
            elif info_car[i][2] >= info_car[i][1]: #dest>src后走
                speed = info_car[i][3]
                result_temp[i][2] = max(map_speed_increase_time2[speed],result_temp[i][2]) + pre_time + priority_time
        else: #prioity
            if info_car[i][2] >= info_car[i][1]: #dest>=src先走
                speed = info_car[i][3]
                result_temp[i][2] = max(priority_speed1[speed],result_temp[i][2]) + pre_time
                
            elif info_car[i][2] < info_car[i][1]: #dest<src后走
                speed = info_car[i][3]
                result_temp[i][2] = max(priority_speed2 [speed],result_temp[i][2]) + pre_time
        
        result_temp[i].pop(1)
        result_temp[i].pop(2)
        result_final.append(tuple(result_temp[i]))
    return result_final


def change_path_weight(path,path_all_original,path_all,num_src_dest): #path:是刚刚新src得到的所有路径，是一个字典；path_all： 图的输入
    global node_pair
    global node_pair_channel
    path_all_weight = copy.deepcopy(path_all_original)

    for path_item in path.values():
        path_item = list(path_item) #某个路径的节点序列
        src_dest_index = (path_item[0],path_item[-1]) #(src,dest)
        #print('path_item:',path_item)
        for i in range(len(path_item)-1): #遍历每个路径
            node_pair_index = (path_item[i], path_item[i+1])  #(crossid1,crossid2)
            node_pair[node_pair_index] = node_pair[node_pair_index] + num_src_dest[src_dest_index]
    #这里说明计算过程：每次以一个源节点规划到各个目标的路径，每次，有不同dest,对每个dest:（src,dest)都有一定数量的车，
    #这些车的路径path_item都是一样的，因此车的数量有n，相当于它的路径下，每个（crossid1,crossid2)出现了n次
    #这里n即为num_src_dest[src_dest_index]
    
    for node_pair_item in node_pair.keys():#查找节点对
        road_car_num = node_pair[node_pair_item]

        if node_pair_channel[node_pair_item] >=3:
            if road_car_num > 2000 and road_car_num <= 2500:
                
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*2/2000) #更新权重

            elif road_car_num > 2500 and road_car_num <= 3500:
                
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*4/2000) #更新权重
            
            elif road_car_num > 3500:
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*6/2000) #更新权重
        
        elif node_pair_channel[node_pair_item] == 1:
            if road_car_num > 1000 and road_car_num <= 1500:
                #print(node_pair_item,road_car_num)
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*2/1000) #更新权重
            elif road_car_num > 1500 :
                #print(node_pair_item,road_car_num)
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*4/1000) #更新权重

        elif node_pair_channel[node_pair_item] == 2:
            if road_car_num > 1500 and road_car_num <= 2500:
                #print(node_pair_item,road_car_num)
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*1.5/1500) #更新权重
            elif road_car_num > 2500 :
                #print(node_pair_item,road_car_num)
                index = path_all.index(node_pair_item)
                path_all_weight[index][2] = path_all_original[index][2]*(road_car_num*5/1500) #更新权重  
    #print('node_pair:',node_pair)
    
    return path_all_weight


def node_pair_initialize(info_road):
    global node_pair
    global node_pair_channel
    for road_item in info_road:
        src = road_item[4]
        dest = road_item[5]
        node_pair[(src,dest)] = 0
        node_pair_channel[(src,dest)] = road_item[3]
        if road_item[6] == 1:
            node_pair[(dest,src)] = 0
            node_pair_channel[(dest,src)] = road_item[3]

#统计返回 {各个cross_id： 出发的车总量}
def count_car_per_cross(info_car,len_info_cross):
    cross_car_num = {}
    #print('info_car',info_car)
    for car_item in info_car:
        if car_item[1] not in list(cross_car_num.keys()): #car_item[1]是该车的始发点
            cross_car_num[car_item[1]] = 1
        else:
            cross_car_num[car_item[1]] = cross_car_num[car_item[1]] + 1
    #print('cross_car_num',cross_car_num)
    for i in range(len_info_cross):
        if i not in cross_car_num.keys():
            cross_car_num[i] = 0
    return cross_car_num






        





        




