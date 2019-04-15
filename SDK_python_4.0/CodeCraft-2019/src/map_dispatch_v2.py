# -*- coding: utf-8 -*-
"""
Created on Wed Apr  3 14:52:07 2019

@author: fqsfyq
"""
import sys
import numpy as np
import time

SPEEDLIST = [16, 14, 12, 10, 8, 6, 4]
global DEADLOCKLOOPLOG, NOMOVECAR
DEADLOCKCAR, DEADLOCKLOOPLOG, NOMOVECAR = None, [], set()
CAR_NAMESPACE, ROAD_NAMESPACE, CROSS_NAMESPACE = [], [], []
CAR_DICT, ROAD_DICT, CROSS_DICT = {}, {}, {}
ALL_ROAD_PATH = {}
FINISHED_CAR, CARonGOING = [], []
ROAD_MAP, WEIGHT_CROSS, FirstPriCarSequence = {}, {}, {}
# =============================================================================
# data log of priority car
# =============================================================================
global TotalNumOfPriCar, TimeFirstPriCarOut, TimeLastPriCarOut, TimeLastPriCarEnd, MaxSpeedOfPriCar, MinSpeedOfPriCar
global DepartureOfPriCar, DestinationOfPriCar
TotalNumOfPriCar, TimeFirstPriCarOut, TimeLastPriCarOut, TimeLastPriCarEnd, MaxSpeedOfPriCar, MinSpeedOfPriCar = \
0, sys.maxsize, 0, 0, 0, sys.maxsize
DepartureOfPriCar, DestinationOfPriCar = set(), set()
# =============================================================================
# data log of all car
# =============================================================================
global TotalNumOfUsualCar, TimeFirstUsualCarOut, TimeLastUsualCarOut, MaxSpeedOfUsualCar, MinSpeedOfUsualCar
global DepartureOfUsualCar, DestinationOfUsualCar
TotalNumOfUsualCar, TimeFirstUsualCarOut, TimeLastUsualCarOut, MaxSpeedOfUsualCar, MinSpeedOfUsualCar = \
0, sys.maxsize, 0, 0, sys.maxsize
DepartureOfUsualCar, DestinationOfUsualCar = set(), set()

TEST_LOG = []
global sysTIME
sysTIME = 0

def read_file(filename):
    s1 = []
    s2 = []
    s3 = []
    s4 = {}
    
    with open(filename) as f:
        s1 = f.readlines()
        for line in s1:
            s2.append([x.strip('()\n') for x in line.split(',')])
        if 'answer' not in filename:
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
                
def Find_Minindex(Sptset, distance):
    min = sys.maxsize
    min_index = -1
    
    for cross_id in Sptset.keys():
        if Sptset[cross_id] and min > distance[cross_id][0]:
            (min, min_index) = (distance[cross_id][0], cross_id)
    return min_index

#def Map_Stimulation(car_input_dict, cross_input_dict, road_input_dict):
#    cross_garage = {}
#    for i, tmp in car_input_dict.items():
#        if(tmp[0] not in cross_garage.keys()):
#            cross_garage[tmp[0]] = [[i, tmp[2], tmp[3]]]   #{cross_id:[car_id, speed, planTime]}
#        else:
#            cross_garage[tmp[0]].append([i, tmp[2], tmp[3]])
#    
#    for i, tmp in cross_garage.items():
#        cross_garage[i] = sorted(cross_garage[i], key=lambda x: [-x[:][1], x[:][2]]) #优先级排序
#        for j, speed in SPEEDLIST:
#            
#        
#    return cross_garage

def Dijkstra(node, cross_file_input):
    V = list(cross_file_input.keys())
    sptSet = {}
    Distance = {}
    for cross_id in V:
        sptSet[cross_id] = 1
        Distance[cross_id] = [0, 0] #[距离， 节点]
    path_dict = {}
    node_dict = {}
    
    for cross_id in V:
        Distance[cross_id][0] = 100000
        Distance[cross_id][1] = -1
        path_dict[cross_id] = []
        node_dict[cross_id] = []
    
    Distance[node][0] = 0
    Distance[node][1] = node
    node_dict[node] = [node]
    
    for cross_id in V:
        u = Find_Minindex(sptSet, Distance)
        sptSet[u] = 0
        
        for tmp in cross_file_input[u]:
            if tmp[0] != -1 and sptSet[tmp[0]] and Distance[u][0] + tmp[2] <  Distance[tmp[0]][0]:
                Distance[tmp[0]][0] = Distance[u][0] + tmp[2]
                Distance[tmp[0]][1] = u
                node_dict[tmp[0]] = node_dict[u] + [tmp[0]]
                path_dict[tmp[0]] = path_dict[u] + [tmp[1]]
    #Distance[:][1] += 1
    return path_dict, node_dict
# =============================================================================
# 输入参数说明：node：起点路口id
#             cross_input_dict：引用我的那个cross_dict就好
#             end_node:终点路口id
#             ban_node_dict：忽略路径字典，具体格式为：{1：[...],2:[...],3:[...]}
#                           比如说要忽略路口1到路口10的道路,则令 ban_node_dict[1].append(10)。
#
#输出：如果不存在该路径，则输出0，如存在，则在忽略相关路径的基础上，返回node到end_node的最短路径，返回的路径数据有两种格式，
#     一种以路口节点表示的路径，名为node_dict，另一种是以道路表示的路径，名为path_dict。
#    
#示范举例：如搜索节点1到节点60的路径，且忽略路口10到路口18、路口36到路口37的道路，相关程序为：
#                                            ban_node = {}
#                                            for cross_id in cross_dict.keys():
#                                                ban_node[cross_id] = []
#                                            
#                                            ban_node[36].append(37)
#                                            ban_node[10].append(18)
#                                            
#                                            test = Dijkstra(1, cross_dict)  #正常情况下，路口1到所有路口的最短路径
#                                            test1 = Dijkstra_Ver2(1, cross_dict, 60, ban_node)  #在忽略相关道路下，搜索路口1到路口60的最短路径
# =============================================================================
def Dijkstra_Ver2(node, cross_input_dict, road_input_dict, end_node, pre_RoadPath, from_node):
    ban_node_dict = {}
    for cross_id in cross_input_dict.keys():
        ban_node_dict[cross_id] = 0
    for road_id in pre_RoadPath:
        road_from, road_to = road_file[road_id][3], road_file[road_id][4]
        if from_node == road_from:
            head_node = road_from
            tail_node = road_to
        else:
            head_node = road_to
            tail_node = road_from
        ban_node_dict[head_node] = tail_node
        ban_node_dict[tail_node] = head_node
        from_node = tail_node
    
    V = list(cross_input_dict.keys())
    sptSet = {}
    Distance = {}
    for cross_id in V:
        sptSet[cross_id] = 1
        Distance[cross_id] = [0, 0] #[距离， 节点]
    path_dict = {}
    node_dict = {}
    for cross_id in V:
        Distance[cross_id][0] = 100000
        Distance[cross_id][1] = -1
        path_dict[cross_id] = []
        node_dict[cross_id] = []
    
    Distance[node][0] = 0
    Distance[node][1] = node
    node_dict[node] = [node]
    
    for cross_id in V:
        u = Find_Minindex(sptSet, Distance)
        sptSet[u] = 0
        
        if Distance[u][0] == 100000:
            return -1 
        
        for tmp in cross_input_dict[u]:
            if tmp[0] != -1 and tmp[0] != ban_node_dict[u]:
                if sptSet[tmp[0]] and Distance[u][0] + tmp[2] <  Distance[tmp[0]][0]:
                    Distance[tmp[0]][0] = Distance[u][0] + tmp[2]
                    Distance[tmp[0]][1] = u
                    node_dict[tmp[0]] = node_dict[u] + [tmp[0]]
                    path_dict[tmp[0]] = path_dict[u] + [tmp[1]]
                    if tmp[0] == end_node:
                        return path_dict[tmp[0]]
    return None
                        
def Get_All_Shortest_Path(cross_file_input):
    total_road_path = {}
    for cross_id in cross_file_input.keys():
        total_road_path[cross_id], _ = Dijkstra(cross_id, cross_file_input)
    return total_road_path
        
class CAR:
    def __init__(self, id_, from_ ,to_ , speed, planTime, priority, preset):
        self.car_id, self.cross_from, self.cross_to, self.car_speed, self.min_plantime, self.priority, self.preset = \
            id_, from_, to_, speed, planTime, priority, preset
        self.planTime, self.nextroad_index, self.route, self.state = 0, 0, None, 0
        self.loc, self.channel_id, self.nowRoad = None, None, None
        self.start_time, self.end_time = None, None   #the time of the car went out of the carpot and got to the destination
    
    def get_route(self, planTime, route):
        assert planTime >= self.min_plantime, 'the palntime in the input is illegal!'
        assert route != None, 'the route is None!'
        self.planTime = planTime
        self.route = route
    
    def update_route(self, route):
        assert route != None, 'the route is None!'
        self.route[self.nextroad_index:] = route
        
    def update_data(self, state=None, loc=None, channel_id=None, nowRoad=None, start_time=None, end_time=None):
        self.state = state if state != None else self.state
        self.loc = loc if loc != None else self.loc
        self.start_time = start_time if start_time != None else self.start_time
        self.end_time = end_time if end_time != None else self.end_time
        self.channel_id = channel_id if channel_id != None else self.channel_id
        if nowRoad != None:
            self.nowRoad = nowRoad
            self.nextroad_index += 1
    
    def __id__(self):
        return self.car_id
    def __from__(self):
        return self.cross_from
    def __to__(self):
        return self.cross_to
    def __speed__(self):
        return self.car_speed
    def __priority__(self):
        return self.priority
    def __preset__(self):
        return self.preset
    def __min_plantime__(self):
        return self.min_plantime
    def __planTime__(self):
        return self.planTime
    def __start_time__(self):
        return self.start_time
    def __end_time__(self):
        return self.end_time
    def __nextroad_index__(self):
        return self.nextroad_index
    def __route__(self):
        return self.route
    def __state__(self):
        return self.state
    def __loc__(self):
        return self.loc
    def __channel_id__(self):
        return self.channel_id
    def __nowRoad__(self):
        return self.nowRoad
        

class ROAD:
    def __init__(self, id_, length, speed, channel, from_, to, isDuplex):
        self.road_id, self.length, self.speed, self.channel, self.from_, self.to_, self.isDuplex = \
            id_, length, speed, channel, from_, to, isDuplex
            
        self.forward_stack = np.zeros((self.channel, self.length)) - 1
        self.backward_stack = np.zeros((self.channel, self.length)) - 1 if self.isDuplex == 1 else None
        self.cross_in_stack, self.cross_out_stack, self.dispatch_done = None, None, False
        #log the priority search location of road
        self.pri_loc = np.zeros(self.channel)
        self.forward_numCar, self.backward_numCar, self.capacity = [0, 0], [0, 0], self.channel*self.length  #[numOfUsualCar, numOfPriCar]
        
    def adjust_road_direction(self, cross_id):
        assert cross_id in [self.from_, self.to_], 'the road not in this cross'
        if cross_id == self.from_:
            self.cross_out_stack = self.forward_stack
            self.cross_in_stack = self.backward_stack if self.isDuplex == 1 else None
            self.CrossIn_numCar, self.CrossOut_numCar = self.backward_numCar, self.forward_numCar
        else:
            self.cross_out_stack = self.backward_stack if self.isDuplex == 1 else None
            self.cross_in_stack = self.forward_stack
            self.CrossIn_numCar, self.CrossOut_numCar = self.forward_numCar, self.backward_numCar
    
    #make all car of the road to be the state in waiting        
    def Road_Init(self):  
        assert self.cross_in_stack.all() != None, 'the cross_in_stack did not set'
        self.dispatch_done = False
        for channel_id in range(self.channel):
            self.pri_loc[channel_id] = self.length - 1
            for loc in range(self.length - 1, -1, -1):
                car_id = self.cross_in_stack[channel_id][loc]
                if car_id != -1:
                    car_object = CAR_DICT[car_id]
                    assert car_object.__loc__() == loc, 'the loc of car is not fit'
                    assert car_object.__channel_id__() == channel_id, 'the channel id of car is not fit'
                    car_object.update_data(state=1)
    
    def Find_Car(self, start_loc, end_loc, channel_id):
        assert start_loc < end_loc, 'the loc input is illegal'
        assert self.cross_out_stack.all() != None, 'the cross_in_stack did not set'
        for loc in range(start_loc, end_loc):
            car_id = self.cross_out_stack[channel_id][loc]
            if car_id != -1:
                return car_id
        return -1
    
    #uodate value weight of road
    def update_weight(self):
        from_id = self.from_
        to_id = self.to_
        for direction in range(self.isDuplex + 1):
            weight = self.length
            if direction == 0:
                UsualCarNum, PriCarNum = self.forward_numCar[0], self.forward_numCar[1]
                weight += 30 * (UsualCarNum / (self.capacity + 0.5 * self.speed)) + \
                60 * (PriCarNum / (self.capacity + 0.5 * self.speed))
                WEIGHT_CROSS[from_id][to_id] = weight
            else:
                UsualCarNum, PriCarNum = self.backward_numCar[0], self.backward_numCar[1]
                weight += 30 * (UsualCarNum / (self.capacity + 0.5 * self.speed)) + \
                60 * (PriCarNum / (self.capacity + 0.5 * self.speed))
                WEIGHT_CROSS[to_id][from_id] = weight
            
            
        
    #drive the car in the channel to be end state or wait state       
    def Drive_CurChannel(self, channel_id):
        assert self.cross_in_stack.all() != None, 'the cross_in_stack did not set'
        pre_carState, pre_carLoc = None, -1
        for loc in range(self.length - 1, -1, -1):
            car_id = self.cross_in_stack[channel_id][loc]
            if car_id != -1:
                car_object = CAR_DICT[car_id]
                assert car_object.__loc__() == loc, 'the loc of car is not fit'
                assert car_object.__channel_id__() == channel_id, 'the channel id of car is not fit'
                if car_object.__state__() == 2:#car stop state
                    pre_carState, pre_carLoc = 2, loc
                    continue
                if car_object.__state__() == 1:#car waiting sate
                    max_speed = min(self.speed, car_object.__speed__())
                    if loc + max_speed < pre_carLoc or pre_carLoc == -1: #no front car
                        if loc + max_speed <= self.length - 1:
                            self.cross_in_stack[channel_id][loc] = -1
                            self.cross_in_stack[channel_id][loc + max_speed] = car_id
                            car_object.update_data(state=2, loc=loc + max_speed)
                            pre_carState, pre_carLoc = 2, loc + max_speed
                        else: #car is ready to go across the cross
                            pre_carState, pre_carLoc = 1, loc
                            continue
                    else:  #front car
                        if pre_carState == 2: # stop front car
                            self.cross_in_stack[channel_id][loc] = -1
                            self.cross_in_stack[channel_id][pre_carLoc - 1] = car_id
                            car_object.update_data(state=2, loc=pre_carLoc - 1)
                            pre_carState, pre_carLoc = 2, pre_carLoc - 1
                        else: #the sate of the front car is waiting
                            pre_carState, pre_carLoc = 1, loc
                            continue
    
    #drive the car in the all channel to be end state or wait state 
    def Drive_Road(self):
        self.Road_Init()
        for channel_id in range(self.channel):
            self.Drive_CurChannel(channel_id)
    
    #find the first priority car in the road       
    def Find_FirstPriCar(self):
        assert self.cross_in_stack.all() != None, 'the cross_in_stack did not set'
        PriCarOfChannel = []
        for channel_id in range(self.channel):
            for loc in range(self.length-1, -1, -1):
                car_id = self.cross_in_stack[channel_id][loc]
                if car_id != -1:
                    car_object = CAR_DICT[car_id]
                    if car_object.__state__() == 1:
                        priority = car_object.__priority__()
                        PriCarOfChannel.append([car_id, priority, loc, channel_id])
                        break
        if PriCarOfChannel.__len__() == 0:
            return -1
        else:
            PriCarOfChannel.sort(key=lambda x: [-x[1], -x[2], x[3]]) #priority sort
            return PriCarOfChannel[0] #return the highest priority car data list 
    
    def FirstPriCar_Action(self, car_id, loc, channel_id, action):
        assert self.cross_in_stack.all() != None, 'the cross_in_stack did not set'
        car_object = CAR_DICT[car_id]
        if action == -1:   #stay at the end of the road
            self.cross_in_stack[channel_id][loc] = -1
            self.cross_in_stack[channel_id][self.length - 1] = car_id                
            car_object.update_data(state=2, loc=self.length - 1)
        elif action == 1:  #the car finished the road change
            self.cross_in_stack[channel_id][loc] = -1
            if car_object.__priority__() == 1:
                self.CrossIn_numCar[1] -= 1
            else:
                self.CrossIn_numCar[0] -= 1
            assert self.CrossIn_numCar[0] >= 0, 'the count of the road car is fault ' 
            
    def Recieve_Car(self, car_id, simulation=False):
        assert self.cross_out_stack.all() != None, 'the cross_out_stack did not set'
        car_object = CAR_DICT[car_id]
        if car_object.__nowRoad__() != None: #car on the road
            S1 = ROAD_DICT[car_object.__nowRoad__()].__length__() - car_object.__loc__() - 1
        else: #car out of the carpot
            S1 = 0
        SV2 = min(car_object.__speed__(), self.speed)
        if SV2 - S1 <= 0:
            return -1    #the car is failed in coming in the road
        for channel_id in range(self.channel):
            front_carid = self.Find_Car(0, SV2 - S1, channel_id)
            if front_carid == -1:
                if simulation == False:
                    self.cross_out_stack[channel_id][SV2 - S1 - 1] = car_id
                    car_object.update_data(state=2, loc=SV2-S1-1, channel_id=channel_id, nowRoad=self.road_id)
                    if car_object.__priority__() == 1:
                        self.CrossOut_numCar[1] += 1
                    else:
                        self.CrossOut_numCar[0] += 1
                return 1 #the car came in the road successfully
            else:
                front_car_object = CAR_DICT[front_carid]
                if front_car_object.__state__() == 1:
                    return 0  #come across the wait car
                elif front_car_object.__state__() == 2: #the state of the front is stop
                    if front_car_object.__loc__() == 0: #channel is full
                        continue
                    else: #channel is not full
                        if simulation == False:
                            self.cross_out_stack[channel_id][front_car_object.__loc__() - 1] = car_id
                            car_object.update_data(state=2, loc=front_car_object.__loc__()-1, channel_id=channel_id, nowRoad=self.road_id)
                            if car_object.__priority__() == 1:
                                self.CrossOut_numCar[1] += 1
                            else:
                                self.CrossOut_numCar[0] += 1
                        return 1  #the car came in the road successfully
        return -1 #the road is full of the car whose state is stop
                        
    def __id__(self):
        return self.road_id
    def __from__(self):
        return self.from_
    def __isDuplex__(self):
        return self.isDuplex
    def __channel__(self):
        return self.channel
    def __speed__(self):
        return self.speed
    def __length__(self):
        return self.length
    def __to__(self):
        return self.to_
    def __dispatch_done__(self):
        return self.dispatch_done
    def __forward_stack__(self):
        return self.forward_stack
    def __backward_stack__(self):
        return self.backward_stack
    def __cross_in_stack__(self):
        return self.cross_in_stack
    def __cross_out_stack__(self):
        return self.cross_out_stack
    def __forward_numCar__(self):
        return self.forward_numCar
    def __backward_numCar__(self):
        return self.backward_numCar
    
    

class CROSS:
    def __init__(self, id_, roadId0, roadId1, roadId2, roadId3):
        self.cross_id, self.road_set = id_, [roadId0, roadId1, roadId2, roadId3]
        self.direction_map = {roadId0:{roadId1:1, roadId2:2, roadId3:-1},\
                              roadId1:{roadId0:-1, roadId2:1, roadId3:2},\
                              roadId2:{roadId1:-1, roadId3:1, roadId0:2},\
                              roadId3:{roadId2:-1, roadId0:1, roadId1:2},} #-1, 1, 2 represent the turn right, left and straight
        self.carpot, self.car_retention, self.cross_Done, self.cross_update = {}, [], False, False
        self.validRoad = [road_id for road_id in self.road_set if road_id != -1]
        self.CrossInRoad, self.CrossOutRoad = [], []
        for roadId in self.validRoad:
            road_object = ROAD_DICT[roadId]
            if road_object.__to__() == self.cross_id or (road_object.__from__() == self.cross_id and road_object.__isDuplex__() == 1):
                self.CrossInRoad.append(roadId)
            if road_object.__from__() == self.cross_id or (road_object.__to__() == self.cross_id and road_object.__isDuplex__() == 1):
                self.CrossOutRoad.append(roadId)
    
    def RoadStackAdaptCross(self):
        for road_id in self.validRoad:
            self.GetDirOfRoad(road_id)
    
    def GetDirOfRoad(self, road_id):
        road_object = ROAD_DICT[road_id]
        road_object.adjust_road_direction(self.cross_id)
        
    def CrossDispatchInit(self):
        self.cross_Done = False
    
    #find the part of the possible dead lock loop
    def UpdateRoadMap(self):
        self.RoadStackAdaptCross()
        for road_id in self.CrossInRoad:
            road_object = ROAD_DICT[road_id]
            FirstPriCarList = road_object.Find_FirstPriCar()#get the first priCar of the road
            if id(road_object.__cross_in_stack__()) == id(road_object.__forward_stack__()):#get direction of the road
                direction = 0
            else:
                direction = 1
            if FirstPriCarList != -1:
                car_id = FirstPriCarList[0]
                FirstPriCarSequence[road_id][direction] = car_id
                car_object = CAR_DICT[car_id]
                if car_object.__nextroad_index__() < car_object.__route__().__len__():
                    next_roadId = car_object.__route__()[car_object.__nextroad_index__()]
                    next_roadObject = ROAD_DICT[next_roadId]
                    #get direction of the next road
                    if id(next_roadObject.__cross_out_stack__()) == id(next_roadObject.__forward_stack__()):
                        next_direction = 0
                    else:
                        next_direction = 1
                    #check the car whether the car on the next road would hinder it
                    result = next_roadObject.Recieve_Car(car_id, simulation=True)
                    if result == 0:#the hinder car exist
                        ROAD_MAP[road_id][direction] = [next_roadId, next_direction]
                        continue
            else:
                FirstPriCarSequence[road_id][direction] = -1
            ROAD_MAP[road_id][direction] = [-1, -1]
            
    def driveCarInWaitState(self):
        RoadDone = set()
        sorted_roadSet = sorted(self.CrossInRoad) #sort the road set according to the road id
        StopRoad = set()
        self.cross_update = False
        update_state = True
        while update_state == True:
            for stop_roadId in StopRoad:
                if stop_roadId in sorted_roadSet:
                    sorted_roadSet.remove(stop_roadId)
            update_state = False
            for road_id in sorted_roadSet:
                road_object = ROAD_DICT[road_id]
                while road_id not in RoadDone:
                    self.RoadStackAdaptCross()   #get the direction of the road
                    nowPricar_list = road_object.Find_FirstPriCar()
                    if nowPricar_list == -1:   #the road dispatch is done
                        RoadDone.add(road_id)
                        StopRoad.add(road_id)
                        continue
                    nowPricar_id, nowPricar_channelId, nowPricar_priority  = nowPricar_list[0], nowPricar_list[3], nowPricar_list[1]
                    nowCar_object = CAR_DICT[nowPricar_id]
                    if nowCar_object.__nextroad_index__() < nowCar_object.__route__().__len__():
                        nowCar_nextRoadId = nowCar_object.__route__()[nowCar_object.__nextroad_index__()]
                        assert nowCar_nextRoadId in self.road_set, 'the data of the cross or car object is fault'
                        nowCar_direction = self.direction_map[nowCar_object.__nowRoad__()][nowCar_nextRoadId]
                    else:  #the car is going to the end
                        nowCar_direction = 1
                        
                    for other_roadId in self.CrossInRoad:
                        if other_roadId == road_id:
                            continue
                        conflict = False
                        other_roadObject = ROAD_DICT[other_roadId]
                        other_roadObject.adjust_road_direction(self.cross_id)
                        other_pricarlist = other_roadObject.Find_FirstPriCar()
                        if other_pricarlist == -1:   #the road dispatch is done
                            RoadDone.add(other_roadId)
                            StopRoad.add(other_roadId)
                            continue 
                        otherPricar_id, otherPricar_priority = other_pricarlist[0], other_pricarlist[1]
                        otherCar_object = CAR_DICT[otherPricar_id]
                        if otherCar_object.__nextroad_index__() < otherCar_object.__route__().__len__():
                            otherCar_nextRoadId = otherCar_object.__route__()[otherCar_object.__nextroad_index__()]
                            assert otherCar_nextRoadId in self.road_set, 'the data of the cross or car object is fault'
                            otherCar_direction = self.direction_map[otherCar_object.__nowRoad__()][otherCar_nextRoadId]
                        else:  #the car is going to the end
                            otherCar_direction = 1
                        #check whether the same next road exist between different car 
                        if (nowCar_direction + self.road_set.index(road_id)) % 4 == \
                        (otherCar_direction + self.road_set.index(other_roadId)) % 4:   
                            if otherPricar_priority == nowPricar_priority: 
                                if nowCar_direction < otherCar_direction:    #direction conflict exist
                                    conflict = True
                                    break
                            elif otherPricar_priority > nowPricar_priority: #priority conflict exist
                                conflict = True
                                break
                    if conflict:  #conflict exist
                        break
                    else:#no conflict
                        loc = nowCar_object.__loc__()
                        if nowCar_object.__nextroad_index__() == nowCar_object.__route__().__len__():  #the car is going to the end
                            road_object.FirstPriCar_Action(nowPricar_id, loc, nowPricar_channelId, 1)
                            nowCar_object.update_data(end_time=sysTIME)
                            FINISHED_CAR.append(nowPricar_id)
                        else:
                            next_roadObject = ROAD_DICT[nowCar_object.__route__()[nowCar_object.__nextroad_index__()]]
                            dispatch_result = next_roadObject.Recieve_Car(nowPricar_id)
                            if dispatch_result != 0:  #the car is dispatched sucessfully
                                road_object.FirstPriCar_Action(nowPricar_id, loc, nowPricar_channelId, dispatch_result)
                            else:   #the nowCar is disturbed by the waiting car in the next road
                                StopRoad.add(road_id)
                                break
                        self.cross_update = True
                        update_state = True
                        road_object.Drive_CurChannel(nowPricar_channelId)
# =============================================================================
#                         program update
# =============================================================================
                        preCrossId = road_object.__from__() if road_object.__to__() == self.cross_id else road_object.__to__()
                        preCrossObject = CROSS_DICT[preCrossId]
                        preCrossObject.DriveCarInitList(sysTIME, priority=True,dest_cross = self.cross_id)
                        
        if RoadDone.__len__() == self.CrossInRoad.__len__():
            self.cross_Done = True

    def DriveCarInitList(self, time_plan, priority=False,dest_cross = None):
        assert priority in [True, False], 'the input of priority is fault'
        self.RoadStackAdaptCross()
        if time_plan in self.carpot.keys():  #priority sort
            self.car_retention.extend(self.carpot[time_plan])
            del self.carpot[time_plan]
            self.car_retention.sort(key=lambda x: [-x[1], x[2], x[0]])
        if priority == False:
            i = 0
            while self.car_retention.__len__() - i > 0:
                car_id = self.car_retention[i][0]
                car_object = CAR_DICT[car_id]
                road_object = ROAD_DICT[car_object.__route__()[0]]
                result = road_object.Recieve_Car(car_id)
                assert result != 0, 'it cant be that some car waiting in the road in this condition'
                if result == -1:  #the road is full
                    i += 1
                    continue
                CARonGOING.append(self.car_retention[i][0])
                car_object.update_data(start_time=sysTIME)
                del self.car_retention[i]
        elif priority == True and dest_cross == None:
              #dipatch the priority only
            if self.car_retention.__len__() == 0:
                return
            i = 0
            while self.car_retention.__len__() - i > 0 and self.car_retention[i][1] == 1:#the priority car exist
                car_id = self.car_retention[i][0]
                car_object = CAR_DICT[car_id]
                road_object = ROAD_DICT[car_object.__route__()[0]]
                result = road_object.Recieve_Car(car_id)
                if result != 1:   #the road is full or the waiting front car exist
                    i += 1
                    continue
                car_object.update_data(start_time=sysTIME)
                CARonGOING.append(self.car_retention[i][0])
                del self.car_retention[i]
        elif priority == True and dest_cross != None:
            if self.car_retention.__len__() == 0:
                return
            i = 0
            while self.car_retention.__len__() - i > 0 and self.car_retention[i][1] == 1:#the priority car exist
                car_id = self.car_retention[i][0]
                car_object = CAR_DICT[car_id]
                road_object = ROAD_DICT[car_object.__route__()[0]]
                if road_object.__from__ == dest_cross or road_object.__to__ == dest_cross:#the car's next dest is dest_cross
                    result = road_object.Recieve_Car(car_id)
                    if result != 1:   #the road is full or the waiting front car exist
                        i += 1
                        continue
                    car_object.update_data(start_time=sysTIME)
                    CARonGOING.append(self.car_retention[i][0])
                    del self.car_retention[i]
                else:
                    i += 1
                    continue 
    def CarInTheCarpot(self, time_plan, car_id, car_priority):
        if time_plan not in self.carpot.keys():
            self.carpot[time_plan] = [[car_id, car_priority, time_plan]]
        else:
            self.carpot[time_plan].append([car_id, car_priority, time_plan])
    
    
    def __id__(self):
        return self.cross_id
    def __road_set__(self):
        return self.road_set
    def __direction_map__(self):
        return self.direction_map
    def __carpot__(self):
        return self.carpot
    def __cross_Done__(self):
        return self.cross_Done
    def __cross_update__(self):
        return self.cross_update
    def __validRoad__(self):
        return self.validRoad
    def __CrossInRoad__(self):
        return self.CrossInRoad
    def __CrossOutRoad__(self):
        return self.CrossOutRoad

def FindDeadLockCar(DeadLockCrossList):
    DeadLockCarList = {}
    for cross_id in DeadLockCrossList:
        DeadLockCarList[cross_id] = {}
        cross_object = CROSS_DICT[cross_id]
        cross_object.RoadStackAdaptCross()    #get the direction of the road accroding to the cross
        for road_id in cross_object.__CrossInRoad__():
            road_object = ROAD_DICT[road_id]
            road_matrix = road_object.__cross_in_stack__()
            for channel_id in range(road_object. __channel__()):
                DeadLockCarList[cross_id][channel_id] = []
                for loc in range(road_object. __length__() -1, -1, -1):
                    if road_matrix[channel_id][loc] != -1:
                        DeadLockCarList[cross_id][channel_id].append([road_matrix[channel_id][loc], loc])
    return DeadLockCarList

def DestroyDeadLock():
    global DEADLOCKLOOPLOG, NOMOVECAR
# =============================================================================
#     predict the possible dead lock
# =============================================================================
    for cross_id in CROSS_NAMESPACE:
        cross_object = CROSS_DICT[cross_id]
        cross_object.UpdateRoadMap()  #prepare for the dead lock loop check
    visited_road = []
    deadLockLoop = []
    MaybeLoop = []
    for road_id in ROAD_MAP.keys():
        for direction in range(ROAD_MAP[road_id].__len__()):
            if MaybeLoop.__len__() >= 3:  #collect the loop chain whose length is enough long
                deadLockLoop.append(MaybeLoop)
            MaybeLoop = []
            start_road = [road_id, direction]
            while start_road[0] != -1 and start_road not in visited_road:
                visited_road.append(start_road)
                MaybeLoop.append(start_road)
                start_road = ROAD_MAP[start_road[0]][start_road[1]]
# =============================================================================
#         adjust the route of the car whose route is intersect with deadlock loop
# =============================================================================
    DEADLOCKLOOPLOG.append(deadLockLoop)
    for loopList in deadLockLoop:
        i = 0
        while i < loopList.__len__() - 1:
            road_id, direction = loopList[i][0], loopList[i][1]
            car_id = FirstPriCarSequence[road_id][direction]  #get the first priCar
            if car_id not in NOMOVECAR:
                NOMOVECAR.add(car_id)
                car_object = CAR_DICT[car_id]
                #get the cross id which the car is waiting for
                cross_id = road_file[road_id][4] if direction == 0 else road_file[road_id][3]
                lantentRoadList = []
                for cross_list in cross_file[cross_id]:
                    if cross_list[1] != -1 and cross_list[1] != loopList[i+1][0] and cross_list[1] != road_id:
                        road_object = ROAD_DICT[cross_list[1]]
                        numCar = road_object.__forward_numCar__() if cross_list[3] == 0 else road_object.__backward_numCar__()
                        nextCrossId = road_object.__to__() if cross_list[3] == 0 else road_object.__from__()
                        numCar = sum(numCar)
                        lantentRoadList.append([road_object.__id__(), numCar, cross_id, nextCrossId])
                lantentRoadList.sort(key=lambda x: x[1])
                for tmp in lantentRoadList:
                    prePath = car_object.__route__()[:car_object.__nextroad_index__()] + [tmp[0]]
                    start_node = tmp[3]
                    end_node = car_object.__to__()
                    from_node = car_object.__from__()
                    updateRoadPath = Dijkstra_Ver2(start_node, cross_file, road_file, end_node, prePath, from_node)
                    if updateRoadPath == -1:
                        continue
                    if updateRoadPath != None:
                        updateRoadPath = [tmp[0]] + updateRoadPath
                    else:
                        updateRoadPath = [tmp[0]]
                    car_object.update_route(updateRoadPath)
                    break
            i += 1

def Map_Dispatch():
    global sysTIME, DEADLOCKCAR, DEADLOCKLOOPLOG, NOMOVECAR
    global TotalNumOfPriCar, TimeFirstPriCarOut, TimeLastPriCarOut, TimeLastPriCarEnd, MaxSpeedOfPriCar, MinSpeedOfPriCar
    global DepartureOfPriCar, DestinationOfPriCar
    global TotalNumOfUsualCar, TimeFirstUsualCarOut, TimeLastUsualCarOut, MaxSpeedOfUsualCar, MinSpeedOfUsualCar
    global DepartureOfUsualCar, DestinationOfUsualCar
    UPDATE_FLAG, DEADLOCK_FLAG = False, False
# =============================================================================
#    put the car in carpot 
# =============================================================================
    for car_id, car_object in CAR_DICT.items():
        fromId = car_object.__from__()
        carPriority = car_object.__priority__()
        planTime = car_object.__planTime__()
        cross_object = CROSS_DICT[fromId]
        cross_object.CarInTheCarpot(planTime, car_id, carPriority)
# =============================================================================
#    check the dispatch is done
# =============================================================================
    while FINISHED_CAR.__len__() != car_file.__len__():
        sysTIME += 1
# =============================================================================
#    drive all the road  
# =============================================================================
        for cross_id in CROSS_NAMESPACE:
            cross_object = CROSS_DICT[cross_id]
            cross_object.RoadStackAdaptCross()
            for road_id in cross_object.__CrossInRoad__():
                road_object = ROAD_DICT[road_id]
                road_object.Drive_Road()
# =============================================================================
#    drive the priority car
# =============================================================================
        for cross_id in CROSS_NAMESPACE:
            cross_object = CROSS_DICT[cross_id]
            cross_object.DriveCarInitList(sysTIME, priority=True)
# =============================================================================
#    initialize the cross
# =============================================================================
        for cross_id in CROSS_NAMESPACE:
            cross_object = CROSS_DICT[cross_id]
            cross_object.CrossDispatchInit()
# =============================================================================
#          drive the car whose is in waiting state   
# =============================================================================
        unFinishedCross = CROSS_NAMESPACE.copy()
        NOMOVECAR = set()
        while unFinishedCross.__len__() > 0:                
            UPDATE_FLAG = False
            #DestroyDeadLock()
            for cross_id in CROSS_NAMESPACE:
                if cross_id in unFinishedCross:
                    cross_object = CROSS_DICT[cross_id]
                    cross_object.driveCarInWaitState()
                    if cross_object.__cross_Done__():
                        unFinishedCross.remove(cross_id)
                    if cross_object.__cross_update__():
                        UPDATE_FLAG = True
                    #print(cross_id, cross_object.__cross_update__(), cross_object.__cross_Done__())
                    
            if UPDATE_FLAG == False and unFinishedCross.__len__() > 0:
                DEADLOCK_FLAG = True
            if DEADLOCK_FLAG == True:
                print('dead lock exist, the dead lock cross is:')
                print(unFinishedCross)
#                DEADLOCKCAR = FindDeadLockCar(unFinishedCross)
#                print(DEADLOCKCAR)
                assert DEADLOCK_FLAG == False
# =============================================================================
#     drive the car into the road
# =============================================================================
        for cross_id in CROSS_NAMESPACE:
            cross_object = CROSS_DICT[cross_id]
            cross_object.DriveCarInitList(sysTIME, priority=False)
        print('time %d is dispatch done...' % (sysTIME))
    print('All Dispatch is Done!')
# =============================================================================
#     calculte the system dispatch time according to the rule
# =============================================================================
    for car_id, car_object in CAR_DICT.items():
        car_priority = car_object.__priority__()
        start_time, end_time = car_object.__min_plantime__(), car_object.__end_time__()
        car_speed = car_object.__speed__()
        if car_priority == 1:
            if start_time < TimeFirstPriCarOut:
                TimeFirstPriCarOut = start_time
            if start_time > TimeLastPriCarOut:
                TimeLastPriCarOut = start_time
            if TimeLastPriCarEnd < end_time:
                TimeLastPriCarEnd = end_time
            if car_speed > MaxSpeedOfPriCar:
                MaxSpeedOfPriCar = car_speed
            if car_speed < MinSpeedOfPriCar:
                MinSpeedOfPriCar = car_speed
        if start_time < TimeFirstUsualCarOut:
            TimeFirstUsualCarOut = start_time
        if start_time > TimeLastUsualCarOut:
            TimeLastUsualCarOut = start_time
        if car_speed > MaxSpeedOfUsualCar:
            MaxSpeedOfUsualCar = car_speed
        if car_speed < MinSpeedOfUsualCar:
            MinSpeedOfUsualCar = car_speed
    
    coefficient_a = 0.05 * round((TotalNumOfUsualCar / TotalNumOfPriCar),5) + \
    0.2375 *round(round(MaxSpeedOfUsualCar / MinSpeedOfUsualCar,5) / round(MaxSpeedOfPriCar / MinSpeedOfPriCar,5),5) + \
    0.2375 * round(round(TimeLastUsualCarOut / TimeFirstUsualCarOut,5) / round(TimeLastPriCarOut / TimeFirstPriCarOut,5),5) + \
    0.2375 * round(DepartureOfUsualCar.__len__() / DepartureOfPriCar.__len__(),5) + \
    0.2375 * round(DestinationOfUsualCar.__len__() / DestinationOfPriCar.__len__(),5)
    
    Tpri = TimeLastPriCarEnd - TimeFirstPriCarOut
    Te = coefficient_a * Tpri + sysTIME
    print('origin schedule Time is %d, special schedule Time is %d, CodeCraftJudge end schedule time is %d' % \
          (sysTIME, Tpri, Te))
    
        
if __name__ == "__main__":
    car_path = '../config/car.txt'
    road_path = '../config/road.txt'
    cross_path = '../config/cross.txt'
    preset_path = '../config/presetAnswer.txt'
    answer_path = '../config/answer.txt'
    start1 = time.time()
# =============================================================================
#     get answer file
# =============================================================================
    answer_file = read_file(answer_path)
    preset_answer_file = read_file(preset_path)
# =============================================================================
#    get road file , road dict and road map
# =============================================================================
    road_file = read_file(road_path)
    for road_id, road_data in road_file.items():
        ROAD_DICT[road_id] = ROAD(road_id, road_data[0], road_data[1], road_data[2], road_data[3], road_data[4], road_data[5])
        ROAD_NAMESPACE.append(road_id) 
        ROAD_MAP[road_id] = {}
        FirstPriCarSequence[road_id] = {}
        for i in range(road_data[5] + 1):
            ROAD_MAP[road_id][i] = []
            FirstPriCarSequence[road_id][i] = -1
# =============================================================================
#     get cross file and cross dict
# =============================================================================
    cross_file = read_file(cross_path)
    for cross_id, cross_data in cross_file.items():
        CROSS_DICT[cross_id] = CROSS(cross_id, cross_data[0], cross_data[1], cross_data[2], cross_data[3])
        CROSS_NAMESPACE.append(cross_id)
# =============================================================================
#     get car file and car dict   
# =============================================================================
    car_file = read_file(car_path)
    for car_id, car_data in car_file.items():
        if car_data[4] == 1:
            TotalNumOfPriCar += 1   #count the num of priority car
            DepartureOfPriCar.add(car_data[0])  #log the place of departure
            DestinationOfPriCar.add(car_data[1])  #log the place of destination
        TotalNumOfUsualCar += 1    #count the num of usual car 
        DepartureOfUsualCar.add(car_data[0])
        DestinationOfUsualCar.add(car_data[1])
        CAR_DICT[car_id] = CAR(car_id, car_data[0], car_data[1], car_data[2], car_data[3], car_data[4], car_data[5])
        if car_data[5] == 1:  #preset car:
            CAR_DICT[car_id].get_route(preset_answer_file[car_id][0], preset_answer_file[car_id][1:])
        else:
            CAR_DICT[car_id].get_route(answer_file[car_id][0], answer_file[car_id][1:])
        CAR_NAMESPACE.append(car_id)
# =============================================================================
#    update cross file
# =============================================================================
    update_cross(cross_file, road_file)
    CROSS_NAMESPACE.sort()
# =============================================================================
#     sort the car into its cross dict
# =============================================================================
    #cross_garage = Map_Stimulation(car_file, cross_file, road_file)
# =============================================================================
#   Map dispatch
# =============================================================================
    Map_Dispatch()
    end1 = time.time()
    print('run time:',end1-start1)#运行时间
# =============================================================================
#    generate the example answer txt
# ============================================================================= 
#    ALL_ROAD_PATH = Get_All_Shortest_Path(cross_file)
#    preset_file = read_file(preset_path)
#    f = open(answer_path,"w+") 
#    for car_id, tmp in car_file.items():
#        if tmp[5] != 1:
#            start_time = tmp[3]
#            route = ALL_ROAD_PATH[tmp[0]][tmp[1]]
#            answer_list = [car_id, start_time, route]
#            f.write("(")
#            temp_str = str(answer_list)
#            temp_str = temp_str.replace('[', '')
#            temp_str = temp_str.replace(']', '')
#            f.write(temp_str)
#            f.write(")\n") 