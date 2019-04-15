import copy
from collections import deque

class GRAPH(object):
    def __init__(self,edge_list):
        for edge_item in edge_list:
            assert len(edge_item) == 3, print("edge_list not legal") 
        self.edge_list = edge_list
    
    #返回点的集合
    @property
    def vertex_set(self):
        vertex_set = set()
        for edge_item in self.edge_list:
            vertex_set.add(edge_item[0])
            vertex_set.add(edge_item[1])
        return vertex_set
    
    @property
    def neighbour_set(self):
        neighbour_set = {}
        for vertex_item in self.vertex_set:
            neighbour_set[vertex_item] = set()
        for edge_item in self.edge_list:
            neighbour_set[edge_item[0]].add((edge_item[1],edge_item[2]))
        return neighbour_set
    
    def get_path(self,src,dest_list):
        if src not in self.vertex_set:
            assert True,print("error:no sucn src in the graph!")
        
        distances = {}
        for vertex_item in self.vertex_set:
            distances[vertex_item] = float('inf')        
        distances[src] = 0
        vertex_set = copy.deepcopy(self.vertex_set)

        previous_vertex_set = {}
        for vertex_item in self.vertex_set:
            previous_vertex_set[vertex_item] = None

        while vertex_set:
            present_vertex = min(vertex_set,key = lambda vertex_item: distances[vertex_item])
            if distances[present_vertex] == float('inf'):
                break    
            for neighbour ,weigth in self.neighbour_set[present_vertex]:
                temp_distances = weigth + distances[present_vertex]
                if distances[neighbour] > temp_distances:
                    distances[neighbour] = temp_distances
                    previous_vertex_set[neighbour] = present_vertex
            vertex_set.remove(present_vertex)

        path = {} #目的节点：最短路径
        for i in range(len(dest_list)):
            path_one, present_vertex = deque(), dest_list[i]
            if present_vertex in previous_vertex_set:
                while previous_vertex_set[present_vertex] is not None:
                    path_one.appendleft(present_vertex)
                    present_vertex = previous_vertex_set[present_vertex]
                if path_one:
                    path_one.appendleft(present_vertex)
                path[dest_list[i]]=path_one
            else:
                path[dest_list[i]]=[]
        #print('distances:',distances)# shortest dist of every node from src 

        return path#格式: {dest_list1:[cross_id1,cross_id2...], dest_list2:[cross_id1,cross_id2...],...}



