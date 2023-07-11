from common import *
import numpy as np
import copy
import time
from math import sqrt, cos, sin
from scipy.spatial import KDTree
import random

class Planner:
    def __init__(self ,height, width, map_data, planner_type="JPS"):
        self.map = MapGrid(height, width)
        self.height = height
        self.width = width     
        self.planner_type = planner_type
        for i in range(height):
            for j in range(width):
                if map_data[i][j] > 50:
                    self.map.set_grid(j, i, "obstacle")
        
    def planning(self, start_point, end_point):

        print("*"*20)
        print("planner: ", self.planner_type)
        print("start: ", start_point.x, start_point.y, " end: ", end_point.x, end_point.y)
        
        map = copy.deepcopy(self.map)
        map.set_grid(start_point.x, start_point.y, "origin")
        map.set_grid(end_point.x, end_point.y, "goal")
        
        planner = None
        if self.planner_type == "JPS":
            planner = JPS(start_point, end_point, map)
        elif self.planner_type == "A*":
            planner = A_star(start_point, end_point, map)
        elif self.planner_type == "RRT*":
            obs = Obstacle(map)
            planner = RRT_Star(obs)

        t = time.time()
        if self.planner_type == "RRT*":
            arrived, path = planner.Process(end_point, start_point)
        else:
            arrived, path = planner.Process()      
        print(f"time: {time.time()-t:.3f}s")  
        plan_rx = []
        plan_ry = []
        if self.planner_type == "RRT*":
            for i in range(len(path)):
                plan_rx.append((path[i].x - 64) / 6.4) 
                plan_ry.append((path[i].y - 64) / 6.4)
        else:
            for i in range(len(path)):
                plan_rx.append((path[i].point.x - 64) / 6.4)
                plan_ry.append((path[i].point.y - 64) / 6.4)
        if plan_rx and len(plan_rx) > 1:
            total_len = sum([sqrt((plan_rx[i]-plan_rx[i+1])**2 + (plan_ry[i]-plan_ry[i+1])**2) for i in range(len(plan_rx)-1)])
            print(f"totol length(in pixel): {total_len:.3f}")
        else:
            print("totol length(in pixel): 0")
        
        print("*"*20)

        return plan_rx, plan_ry
    
class JPS:  
    def __init__(self, start_point, end_point, map):

        self.explored=[]   # explored points
        self.open = []  # list of points to be explored
        self.now = 0       # Vertex now
        self.start_point = start_point
        self.end_point = end_point
        self.map = map    
        self.end_Vertex = 0
        self.max_iter = 10000

        # print("start: ", start_point.x, start_point.y)
        # print("end: ", end_point.x, end_point.y)

    class Vertex: 

        def __init__(self, point, endpoint, g):
            self.point = point  
            self.endpoint = endpoint  
            self.father = None        
            self.g = g                
            self.h = abs(endpoint.y - point.y) + abs(endpoint.x - point.x)
            self.f = self.g + self.h

        def neibor(self, horizonal, vertical):
            nextpoint = Point(self.point.x + horizonal, self.point.y + vertical)
            if vertical != 0 and horizonal != 0:  
                nearVertex = JPS.Vertex(nextpoint, self.endpoint, self.g + 1.414)  
            else: 
                nearVertex = JPS.Vertex(nextpoint, self.endpoint, self.g + 1)    
            return nearVertex 

        def get_direction(self):
            x = self.father and self.father.point.x - self.point.x and -(self.father.point.x - self.point.x)/abs(self.father.point.x - self.point.x) or 0
            y = self.father and self.father.point.y - self.point.y and -(self.father.point.y - self.point.y)/abs(self.father.point.y - self.point.y) or 0
            return Vector(x ,y)

    def search_next(self, Vertex, horizonal, vertical): 
        nearVertex = Vertex.neibor(horizonal, vertical)  
        if not self.is_obstacle(nearVertex): 
            return nearVertex
        else :
            return None    

    def direction(self, x1, x2):
        return x1-x2 and (x1 - x2)/abs(x1 - x2) or 0
        
    def Process(self): 
        start_Vertex = self.Vertex(self.start_point, self.end_point, 0)
        self.open.append(start_Vertex)
        arrive = 0
        count = 0
        while arrive != 1 and count < self.max_iter:
            self.now = self.min_cost_Vertex()    
            if self.now:
                arrive = self.prune_and_jump(self.now)
                count += 1
            else:
                break
        
        print("searched: ", len(self.explored))

        if arrive != 1:
            print("Not found!")
            return [], []
        
        # get jump points
        jump = []
        temp_Vertex = self.end_Vertex.father  
        while (temp_Vertex.point.x != self.start_point.x or temp_Vertex.point.y != self.start_point.y):
            jump.append(temp_Vertex)
            temp_Vertex = temp_Vertex.father
        jump.reverse()

        # get path
        path=[]
        now = self.end_Vertex
        father = self.end_Vertex.father
        dir = self.end_Vertex.get_direction()
        while (now.point.x != self.start_point.x or now.point.y != self.start_point.y):
            while (now.point.x != father.point.x or now.point.y != father.point.y):
                if not self.is_end(now):
                    path.append(now)
                now = now.neibor(-dir.x, -dir.y)
            now = father
            father = now.father
            dir = now.get_direction()

        #path.reverse()
        return self.explored, path 

    def min_cost_Vertex(self):
        if len(self.open)==0:
            return None

        Vertex_min = self.open[0]
        for Vertex_temp in self.open:
            if Vertex_temp.f < Vertex_min.f:
                Vertex_min = Vertex_temp
        self.close(Vertex_min)
        return Vertex_min

    def is_open(self, Vertex):
        for Vertex_temp in self.open:
            if Vertex_temp.point == Vertex.point:
                return Vertex_temp  
        return 0
    
    def is_closed(self, Vertex):
        if Vertex.point.x<0 or Vertex.point.y<0 or Vertex.point.x>=self.map.width or Vertex.point.y>=self.map.height:
            return 1
        if self.map.data[Vertex.point.y][Vertex.point.x] == 5: # explored
            return 1
        if self.map.data[Vertex.point.y][Vertex.point.x] == 2: # start
            return 1
        return 0

    def close(self, Vertex):
        self.open.remove(Vertex)
        self.explored.append(Vertex)
        # avoid duplicate
        data = self.map.data[Vertex.point.y][Vertex.point.x]
        if data==2 or data==10 or data==1: 
            return 
        self.map.set_grid(Vertex.point.x, Vertex.point.y, "explored")
        
                
    def is_obstacle(self, Vertex):
        if Vertex.point.x<0 or Vertex.point.y<0 or Vertex.point.x>=self.map.width or Vertex.point.y>=self.map.height:
            return 1
        if self.map.data[Vertex.point.y][Vertex.point.x] == 10:
            return 1
        return 0

    def is_end(self, Vertex):
        return Vertex.point.x == self.end_point.x and Vertex.point.y == self.end_point.y

    def prune_and_jump(self, Vertex): 

        neibors=[]# append possible neibors 
        temp_Vertex=[]# append all neibors
        if Vertex.father:
            d = Vertex.get_direction()

            #put the original direction first
            temp_Vertex.append(self.search_next(Vertex, d.x, d.y))
            
            if d.x!=0 and d.y!=0:#if diagonal
                temp_Vertex.append(self.search_next(Vertex, d.x, 0))
                temp_Vertex.append(self.search_next(Vertex, 0, d.y))

                if self.is_obstacle(Vertex.neibor(-d.x, 0)):
                    temp_Vertex.append(self.search_next(Vertex, -d.x, d.y))
                if self.is_obstacle(Vertex.neibor(0, -d.y)):
                    temp_Vertex.append(self.search_next(Vertex, d.x, -d.y))    

            if d.x!=0 and d.y==0:#if horizonal
                if self.is_obstacle(Vertex.neibor(0, 1)):
                    temp_Vertex.append(self.search_next(Vertex, d.x, 1))     
                if self.is_obstacle(Vertex.neibor(0, -1)):
                    temp_Vertex.append(self.search_next(Vertex, d.x, -1))       

            if d.x==0 and d.y!=0:#if vertical
                if self.is_obstacle(Vertex.neibor(-1, 0)):
                    temp_Vertex.append(self.search_next(Vertex, -1, d.y))     
                if self.is_obstacle(Vertex.neibor(1, 0)):
                    temp_Vertex.append(self.search_next(Vertex, 1, d.y))                            

        else:
            all_dir = [[1, 0], [0, 1], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
            for d in all_dir :
                temp_Vertex.append(self.search_next(Vertex, d[0], d[1]))

        for t in temp_Vertex:
            if t:
                neibors.append(t)

        for n in neibors:
            jump_node = self.jump_node(n, Vertex)
            if jump_node :
                #if in open list, then update the old one
                pre=self.is_open(jump_node) 
                if  pre and pre.g>jump_node.g:
                    pre.g = jump_node.g
                    pre.f = jump_node.f
                    pre.father = Vertex

                #if not in open list and not in closed list, then add to open list
                if not pre and not self.is_closed(jump_node): 
                    jump_node.father = Vertex
                    self.open.append(jump_node)
                if self.is_end(jump_node):
                    self.end_Vertex = jump_node
                    return 1
        return 0


    def jump_node(self, now, pre):  
        
        # search the jump point in the same direction
        
        # if return None, then no jump point
        # else return the jump point
        
        # what is jump point?
        # 1. start or end point
        # 2. it has a forced neighbor(it has obs in one direction and not in the other)
        # 3. it has a jump point in horizontal or vertical direction if it is diagonal

        if not now or self.is_closed(now) or self.is_obstacle(now):
            return None
        
        d = Vector(self.direction(now.point.x, pre.point.x), self.direction(now.point.y, pre.point.y))
        
        # 1.
        if self.is_end(now):
            return now
        
        # 2.
        if d.x!=0 and d.y!=0:
            if (self.search_next(now, -d.x,d.y) and self.is_obstacle(now.neibor(-d.x, 0)))or (self.search_next(now, d.x,-d.y) and self.is_obstacle(now.neibor(0, -d.y))):
                return now
        if d.x!=0 and d.y==0:
            if (self.search_next(now, d.x, 1) and self.is_obstacle(now.neibor(0, 1)))or (self.search_next(now, d.x,-1) and self.is_obstacle(now.neibor(0, -1))):
                return now
        if d.x==0 and d.y!=0:
            if (self.search_next(now, 1, d.y) and self.is_obstacle(now.neibor(1, 0)))or (self.search_next(now, -1, d.y) and self.is_obstacle(now.neibor(-1, 0))):
                return now

        # 3.
        if d.x!=0 and d.y!=0:
            temp1 = self.jump_node(self.search_next(now, d.x, 0), now)
            temp2 = self.jump_node(self.search_next(now, 0, d.y), now)
            if temp1 or temp2:
                return now
            
        # return the jump point in the same direction
        temp = self.jump_node(self.search_next(now, d.x, d.y), now)
        if temp:
            return temp 
        
        return None

class A_star:  

    def __init__(self, start_point, end_point, map):
        self.arrived=[]   # explored points, same as closed list
        self.closed = []  # explored points
        self.open = []  # to be explored
        self.now = 0     
        self.start_point = start_point
        self.end_point = end_point
        self.map = map    
        self.end_Vertex = 0
        self.max_iter = 10000

    class Vertex: 

        def __init__(self, point, endpoint, g):
            self.point = point        
            self.endpoint = endpoint  
            self.father = None        
            self.g = g              
            self.h = abs(endpoint.y - point.y) + abs(endpoint.x - point.x) 
            self.f = self.g + self.h

        def search_next(self, vertical, horizontal):
            nextpoint = Point(self.point.x + horizontal, self.point.y + vertical)
            if vertical != 0 and horizontal != 0: 
                nearVertex = A_star.Vertex(nextpoint, self.endpoint, self.g + 1.414)  
            else:   
                nearVertex = A_star.Vertex(nextpoint, self.endpoint, self.g + 1)     
            return nearVertex

    def Process(self):
        start_Vertex = self.Vertex(self.start_point, self.end_point, 0)
        self.open.append(start_Vertex)
        arrive = 0
        count = 0

        while arrive != 1 and count < self.max_iter:

            self.now = self.min_cost_Vertex() # get the Vertex with the lowest cost  
            
            if count > 0 and self.now:
                self.arrived.append(self.now) 
            elif count > 0:
                break

            arrive = self.explore_next(self.now) # expand the open list
            count += 1

        best_path = []

        print("searched: ", len(self.arrived))
        if self.end_Vertex == 0:
            print("No solution!!")
            return [], []
        
        temp_Vertex = self.end_Vertex.father 
        while (temp_Vertex.point.x != self.start_point.x or temp_Vertex.point.y != self.start_point.y):
            best_path.append(temp_Vertex)
            temp_Vertex = temp_Vertex.father

        return self.arrived, best_path

    def min_cost_Vertex(self):
        if len(self.open)==0:
            return None
        Vertex_min = self.open[0]
        for Vertex_temp in self.open:
            if Vertex_temp.f < Vertex_min.f:
                Vertex_min = Vertex_temp
        self.open.remove(Vertex_min)
        self.closed.append(Vertex_min)
        return Vertex_min

    def is_open(self, Vertex):
        for Vertex_temp in self.open:
            if Vertex_temp.point == Vertex.point:
                return Vertex_temp  
        return 0

    def is_closed(self, Vertex):
        for closeVertex_temp in self.closed:
            if closeVertex_temp.point == Vertex.point:
                return 1
        return 0

    def is_obstacle(self, Vertex):
        if self.map.data[Vertex.point.y][Vertex.point.x] == 10:
            return 1
        return 0

    def explore_next(self, Vertex):

        for i in range(-1, 2):
            for j in range(-1, 2):
                if (i!=0 or j!=0) and Vertex.point.x+i>=0 and Vertex.point.y+j>=0 and Vertex.point.x+i<=self.map.width-1 and Vertex.point.y+j<=self.map.height-1:
                    temp_Vertex = Vertex.search_next(i,j)
                    if self.is_obstacle(temp_Vertex)==0 and self.is_closed(temp_Vertex)==0:
                        # if in open list, update it
                        flag = 0
                        for temp_Vertex1 in self.open:
                            if temp_Vertex1.point == temp_Vertex.point:
                                if temp_Vertex1.g > temp_Vertex.g:
                                    temp_Vertex1.g = temp_Vertex.g
                                    temp_Vertex1.f = temp_Vertex.f
                                    temp_Vertex1.father = Vertex
                                flag = 1# in open list

                        #if not in open list , add to open list
                        if flag==0:
                            temp_Vertex.father = Vertex
                            self.open.append(temp_Vertex)
                            if temp_Vertex.point.x == self.end_point.x and temp_Vertex.point.y == self.end_point.y:
                                self.end_Vertex = temp_Vertex
                                return 1 
        return 0
    


class RRT_Star():
    def __init__(self, obs) -> None:
        self.search_tree = []
        self.search_list = []
        self.path = []
        self.new_path = []
        self.start = None
        self.end = None
        self.obs = obs
        self.dynamic = False
        self.end_probility = 0.3
        self.step_len = 10
        self.min_dis = 1
        self.sample_num = 2000
        self.x_bound = [0, 127]
        self.y_bound = [0, 127]
        

    def expand(self, temp_node, father_idx):
        #expand for one step 
        father_node = self.search_list[father_idx].data
        new_node = father_node + self.step_len * (temp_node - father_node)/(temp_node - father_node).mod()
        return new_node

    def tree_grow(self, use_KDTree = False):
        for i in range(self.sample_num):

            # check for the stop 
            end_idx = len(self.search_list) - 1
            end_node = self.search_list[end_idx]
            if end_node.data.dis(self.end)<self.min_dis or self.obs.line_check(end_node.data, self.end, self.dynamic):
                self.search_list.append(Tree(self.end, end_idx, end_node.dist + end_node.data.dis(self.end)))
                print("search step: ", i)
                return True

            # sample 
            if random.random() > self.end_probility:
                # point_s = Point(random.uniform(self.x_bound[0], self.x_bound[1]), random.uniform(self.y_bound[0], self.y_bound[1]))
                point_s = Point(int(random.uniform(self.x_bound[0], self.x_bound[1])), int(random.uniform(self.y_bound[0], self.y_bound[1])))
            else:
                point_s = self.end

            # search for the nearst.If we use ikdtree ,it'll be faster
            if use_KDTree:
                self.search_tree = KDTree([[node.data.x, node.data.y] for node in self.search_list])
                _, idx = self.search_tree.query([point_s.x, point_s.y],k = 1)
                if point_s != self.search_list[idx].data:
                    new_node = self.expand(point_s, idx)
                else:
                    continue
            else:
                min_dist = 99999
                idx = -1
                for j in range(len(self.search_list)):
                    dist = self.search_list[j].data.dis(point_s)
                    if dist < min_dist and dist > 0:
                        min_dist = dist
                        idx = j
                if idx == -1:
                    continue
                new_node = self.expand(point_s, idx)
                

            #check collision
            if not self.obs.collision(new_node, self.dynamic):
                # add it , find global path shrotest father
                if use_KDTree:
                    near_idx = self.search_tree.query_ball_point([new_node.x, new_node.y], self.step_len * 1.5)
                else:
                    near_idx = []
                    for j in range(len(self.search_list)):
                        if self.search_list[j].data.dis(new_node) < self.step_len * 1.5:
                            near_idx.append(j)
                father = -1
                min_dist = 99999
                for i in range(0, len(near_idx)):

                    if not self.obs.line_check(new_node, self.search_list[near_idx[i]].data, self.dynamic):
                        continue

                    if new_node.dis(self.search_list[near_idx[i]].data) + self.search_list[near_idx[i]].dist < min_dist:
                        father = near_idx[i]
                        min_dist = new_node.dis(self.search_list[near_idx[i]].data) + self.search_list[near_idx[i]].dist
                
                if father == -1:
                    continue
                self.search_list.append(Tree(new_node, father, min_dist))
        print("search step: ", self.sample_num)
        print("search failed")
        return False
    
    def run(self, start, end, dynamic = False):
        self.dynamic = dynamic
        self.search_list.clear()
        self.path.clear()
        self.new_path.clear()
        self.start = start
        self.end = end
        self.search_list.append(Tree(start,-1,0))
        if not self.tree_grow():
            return False
        self.trace_back()
        # self.print_old_path()
        print("path length: ", len(self.path))
        self.opt_path()
        # self.print_new_path()
        print("new path length: ", len(self.new_path))
        print("search success")
        return True
    
    def Process(self, start, end):
        arrived = self.run(start, end)
        return arrived, self.get_path()
    
    def trace_back(self):
        node = self.search_list[len(self.search_list)-1]
        self.path.append(node.data)
        while node.father>=0:
            node = self.search_list[node.father]
            self.path.append(node.data)
        self.path.reverse()

    def opt_path(self):
        node = self.path[0]
        next_node = None
        self.new_path.append(node)
        for i in range(1, len(self.path)):
            if node and self.obs.line_check(node, self.path[i], self.dynamic):
                next_node = self.path[i]
            else:
                node = next_node
                if node:
                    self.new_path.append(node)
                next_node = self.path[i]
        if next_node:
            self.new_path.append(next_node)
        # self.new_path.reverse() # dont need to reverse if input is reversed

    def get_path(self):
        return self.new_path
    
    def print_old_path(self):
        print("*"*20)
        for i in range(len(self.path)):
            print(self.path[i].x, self.path[i].y)
        
    def print_new_path(self):
        print("*"*20)
        for i in range(len(self.new_path)):
            print(self.new_path[i].x, self.new_path[i].y)