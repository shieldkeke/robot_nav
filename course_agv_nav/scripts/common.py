import numpy as np
import math
from math import atan2, sqrt

# Parameters
dt = 0.02  # time tick
WB = 0.3  # wheel base of vehicle

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.w = w

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.w = delta

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
    

class Point:  
    def __init__(self, x, y):
        self.x = int(x) # attention: int
        self.y = int(y)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __sub__(self, p):
        return Vector(self.x - p.x, self.y - p.y)
    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)
    def dis(self, p):
        return sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
    
class MapGrid:  

    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.data = np.zeros([height, width], dtype=np.float64)

    def set_grid(self, x, y, type):
        if type == "obstacle":
            expand = 2
            for i in range(-expand,expand+1): #expand the obstacle
                for j in range(-expand,expand+1):
                    if x+i>0 and x+i<self.width and y+j>0 and y+j<self.height:
                        self.data[y+j][x+i] = 10
        if type == "origin":
            self.data[y][x] = 2 
        if type == "goal":
            self.data[y][x] = 1   
        if type == "path":
            self.data[y][x] = 7   
        if type == "jump":
            self.data[y][x] = 8  
        if type == "explored":
            self.data[y][x] = 5  
        if type == "normal":
            self.data[y][x] = 0 
        if type == "open":
            self.data[y][x] = 4 

class Vector(): 
    def __init__(self, x, y) -> None:
        self.x = int(x) # attention: int
        self.y = int(y)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __add__(self, p):
        return Vector(self.x + p.x, self.y + p.y)
    def __sub__(self, p):
        return Vector(self.x - p.x, self.y - p.y)
    def __rmul__(self, p):
        if isinstance(p, int) or isinstance(p, float):
            return Vector(self.x * p, self.y * p)
        elif isinstance(p, Vector):
            return self.x * p.x + self.y * p.y
    def __mul__(self, p):
        if isinstance(p, int) or isinstance(p, float):
            return Vector(self.x * p, self.y * p)
        elif isinstance(p, Vector):
            return self.x * p.x + self.y * p.y
    def __truediv__(self, p):
        return Vector(self.x/p, self.y/p)
    def ang(self):
        return atan2(self.y, self.x)
    def mod(self):
        return sqrt(self.x*self.x + self.y*self.y)

class Tree():
    def __init__(self, data, father = -1,dist = 0) -> None:
        self.data = data
        self.father = father
        self.dist = dist

class Obstacle():
    def __init__(self, map) -> None:
        self.map = map
        
    def collision(self, p, dynamic = False):
        # print("check: ", p.x, p.y)
        if p.x<0 or p.y<0 or p.x>=self.map.width or p.y>=self.map.height:
            return True
        if self.map.data[p.y][p.x] == 10:
            # print("collision")
            return True
        # print("no collision")
        return False
    
    def line_check(self, p1, p2, dynamic = False):
        # print("line check: ", p1.x, p1.y, p2.x, p2.y)
        if p1 == p2:
            return True
        if self.collision(p1) or self.collision(p2):
            return False
        
        if abs(p1.x-p2.x)>abs(p1.y-p2.y):
            if p1.x>p2.x:
                p1, p2 = p2, p1
            for i in range(p1.x, p2.x+1):
                p = Point(i, p1.y + (p2.y-p1.y)*(i-p1.x)/(p2.x-p1.x))
                if self.collision(p):
                    return False
        else:
            if p1.y>p2.y:
                p1, p2 = p2, p1
            for i in range(p1.y, p2.y+1):
                p = Point(p1.x + (p2.x-p1.x)*(i-p1.y)/(p2.y-p1.y), i)
                if self.collision(p):
                    return False
        return True
        

