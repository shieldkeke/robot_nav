import numpy as np
import math

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
    
class Vec:
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class Point:  
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class MapGrid:  

    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.data = np.zeros([height, width], dtype=np.float64)

    def set_grid(self, x, y, type):
        if type == "obstacle":
            for i in range(-3,4): #expand the obstacle
                for j in range(-3,4):
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