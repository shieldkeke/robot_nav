#!/usr/bin/env python3

import math
import numpy as np
from common import *

def find_best_goal(last_idx, poses, x, y):
    idx = last_idx
    min_idx = last_idx
    ans = last_idx
    min_dis = 99999
    max_dis = 0.5 

    arrived = math.hypot(x-poses[last_idx].pose.position.x,y-poses[last_idx].pose.position.y) < max_dis

    if idx == len(poses)-1:
        return idx
    next_idx = idx + 1

    while idx < len(poses):
        p = poses[idx].pose.position
        dis = math.hypot(p.x-x,p.y-y)
        if dis <  max_dis:
            ans = idx # find the farthest point that is within max_dis
        # elif dis < min_dis:
        #     min_dis = dis
        #     min_idx = idx # if no point is within max_dis, find the nearest point
        idx += 1


    if ans != last_idx:
        return ans
    # return min_idx

    if arrived: # if the robot has arrived at the last goal, return the new point
        return next_idx
    else:
        return last_idx

class Simple:

    # simple pure pursuit controller without obstacle avoidance

    def __init__(self):
        # Parameters
        self.kw = 0.6 # angular velocity gain
        self.kp = 0.08  # speed proportional gain
        self.lfc = 0.3  # look-ahead constant
        self.kf = 0.1 # forward gain

    def speed_control(self, current, target = 0.6):
        a = self.kp * (target - current)
        return a

    def steer_control(self, state, goal):
        tx = goal[0]
        ty = goal[1]
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        lfd = self.kf * state.v + self.lfc # look-ahead distance
        delta = math.atan2(self.kw * math.sin(alpha) , lfd)
        return delta
    
    def planning(self, state, goal, target_speed = 0.6):
        delta = self.steer_control(state, goal)
        a = self.speed_control(state.v, target_speed)
        return a, delta
    

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.7  # [m/s]
        self.min_speed = -0.7 # [m/s]
        self.max_yawrate = 200.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.4#*4  # [m/ss]
        self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.8
        self.dir_cost_gain = 0.2
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 0.0
        self.robot_width = 0.4  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check


class DWA:
    def __init__(self,config = Config()):
        self.config = config
        self.u_plot = []


    def planning(self, state, goal, ob):
        
        """
        Dynamic Window Approach control
        """
        # x=[state.x,state.y,state.yaw,state.v,state.w]
        x = [0,0,0,state.v,state.w]
        min_cost = 99999999
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        ob = None
        # calculate dynamic window
        vmin = max(self.config.min_speed, x[3]-self.config.max_accel * self.config.dt)
        vmax = min(self.config.max_speed, x[3]+self.config.max_accel * self.config.dt)
        wmin = max(-self.config.max_yawrate, x[4]-self.config.max_dyawrate * self.config.dt)
        wmax = min(self.config.max_yawrate, x[4]+self.config.max_dyawrate * self.config.dt)

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(vmin, vmax, self.config.v_reso):
            for w in np.arange(wmin, wmax, self.config.yawrate_reso):
                u = np.array([v, w])
                trajectory = self.calc_trajectory(x, u, self.config)
                dist = self.calc_dist(trajectory, goal, ob)
                if v <= (2*dist*self.config.max_accel)**0.5 and w <= (2*dist*self.config.max_dyawrate)**0.5:
                    cost = self.calc_cost(trajectory, goal, dist)
                    if cost < min_cost:
                        min_cost = cost
                        best_u = u
                        best_trajectory = trajectory
        
        # u[0] is vx, u[1] is vw
        # print(best_u)
        self.u_plot.append(best_u)
        return best_u

    def calc_trajectory(self, x, u, config):
        """
        calc_trajectory
        """
        x = np.array(x)
        u = np.array(u)
        trajectory = np.array([x])
        for t in np.arange(1, config.predict_time, config.dt):
            x = self.motion(x, u, config.dt)
            trajectory = np.append(trajectory, np.array([x]), axis=0)
        return trajectory

    def motion(self, x, u, dt):
        """
        motion model
        """
        if u[1] == 0:
            x[0] += u[0] * np.cos(x[2]) * dt
            x[1] += u[0] * np.sin(x[2]) * dt
            x[2] += u[1] * dt
            x[3] = u[0]
            x[4] = u[1]
        else:
            x[0] += u[0] / u[1] * (np.sin(x[2] + u[1] * dt) - np.sin(x[2]))
            x[1] += u[0] / u[1] * (-np.cos(x[2] + u[1] * dt) + np.cos(x[2]))
            x[2] += u[1] * dt
            x[3] = u[0]
            x[4] = u[1]
        return x

    def calc_cost(self, trajectory, goal, dist):
        """
        calc_cost
        """
        cost = 0.0
        goal_dist_dx = goal[0] - trajectory[0, 0]
        goal_dist_dy = goal[1] - trajectory[0, 1]
        goal_dist = np.sqrt(goal_dist_dx**2 + goal_dist_dy**2)
        #print("goal_dist",goal_dist)

        # to goal cost
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        cost_to_goal = np.sqrt(dx**2 + dy**2)
        # print("cost_to_goal",cost_to_goal* self.config.to_goal_cost_gain)
        cost += cost_to_goal * self.config.to_goal_cost_gain
        
        # directions cost
        angle1 = math.atan2(goal[1]-trajectory[-1,1],goal[0]-trajectory[-1,0])
        angle2 = trajectory[-1,2]
        delta = abs(angle1-angle2)
        if delta > math.pi:
            delta = 2*math.pi - delta
        cost += delta * self.config.dir_cost_gain

        # speed cost
        if goal_dist > 0.7:
           cost += (trajectory[0,3] - self.config.max_speed)**2 * self.config.speed_cost_gain
           # cost += (trajectory[0,3] - 0.6)**2 * self.config.speed_cost_gain
        else:
            cost += (trajectory[0,3]+1)**2 * self.config.speed_cost_gain
        # print("speedcost",cost-cost_to_goal* self.config.to_goal_cost_gain)

        # collision cost
        if dist < 1.0:
            cost += 1.0/(dist-np.sqrt((self.config.robot_width/2)**2 + (self.config.robot_length/2)**2)-0.2) * self.config.obstacle_cost_gain
        # print("cost",cost)
        return cost

    def calc_dist(self, trajectory, goal, ob):
        """
        calc_dist
        """
        # collision cost
        if ob is not None:
            min_dist = 99999999
            for i in range(len(trajectory)):
                for ob_x, ob_y in ob:
                    dx = trajectory[i, 0] - ob_x
                    dy = trajectory[i, 1] - ob_y
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist < min_dist:
                        min_dist = dist
        else:
            min_dist = 99999999
        return min_dist
