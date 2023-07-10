#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import LaserScan

from common import *
from pursuit import *

from threading import Lock,Thread
import time

def limitVal(minV,maxV,v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v

class LocalPlanner:
    def __init__(self, type_ = 'simple'):#"simple", "dwa"
        self.arrive = 0.1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.laser_lock = Lock()
        self.threshold = 0.5 # threshold for obstacle
        self.goal_dis = 0.0
        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.planner_thread = None
        self.need_exit = False
        self.type = type_

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

        if len(self.path.poses) == 0:
            return
        
        self.goal_index = find_best_goal(self.goal_index, self.path.poses, self.trans[0], self.trans[1])
        
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        lgoal = self.tf.transformPose("/robot_base", goal)
        self.plan_goal = np.array([lgoal.pose.position.x,lgoal.pose.position.y]) # in robot_base frame
        self.goal_dis = math.hypot(self.x-self.path.poses[-1].pose.position.x,self.y-self.path.poses[-1].pose.position.y)

    def laserCallback(self,msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100,100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment*i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a)*r,math.sin(a)*r])
        self.laser_lock.release()
        
    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob)
        self.laser_lock.release()
        
    def pathCallback(self,msg):
        self.need_exit = True
        time.sleep(0.1)
        self.path = msg
        
        self.planner_thread = Thread(target=self.planThreadFunc)
        if len(self.path.poses) == 0:
            return
        self.initPlanning()
        self.planner_thread.start()

    def initPlanning(self):
        self.goal_index = 0
        self.vx = 0.0
        self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        if len(self.path.poses) == 0:
            return
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.final = np.array([cx[-1],cy[-1]])
        self.plan_cx,self.plan_cy = np.array(cx),np.array(cy)
        self.state = State(x = self.x, y = self.y, yaw = self.yaw, v = 0.0, w = 0.0)
        if self.type == 'simple':
            self.planner = Simple()
        elif self.type == 'dwa':
            self.planner = DWA()
        
    def planThreadFunc(self):
        print("running planning thread!!")
        self.need_exit = False
        while not self.need_exit:
            self.planOnce()
            if len(self.path.poses) == 0:
                return
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
            time.sleep(0.001)
        print("exit planning thread!!")
        self.publishVel(True)
        self.planner_thread = None
        
    def planOnce(self):
        self.updateGlobalPose()
        if len(self.path.poses) == 0:
            return
        
        self.updateObstacle()

        self.state.x = self.x
        self.state.y = self.y
        self.state.yaw = self.yaw

        if self.type == 'simple':
            target_speed = 0.3
            a, d_theta = self.planner.planning(self.state, (self.plan_cx[self.goal_index], self.plan_cy[self.goal_index]), target_speed)
            self.state.update(a, d_theta)

        # [attension] self.plan_goal, plan_ob, state is in robot_base frame; cx,cy is in map frame
        
        elif self.type == 'dwa':
            v, w = self.planner.planning(self.state, self.plan_goal, self.plan_ob)
            self.state.v = v
            self.state.w = w

        self.publishVel()
        

    def publishVel(self,zero = False):
        cmd = Twist()
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        else:
            cmd.linear.x = self.state.v
            cmd.angular.z = self.state.w
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
