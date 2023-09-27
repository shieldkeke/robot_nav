# robot_nav

**其他语言版本: [English](README.md), [中文](README_zh.md).**

浙江大学 2023年短学期《机器人与智能系统综合实践》 

移动机器人运动控制仿真的课程作业

## 介绍

本项目的静态地图已经给出，需要完成运动学（已知v、w后得到双轮的转速）、TF消息发送、全局路径规划和轨迹跟踪。

在这个项目里，本人完成A\*、JPS、RRT\*方法实现的全局路径规划，简单选取中间路径点后，使用朴素方法和DWA方法跟踪中间路径点。

![GIF](README_img/GIF.gif)

![image-20230711103108877](README_img/image-20230711103108877.png)

![image-20230711103003141](README_img/image-20230711103003141.png)

## 使用方法

编译工作空间后运行 `roslaunch course_agv_nav nav.launch`

