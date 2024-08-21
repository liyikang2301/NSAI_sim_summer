#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function
from ur_base import TrueBase
import math
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import tf
import rospy
import numpy as np
import sys
from ur_msgs.srv import *
from ur_msgs.msg import *
import time
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

class TrueDisassemble(TrueBase):
    def __init__(self, group_):
        super(TrueDisassemble, self).__init__(group_)
        self.switch=False
        # wrench
        self.wrench=np.array([[0,0,0,0,0,0]]) #保存力/力矩传感器的历史数据和当前数据
        self.cur_wrench=np.array([[0,0,0,0,0,0]])
        self.collect=False #控制是否收集传感器数据
    
    def get_disassemble_trajectory(self):

        trajectory =  []  #存储生成的轨迹点
        scale_depth= 0.06 #深度缩放因子，用于定义末端执行器沿着 z 轴的运动距离
        print('get_return_trajectory')
        for i in range(1):
            tamp_depth=scale_depth *(i+1) #计算末端执行器沿z轴移动的距离
            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_effector_frame = geometry_msgs.msg.Pose() #表示末端执行器在工具坐标系下的目标位姿
            tgt_pose_in_effector_frame.position.x = 0 #表示没有沿 x 轴和 y 轴的位移
            tgt_pose_in_effector_frame.position.y = 0
            tgt_pose_in_effector_frame.position.z = -tamp_depth
            q = tf.transformations.quaternion_from_euler(0, 0, 0) #生成四元数q，表示工具坐标系下的目标姿态
            tgt_pose_in_effector_frame.orientation.x = q[0]
            tgt_pose_in_effector_frame.orientation.y = q[1]
            tgt_pose_in_effector_frame.orientation.z = q[2]
            tgt_pose_in_effector_frame.orientation.w = q[3]
            
            tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                          "base_link",
                                                          tgt_pose_in_effector_frame,
                                                          rospy.Time.now()) #将目标位姿从末端执行器坐标系 (self.effector) 转换到世界坐标系 (base_link) 下
            print (tgt_pose_in_world_frame) #输出转化后的目标位姿
            (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w]) #将四元数转换为欧拉角
            print(r,p,y)
            if not tgt_pose_in_world_frame is None: #如果转换成功且 tgt_pose_in_world_frame 不是 None，则将其添加到轨迹列表 trajectory 中
                trajectory.append(tgt_pose_in_world_frame)
                print ("the %d-th trajectory"%(i))
        if len(trajectory) > 0: #判断是否正确生成轨迹
            print ("trajectory collected")
        return trajectory

    def get_tgt_pose_in_world_frame(self): #根据末端执行器在其自身坐标系下的一个小范围内移动的目标位姿，将其转换到世界坐标系下
            tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_effector_frame.position.x = 0
            tgt_pose_in_effector_frame.position.y = 0
            tgt_pose_in_effector_frame.position.z = -0.0015
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            tgt_pose_in_effector_frame.orientation.x = q[0]
            tgt_pose_in_effector_frame.orientation.y = q[1]
            tgt_pose_in_effector_frame.orientation.z = q[2]
            tgt_pose_in_effector_frame.orientation.w = q[3]
            tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                          "base_link",
                                                          tgt_pose_in_effector_frame,
                                                          rospy.Time.now())
            return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict, kalman,yolo,plc):
        for param in self.action_params: #检查参数完整性
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to disassemble")
        trajectory = self.get_disassemble_trajectory() #获取拆卸原语机械臂的运动轨迹
        curr_pose = self.group.get_current_pose(self.effector).pose
        plc.set_effector_star_neg(3000) #设置电机转速
        rospy.sleep(1)
        print("speed:",plc.read_effector_speed())
        time.sleep(1.2)
        for ee_pose in trajectory: #遍历每个路径点，并将末端执行器移动至相应位置
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("disassemble failed") #如果有一步失败，则打印错误信息并获取当前位姿进行调试
                ee_pose = self.group.get_current_pose(self.effector).pose 
                print(ee_pose)
        plc.set_effector_stop() # 停止末端执行器
        rospy.sleep(0.1)     #暂停 0.1 秒以确保效应器完全停止
        print("speed:",plc.read_effector_speed()) #再次读取并打印效应器的速度，确认其已停止
        rospy.is_shutdown
        return {'success': True}