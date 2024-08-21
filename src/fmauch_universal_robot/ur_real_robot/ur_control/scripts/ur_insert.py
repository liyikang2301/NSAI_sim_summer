#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from cv2 import fastNlMeansDenoisingColoredMulti
from ur_base import TrueBase
import math
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import tf
import rospy
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from  bolt_position_detector
import copy
import tf2_ros
import traceback
import random

# from PIL import Image,ImageDraw
# import numpy as np 
from rigid_transform_3D import rigid_transform_3D
# from kalman import  Kalman
import math

class TrueInsert(TrueBase):
    def __init__(self, group_):     ## 初始化 TrueInsert 类
        super(TrueInsert, self).__init__(group_)     ## 调用父类 TrueBase 的构造函数，初始化基类的内容
    def get_insert_trajectory(self,real_pose,all_info):      
        ## 生成插入操作的轨迹。real_pose: 目标螺栓的真实的初始位姿。all_info: 包含螺栓相关信息的字典，包括螺栓的时间戳等。
        trajectory = []     ## 定义一个空列表，用于存储轨迹点
        # radius=0.0015
        radius=0     ## 定义插入操作的半径，半径为0，直接插入
        delta_angle = 39    ## 定义每次步进的角度增量
        scale_angle = delta_angle * math.pi / 180    ## 将角度增量转换为弧度
        scale_depth= 0.004  ## 定义每次步进的深度增量（米）
        total_ang=40    ## 定义插入操作的总角度
        print('get_insert_trajectory')     ## 输出信息，轨迹生成开始
        for i in range( int(total_ang / delta_angle + 1) ):     ## 通过for循环生成插入操作的各个轨迹点
            tamp_radius=radius*(1-i*delta_angle/total_ang)    ## 计算当前步的半径，随着插入逐步减小
            tamp_angle = scale_angle * i     ## 计算当前步的角度
            tamp_depth=scale_depth * i      ## 计算当前步的深度
            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_real_frame = geometry_msgs.msg.Pose()   ## 创建一个空的 Pose 对象，用于存储当前步的位姿
            # tgt_pose_in_real_frame.position.x = -0.009+tamp_radius * math.cos(tamp_angle)
            # tgt_pose_in_real_frame.position.y =0.003+tamp_radius * math.sin(tamp_angle)
            ## 根据当前步的参数计算当前的 x, y, z 坐标
            tgt_pose_in_real_frame.position.x =-self.x_shift+tamp_radius * math.cos(tamp_angle) 
            tgt_pose_in_real_frame.position.y =-self.y_shift+tamp_radius * math.sin(tamp_angle)
            tgt_pose_in_real_frame.position.z = -self.z_shift  + tamp_depth - 0.003
            q = tf.transformations.quaternion_from_euler(0, 0, 0)   ## 设置位姿的初始坐标
            # q = tf.transformations.quaternion_from_euler(0, 0, 0)           
            tgt_pose_in_real_frame.orientation.x = q[0]
            tgt_pose_in_real_frame.orientation.y = q[1]
            tgt_pose_in_real_frame.orientation.z = q[2]
            tgt_pose_in_real_frame.orientation.w = q[3]
            
            tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                          "base_link",
                                                          tgt_pose_in_real_frame,
                                                          all_info['bolt_ts'])   #new 坐标转换
            # print (tgt_pose_in_world_frame)
            # (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            # print(r,p,y)
            if not tgt_pose_in_world_frame is None:     ## 如果转换成功，则将转换后的坐标添加到轨迹列表中
                trajectory.append(tgt_pose_in_world_frame)
                # print ("the %d-th trajectory"%(i))  
        if len(trajectory) > 0:     ## 如果长度大于0（即轨迹中有点），输出调试信息，表明轨迹已生成
            print ("trajectory collected")
        return trajectory 

    def get_tgt_pose_in_world_frame(self,all_info):    
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()   ## 创建一个新的 Pose 对象，用于存储目标位姿在 real_bolt_frame 中的表示
        ## 设置目标位姿在 real_bolt_frame 中的位置坐标
        tgt_pose_in_real_frame.position.x = -self.x_shift
        tgt_pose_in_real_frame.position.y = -self.y_shift
        tgt_pose_in_real_frame.position.z = -self.z_shift - 0.075   
         ## 这里的 x_shift, y_shift, z_shift 是预定义的偏移量，用于调整目标位置，例如补偿相机的位置或者其他机械结构的限制

        q = tf.transformations.quaternion_from_euler(0, 0, 0)   ## 将欧拉角 (0, 0, 0) 转换为四元数，表示目标的姿态
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        ## 此处使用的是无旋转的姿态（欧拉角 (0, 0, 0) 对应的四元数）
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                            "base_link",
                            tgt_pose_in_real_frame,
                            all_info['bolt_ts'])       ## 坐标转换
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        print (tgt_pose_in_world_frame)     ## 输出转换后的坐标
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)    ## 将转换后的四元数姿态再转换为欧拉角（roll, pitch, yaw），以便检查姿态
        return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict, kalman,yolo,plc,target_pose):   ## 执行插入操作，包括调整目标螺栓的框架，生成插入轨迹，并控制机械臂进行插入操作
        ## 假设检测到螺栓，准备进行插入操作
        if True:
            ## 输出检测到的目标螺栓的信息
            print('real bolt detected') 
            print('real pose')
            # print(real_pose)
            # (r, p, y) = tf.transformations.euler_from_quaternion([real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w])
            # print(r,p,y)
            self.adjust_bolt_frame(target_pose,all_info)    ## 调整螺栓的坐标系，以便后续的插入操作
            ee_pose=self.get_tgt_pose_in_world_frame(all_info)  ## 获取目标在世界坐标系中的位姿
            curr_pose= self.group.get_current_pose(self.effector).pose  ## 获取当前机械臂末端执行器的位姿
            if not self.set_arm_pose(self.group, ee_pose, self.effector):   ## 尝试将机械臂移动到目标位置
                print("failed")
                print(curr_pose)
                ## 如果移动失败，输出当前的机械臂位姿坐标并退出
            insert_trajectory=self.get_insert_trajectory(target_pose,all_info)      ## 生成插入操作的轨迹
            # plc.set_effector_star_neg(200)
            rospy.sleep(1)
            for ee_pose in insert_trajectory:   ## 通过for循环执行插入轨迹中的每一个位姿
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("insert failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose 
                    print(ee_pose)
                     ## 如果插入过程中失败，输出失败的位姿
                # self.print_wrench()
            # rospy.sleep(30)
            # rospy.sleep(1)
            # plc.set_effector_stop()
            # plc.set_effector_stop()
            # self.plot()
            return {'success': True}    ## 返回成功标志
        else:
            return {'success': False}   ## 如果未检测到螺栓，返回失败标志