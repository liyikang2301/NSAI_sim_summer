#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import importlib
import os
import threading

import tf
import sys
import cv2
import time
import rospy
import random
import pprint
import image_geometry
import message_filters
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
# from  bolt_position_detector
import copy
import tf2_ros
import geometry_msgs.msg
import traceback
import random
import math

from PIL import Image
# from PIL import Image,ImageDraw
# import numpy as np 
# from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from ur_base import TrueBase

class TrueAimTarget(TrueBase):
    def get_tgt_pose_in_world_frame(self,all_info): #计算目标物体（如螺栓）的世界坐标系下的位姿，并进行坐标变换以使机械臂能够正确定位。
        
        # 这里的偏移量是为了让相机正对着螺栓，这两个补偿跟相机的参数（镜头位置）有关系。
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = -self.x_shift
        # tgt_pose_in_real_frame.position.y = -self.y_shift + 0.065
        tgt_pose_in_real_frame.position.y = -self.y_shift

        tgt_pose_in_real_frame.position.z = -self.z_shift - 0.09

        # q = tf.transformations.quaternion_from_euler(0, 0, 0.1*math.pi)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)        
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        #计算一个相对于“真实”螺栓坐标系的目标姿态，并应用补偿量（x_shift, y_shift, z_shift）以确保相机能够正确对准螺栓。
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                      "base_link",
                                                      tgt_pose_in_real_frame,
                                                      all_info['bolt_ts'] )
        #将目标位姿从螺栓坐标系转换到机械臂的基础坐标系（base_link），以便机械臂能够正确识别和操作螺栓。

        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        print (tgt_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)
        return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict, kalman,yolo,plc):
        for param in self.action_params: #检查是否提供了所有必要的参数，否则返回失败
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to mate")
        planner = all_info['planner_handler']
        latest_infos = planner.get_latest_infos()      
        raw_img=latest_infos['rgb_img']
        height=raw_img.shape[0]
        width =raw_img.shape[1]
        r_height=540
        r_width =960
        # print(height,width)
        crop_img= cv2.copyMakeBorder(raw_img,int((r_height-height)/2),int((r_height-height)/2),int((r_width-width)/2),int((r_width-width)/2),cv2.BORDER_CONSTANT,value=0)
        # crop_img=raw_img[int(0.25*height):int(0.75*height),int(0.5*(width-0.5*height)):int(0.5*(width+0.5*height))]
        # crop_img=raw_img[:,int(0.5*(width-height)):int(0.5*(width+height))]
        detect_ret=yolo.finish_YOLO_detect(crop_img)
        #从最新的图像信息中获取RGB图像并进行裁剪，以适应YOLO目标检测，并使用YOLO检测螺栓位置
        target_pose=None
        if detect_ret: #如果yolo返回的有值
            circlesbox=[]
            if 'bolt' in detect_ret[1].keys():
                print('bolt center success')
                circlesbox.extend(detect_ret[1]['bolt'])
                min_dist=100
                curr_pose= self.group.get_current_pose(self.effector).pose #拿到当前末端执行器的pose，用来找哪个螺栓距离tool_0最近，哪个近优先拆哪个
                for screw in circlesbox:
                    # self.add_bolt_frame(screw[0]-(r_width-width)/2,screw[1]-(r_height-height)/2, latest_infos)
                    screw[0] = screw[0]-(r_width-width)/2
                    screw[2] = screw[2]-(r_width-width)/2
                    screw[1] = screw[1]-(r_height-height)/2
                    screw[3] = screw[3]-(r_height-height)/2                        
                    self.add_bolt_frameV2(screw, latest_infos)
                    bolt_pose=self.get_bolt_pose_in_world_frame(latest_infos)
                    temp_dist=math.sqrt(pow(bolt_pose.position.x - curr_pose.position.x ,2)+pow(bolt_pose.position.y - curr_pose.position.y ,2))            
                    if (temp_dist<min_dist):
                        min_dist=temp_dist
                        target_pose = bolt_pose 
        #将检测到的螺栓框架添加到机械臂的坐标系中，并计算每个螺栓到当前机械臂末端位置的距离。
        #选择距离最近的螺栓作为目标。
        if target_pose==None:
            return {'success': False}
        else:
            self.adjust_bolt_frame(target_pose,latest_infos)
            ee_pose=self.get_tgt_pose_in_world_frame(latest_infos)
            while True:
                if self.set_arm_pose(self.group, ee_pose, self.effector):
                    print(curr_pose)
                    break          
            return {'success': True,"target_pose":target_pose}
        #如果找到了目标螺栓，则调整螺栓的框架，并获取末端执行器的目标位姿，使用set_arm_pose 方法将机械臂移动到目标位姿并返回执行结果
