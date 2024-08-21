#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from ur_base import TrueBase
import math
import geometry_msgs.msg
import tf
import rospy
import torch


class TrueMove(TrueBase):   
    def __init__(self, group_):     ## 初始化 TrueMove 类，指定操作组并定义所需的参数。
        super(TrueMove, self).__init__(group_)
        self.request_params = ['coarse_pose']

    def action(self, all_info, pre_result_dict, kalman,yolo,plc):   ## 执行移动操作，根据给定的粗略目标位姿控制机械臂移动
        for param in self.request_params:    ## 检查是否提供了所有必需的参数
            if not param in pre_result_dict.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do move")
        # planner = all_info['planner_handler']
        # latest_infos = planner.get_latest_infos()
        target = pre_result_dict["coarse_pose"]     ## 获取目标粗略位姿
        while True:     ## 循环尝试将机械臂移动到目标位姿
            if self.set_arm_pose(self.group, target, self.effector):
                # return {'success': False}
                break
        return {'success': True}