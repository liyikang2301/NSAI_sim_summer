#!/usr/bin/env python
# -*- coding: utf-8 -*-  
import math
import sys
import select, termios, tty
import rospy
import moveit_commander
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os

##本代码实现通过键盘控制机械臂运动，通过不同按键，可以控制机械臂在空间中的位置和姿态变化，包括平移和旋转及复位机械臂。
usage = """
Control the position of an end effector
---------------------------

j/l : forward/backward   
i/k : right/ left
p/; : up/down



以下是未矫正控制，
t   : robot transform to tilt
a/q : left/right in tilt 
x   : reset all joints on arms to zero
e/r : raw -/+
d/f : pitch -/+
c/v : yaw  -/+
s   : setting x y z R P Y
w   : capture image
<space> : print current pose
<CTRL-C>: quit
"""


# class Camera():
#     def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

#         self.camera_name = camera_name
#         self.rgb_topic = rgb_topic
#         self.depth_topic = depth_topic
#         self.camera_info_topic = camera_info_topic

#         self.pose = None


#         self.br = tf.TransformBroadcaster()

#         # Have we recieved camera_info and image yet?
#         self.ready_ = False

#         self.bridge = CvBridge()

#         self.camera_model = image_geometry.PinholeCameraModel()
#         print(
#             'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
#         print('')

#         q = 1
#         self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
#         self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
#         self.sub_camera_info = message_filters.Subscriber(camera_info_topic, CameraInfo, queue_size=q)
#         # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
#         self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info],
#                                                                queue_size=30, slop=0.2)
#         # self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)

#         self.tss.registerCallback(self.callback)
#         self.capture = False
#         directory = './images'
#         if not os.path.exists(directory):
#             os.makedirs(directory)

#     def callback(self, rgb_msg, depth_msg, camera_info_msg):
#         if not self.capture:
#             return
#         rgb_img_path = './images/rgb_img_%s.jpg'
#         depth_img_path = './images/depth_img_%s.png'

#         self.camera_model.fromCameraInfo(camera_info_msg)
#         img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#         depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
#         print('receiving image')
#         time_str = rospy.get_time()
#         cv2.imwrite(rgb_img_path%(time_str), img)
#         cv2.imwrite(depth_img_path%(time_str), depth_img)
#         self.capture = False

#     def set_capture(self):
#         self.capture = True


def get_key(): #用于读取用户的键盘输入
    tty.setraw(sys.stdin.fileno()) #将终端设置为原始模式，使得输入不会回显，并且不需要按下回车键就能获取输入
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) #用于非阻塞地检查是否有键盘输入，如果有，则读取一个字符并返回，否则返回空字符串。
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) #在读取完键后恢复终端的设置
    return key

def set_arm_pose(group, pose, effector): #设置机械臂的目标位姿，并规划运动路径
    group.set_pose_target(pose, effector) #设置机械臂末端执行器（effector）的目标位姿
    # plan = group.plan()
    plan_success, plan, planning_time, error_code = group.plan() #规划机械臂的运动路径，并返回规划成功与否、路径规划的具体内容、规划所用时间及错误码
    if len(plan.joint_trajectory.points) > 0: #如果规划成功，执行规划的路径，并返回 True，否则返回 False
        group.execute(plan, wait=True)
        return True
    else:
        print ('no plan result')
        return False


        
def reset_arm(group): #将机械臂复位到一个预定的关节角度位置
    joints = {} #包含了各个关节的目标角度值,通过设置这些关节角度值并规划路径，将机械臂复位到这些角度
    joints["elbow_joint"] = math.pi/2.
    joints["shoulder_lift_joint"] = -math.pi/2.
    joints["shoulder_pan_joint"] = math.pi/2.
    joints["wrist_1_joint"] = 1.5* math.pi
    joints["wrist_2_joint"] = -math.pi/2.
    joints["wrist_3_joint"] = 0.
    group.set_joint_value_target(joints)
    # plan = group.plan()
    plan_success, plan, planning_time, error_code = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
        return True
    else:
        return False

def print_pose(pose): #打印机械臂当前的位姿
    # q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    # rpy = tf.transformations.euler_from_quaternion(q)
    # print '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
    #     (effector, pose.position.x, pose.position.y, pose.position.z, \
    #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
    #     rpy[0], rpy[1], rpy[2])
    print (pose) #将位姿中的四元数（orientation）转换为欧拉角（roll、pitch、yaw），并打印出来
    (r, p, y) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    print(r,p,y)

if __name__=="__main__":
    effector = sys.argv[1] if len(sys.argv) > 1 else 'tool0' #通过命令行参数或默认值获取机械臂末端执行器的名称（effector）
    print(effector)

    #初始化ROS节点和MoveIt，Commander
    settings = termios.tcgetattr(sys.stdin)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_end_effector', anonymous=True)
    #group = moveit_commander.MoveGroupCommander("xarm6")
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id("RRTConnectkConfigDefault")

    print (usage)
    ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)
    # camera = Camera('camera', '/camera/color/image_raw', '/camera/depth/image_raw',
    #                 '/camera/color/camera_info')
    all_delta=0.001
    z_delta=all_delta
    x_delta=all_delta
    y_delta=all_delta

    while(1): #进入一个无限循环，等待用户的键盘输入，并根据输入来移动机械臂或者改变机械臂的姿态
              #对应不同的按键，机械臂可以在各个方向上移动，或者绕不同的轴旋转
              #如果按下了 Ctrl+C ，则程序退出
            
            delta_distance_tilt=0.01
        
            key = get_key()
            #if key in moveBindings.keys():
            q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
            rpy = tf.transformations.euler_from_quaternion(q)
            if key == ' ' :
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)

            elif key== 't':
                print ('location 45度到倾斜角')
                tf_angle=-math.pi+math.pi/4
                q = tf.transformations.quaternion_from_euler(tf_angle, 0.001130,-0.000277)
                ee_pose.position.x=-0.080070
                ee_pose.position.y=-0.154564
                ee_pose.position.z=1.135476

                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)


            elif key== 'a':
                print ('-zy,倾斜面的移动,会带动y,z的移动，采取45度倾斜角进行计算')
                z_tilt=1 /1.41*delta_distance_tilt
                y_tilt=1/1.41*delta_distance_tilt
                # z_tilt=1/delta_distance_tilt

                ee_pose.position.z -=z_tilt
                ee_pose.position.y -= y_tilt

                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)

            elif key== 'q':
                print ('+zy,倾斜面的移动,会带动y,z的移动，采取45度倾斜角进行计算')
                z_tilt=1 /1.41*delta_distance_tilt
                y_tilt=1/1.41*delta_distance_tilt
                # z_tilt=1/delta_distance_tilt
                ee_pose.position.z +=z_tilt
                ee_pose.position.y += y_tilt
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)


            elif key== 'c':
                print ('Y-')
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]-0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'v':
                print ('Y+')
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]+0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'd':
                print ('P-')
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]-0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                print_pose(ee_pose)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'f':
                print ('P+')
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]+0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'e':
                print ('R-')
                q = tf.transformations.quaternion_from_euler(rpy[0]-0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'r':
                print ('R+')
                q = tf.transformations.quaternion_from_euler(rpy[0]+0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'p':

                print ('z+')
                ee_pose.position.z += z_delta
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== ';':

                print ('z-')
                ee_pose.position.z -= z_delta
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'l':
                print ('y-')
                ee_pose.position.y -=y_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'j':

                print ('y+')
                ee_pose.position.y += y_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)

                
            elif key== 'i':
                print ('x+')
                ee_pose.position.x +=x_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'k':
                print ('x-')
                ee_pose.position.x -=x_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'x':
                print ('reset')
                reset_arm(group)
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key == 's':
                input_str = input("please input x y z R P Y:")
                val_str = input_str.split()
                if len(val_str) != 6:
                    print ('incorrect input')
                else:
                    ee_pose.position.x = float(val_str[0])
                    ee_pose.position.y = float(val_str[1])
                    ee_pose.position.z = float(val_str[2])
                    q = tf.transformations.quaternion_from_euler(float(val_str[3]),
                                                                 float(val_str[4]),
                                                                 float(val_str[5]))
                    ee_pose.orientation.x = q[0]
                    ee_pose.orientation.y = q[1]
                    ee_pose.orientation.z = q[2]
                    ee_pose.orientation.w = q[3]
                    if not set_arm_pose(group, ee_pose, effector):
                        ee_pose = group.get_current_pose(effector).pose
                    print_pose(ee_pose)
            # elif key == 'w':
            #     # camera.set_capture()
            elif (key == '\x03'):
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
