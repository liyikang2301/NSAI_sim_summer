#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import pickle
import struct
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class YOLO_SendImg():
    def __init__(self, ip_port=('127.0.0.1', 5050)): #创建 TCP 套接字并连接到指定的 IP 地址和端口（默认为 127.0.0.1:5050）
        self.ip_port = ip_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port)

    def pack_image(self, frame): #将输入图像 frame 序列化为字节流，以便通过网络传输
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        # result, frame = cv2.imencode('.jpg', frame, encode_param)
        # data_encode = np.array(frame)
        # str_encode = data_encode.tostring()
        
        # # 缓存数据保存到本地，以txt格式保存
        # with open('img_encode.txt', 'w') as f:
        #     f.write(str_encode)
        #     f.flush
        
        # with open('img_encode.txt', 'r') as f:
        #     str_encode = f.read()
        
        # nparr = np.fromstring(str_encode, np.uint8)
        # img_decode = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        # cv2.imwrite("img_decode_2.jpg", img_decode)
        # cv2.imshow("img_decode", img_decode)
        # cv2.waitKey()

        data = pickle.dumps(frame, 0) #将图像矩阵数据序列化
        size = len(data)
        packed = struct.pack(">L", size) + data #使用大端（big-endian）格式将图像的大小打包为 4 字节。
        return packed, data, size

    def get_YOLO_result(self): #从服务器接收检测结果
        total_data = bytes()
        while True:
            data = self.sock.recv(1024) #每次接收 1024 字节的数据，直到接收完成。
            total_data += data
            if len(data) < 1024:
                break
        # data = self.sock.recv(409600)
        result = pickle.loads(total_data)
        return result

    def finish_YOLO_detect(self, frame): #执行完整的 YOLO 检测流程
        packed, data, size = self.pack_image(frame) #使用 pack_image 将图像数据打包
        self.sock.sendall(packed) #将打包后的图像数据发送给服务器
        print("send all finished")
        result = self.get_YOLO_result() #接收 YOLO 检测结果
        print(result)
        return result


if __name__ == '__main__':
    bolt_detector = YOLO_SendImg() #创建 YOLO_SendImg 对象并连接到服务器
    # frame = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/img_decode_2.jpg')
    frame = cv2.imread('/home/ur/Desktop/ur10e_sim/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/imgs/111.jpg')
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB) #读取一张图像，并将其转换为 RGB 格式
    result = bolt_detector.finish_YOLO_detect(frame) #调用 finish_YOLO_detect 方法发送图像并接收检测结果
    # frame1 = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/3.jpg')
    # result1 = bolt_detector.finish_YOLO_detect(frame1)
    # bolt_detector.sock.close()
    # print(result['bolt0'])
    print(result)
    # print(result1)
