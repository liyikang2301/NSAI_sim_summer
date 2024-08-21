#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import pickle
import struct
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import pickle
import struct
import cv2


# class YOLO_SendImg():
#     def __init__(self, ip_port=('127.0.0.1', 5050)):
#         self.ip_port = ip_port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.sock.connect(ip_port)

#     # def pack_image(self, frame):
#     #     encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#     #     result, frame = cv2.imencode('.jpg', frame, encode_param)
#     #     data = pickle.dumps(frame, 0)
#     #     size = len(data)
#     #     packed = struct.pack(">L", size) + data
#     #     return packed, data, size
#     def pack_image(self, frame):
#         encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#         result, frame_encoded = cv2.imencode('.jpg', frame, encode_param)
#         data = pickle.dumps(frame_encoded, 0)
#         size = len(data)
#         packed = struct.pack(">L", size) + data
#         return packed, data, size

#     def get_YOLO_result(self):
#         data = self.sock.recv(4096)
#         result = pickle.loads(data)
#         return result

#     def finish_YOLO_detect(self, frame):
#         packed, data, size = self.pack_image(frame)
#         self.sock.sendall(packed)
#         print("send all finished")
#         result = self.get_YOLO_result()
#         return result


# if __name__ == '__main__':
#     bolt_detector = YOLO_SendImg()
#     # frame = cv2.imread('/home/inexbot/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/img_crop/1.jpg')

#     frame = '/home/inexbot/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/img_crop/7.jpg'

#     input_stream = frame
#     cap = cv2.VideoCapture(input_stream)
#     ret, frame = cap.read()
#     print("frame:",frame)


#     # detect_with_IR_formed_YOLO(frame_im)
#     result = bolt_detector.finish_YOLO_detect(frame)
#     # frame1 = cv2.imread('/home/ur/Desktop/ur10e_sim/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/imgs/2.jpg')
#     # result1 = bolt_detector.finish_YOLO_detect(frame1)
#     # bolt_detector.sock.close()
#     # print(result['bolt0'])
#     print(result)
#     # print(result1)


# class YOLO_SendImg():
#     def __init__(self, ip_port=('127.0.0.1', 5050)):
#         self.ip_port = ip_port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.sock.connect(ip_port)

#     def pack_image(self, frame):
#         # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#         # result, frame = cv2.imencode('.jpg', frame, encode_param)
#         # data_encode = np.array(frame)
#         # str_encode = data_encode.tostring()
        
#         # # 缓存数据保存到本地，以txt格式保存
#         # with open('img_encode.txt', 'w') as f:
#         #     f.write(str_encode)
#         #     f.flush
        
#         # with open('img_encode.txt', 'r') as f:
#         #     str_encode = f.read()
        
#         # nparr = np.fromstring(str_encode, np.uint8)
#         # img_decode = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
#         # cv2.imwrite("img_decode_2.jpg", img_decode)
#         # cv2.imshow("img_decode", img_decode)
#         # cv2.waitKey()

#         data = pickle.dumps(frame, 0)
#         size = len(data)
#         packed = struct.pack(">L", size) + data
#         return packed, data, size

#     def get_YOLO_result(self):
#         total_data = bytes()
#         while True:
#             data = self.sock.recv(1024)
#             total_data += data
#             if len(data) < 1024:
#                 break
#         # data = self.sock.recv(409600)
#         result = pickle.loads(total_data)
#         return result

#     def finish_YOLO_detect(self, frame):
#         packed, data, size = self.pack_image(frame)
#         self.sock.sendall(packed)
#         print("send all finished")
#         result = self.get_YOLO_result()
#         print(result)
#         return result


# if __name__ == '__main__':
#     bolt_detector = YOLO_SendImg()
#     # frame = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/img_decode_2.jpg')
#     frame = cv2.imread('/home/ur/Desktop/ur10e_sim/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/imgs/111.jpg')
#     frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
#     result = bolt_detector.finish_YOLO_detect(frame)
#     # frame1 = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/3.jpg')
#     # result1 = bolt_detector.finish_YOLO_detect(frame1)
#     # bolt_detector.sock.close()
#     # print(result['bolt0'])
#     print(result)
#     # print(result1)



class YOLO_SendImg():
    def __init__(self, ip_port=('127.0.0.1', 5050)): #创建一个 TCP 套接字并连接到指定的 IP 地址和端口（默认 127.0.0.1:5050）
        self.ip_port = ip_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port) #连接到指定的 IP 和端口，建立 TCP 连接。

    # def pack_image(self, frame):
    #     encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    #     result, frame = cv2.imencode('.jpg', frame, encode_param)
    #     data = pickle.dumps(frame, 0)
    #     size = len(data)
    #     packed = struct.pack(">L", size) + data
    #     return packed, data, size
    def pack_image(self, frame): #将输入图像 frame 进行编码、序列化、打包，以便通过网络传输。
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] #将图像编码为 JPEG 格式，并指定压缩质量为 90。frame_encoded 是编码后的图像。
        result, frame_encoded = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame_encoded, 0) #将编码后的图像序列化为字节流，便于传输。
        size = len(data)
        packed = struct.pack(">L", size) + data #使用大端（big-endian）格式将图像数据大小打包为 4 字节的二进制数据
        return packed, data, size

    def get_YOLO_result(self): #从服务器接收 YOLO 检测结果
        data = self.sock.recv(4096)
        result = pickle.loads(data)
        return result

    def finish_YOLO_detect(self, frame): #执行完整的 YOLO 检测流程
        packed, data, size = self.pack_image(frame) #将图像打包
        self.sock.sendall(packed) #将打包后的图像数据通过网络发送给服务器
        print("send all finished")
        result = self.get_YOLO_result() #接收并处理检测结果
        # print('result@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@', result)
        result2 = [{'bolt': result[0]}, {'bolt': result[1]}]
        return result


if __name__ == '__main__': #执行 YOLO 图像检测任务
    bolt_detector = YOLO_SendImg() #创建 YOLO_SendImg 对象，建立与服务器的连接
    # frame = cv2.imread('/home/inexbot/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/img_crop/1.jpg')

    frame = '/home/inexbot/NeuralSymbol_AI/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/img_crop/6.jpg'

    input_stream = frame
    cap = cv2.VideoCapture(input_stream) #打开视频流或读取图像文件
    ret, frame = cap.read() #读取视频流的第一帧图像或图像文件
    print("frame:",frame)


    # detect_with_IR_formed_YOLO(frame_im)
    result = bolt_detector.finish_YOLO_detect(frame) #发送图像数据给 YOLO 服务并获取检测结果
    # frame1 = cv2.imread('/home/ur/Desktop/ur10e_sim/src/fmauch_universal_robot/ur_real_robot/YOLO_v5_detect/imgs/2.jpg')
    # result1 = bolt_detector.finish_YOLO_detect(frame1)
    # bolt_detector.sock.close()
    # print(result['bolt0'])
    print('result@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@', result)
    # print(result1)