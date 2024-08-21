import torch
from torch import nn
from torchvision import models

#本代码定义了基于VGG16模型的自定义神经网络类“VGG”，用于图像分类

class VGG(nn.Module):
    def __init__(self, num_classes=1000):
        super(VGG, self).__init__()
        self.net = models.vgg16(pretrained=True) #加载预训练的VGG16模型
        self.net.classifier[6] =nn.Linear(4096, num_classes)
        #修改VGG16模型的最后一层全连接层（分类器），将其输出的维度从原始的1000类别调整为num_classes指定的类别数。4096是前一层的输入维度（VGG16的全连接层的输出维度）。

        # # 卷积层提取图像特征
        # self.features = self.net.features
        # self.avgpool = self.net.avgpool
        #
        # # 全连接层对图像分类
        # self.classifier = nn.Sequential(
        #     self.net.classifier[0],
        #     self.net.classifier[1],
        #     self.net.classifier[2],
        #     self.net.classifier[3],
        #     self.net.classifier[4],
        #     self.net.classifier[5],
        #     nn.Linear(4096, num_classes),
        #     nn.Softmax(dim=0)
        # )

    def forward(self, x): #调用模型获得输出
        # x = self.features(x)
        # x = self.avgpool(x)
        # x = self.net.avgpool(x)
        x = self.net(x)
        return x
