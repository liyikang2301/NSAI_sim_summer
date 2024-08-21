import os
import numpy as np
import copy

#动作原语的类型，分别定义不同的前置条件与执行原语之后的效果

#define prim,include their pre-condition and eff
class PrimAction:
    def __init__(self, prim):
        self.pre={}
        self.eff={}
        self.prim=prim
        if self.prim=='move':
            self.pre={'have_coarse_pose':True, 'above_bolt':False}
            self.eff={'above_bolt':True}
        elif self.prim=='mate':
            self.pre={'target_aim':False,'above_bolt':True}
            self.eff={'target_aim':True}
        elif self.prim=='insert':
            self.pre={'target_aim':True,'cramped':False}
            self.eff={'cramped':True}
        elif self.prim=='disassemble':
            self.pre={'cramped':True,'disassembled':False}
            self.eff={'disassembled':True}
    

    #change stage after we finish one primitive
    def action(self,stage): #根据原语执行后的效果，来更新当前的状态
        new_stage=copy.deepcopy(stage)
        for e in self.eff:
            new_stage[e]=self.eff[e]
        return new_stage
    

    #verify pre of a prim
    def able(self,stage): #检查当前状态是否满足执行操作的前置条件
        for p in self.pre:
            if not stage[p]==self.pre[p]:
                return False
        return True