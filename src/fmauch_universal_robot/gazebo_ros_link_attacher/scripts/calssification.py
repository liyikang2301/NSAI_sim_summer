import os
from shutil import copy
import random
 
 
def mkfile(file):
    if not os.path.exists(file):
        os.makedirs(file)
 
 
file = '/home/xps/Desktop/ur10e_sim/src/fmauch_universal_robot/ur_real_robot/VAE_detect/true_mul_bolt_crops'
flower_class = [cla for cla in os.listdir(file) if ".txt" not in cla]
mkfile(''+file+'/train')
for cla in flower_class:
    mkfile(''+file+'/train/'+cla)
 
mkfile(''+file+'/ver')
for cla in flower_class:
    mkfile(''+file+'/ver/'+cla)

mkfile(''+file+'/test')
for cla in flower_class:
    mkfile(''+file+'/test/'+cla)
 
split_rate = 0.3
test_rate = 0.1
# cla_path = file
# images = os.listdir(cla_path)
# num = len(images)
# eval_index = random.sample(images, k=int(num*split_rate))
# test_index = random.sample(eval_index, k=int(num*test_rate))
# for index, image in enumerate(images):
#     if image in test_index:
#         image_path = cla_path + image
#         new_path = ''+file+'/test' 
#         copy(image_path, new_path)
#     elif image in eval_index:
#         image_path = cla_path + image
#         new_path = ''+file+'/ver' 
#         copy(image_path, new_path)
#     else:
#         image_path = cla_path + image
#         new_path = ''+file+'/train' 
#         copy(image_path, new_path)
#     print("\r[{}] processing [{}/{}]".format(cla, index+1, num))  # processing bar
# print()    

for cla in flower_class:
    cla_path = file + '/' + cla + '/'
    images = os.listdir(cla_path)
    num = len(images)
    eval_index = random.sample(images, k=int(num*split_rate))
    test_index = random.sample(eval_index, k=int(num*test_rate))
    for index, image in enumerate(images):
        if image in test_index:
            image_path = cla_path + image
            new_path = file+'/test/' + cla
            copy(image_path, new_path)
        elif image in eval_index:
            image_path = cla_path + image
            new_path = file+'/ver/' + cla
            copy(image_path, new_path)
        else:
            image_path = cla_path + image
            new_path =file+'/train/' + cla
            copy(image_path, new_path)
        print("\r[{}] processing [{}/{}]".format(cla, index+1, num))  # processing bar
    print()
 
print("processing done!")