# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
import pyrealsense2 as rs
import sys
### 模型类的包
from torchvision import transforms
from torch.utils.data.dataloader import DataLoader
import json
from PIL import Image
import matplotlib.pyplot as plt
from model import resnet34


sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../")) )
from grasping_framework.utils import model_factory
from grasping_framework.utils.test_data_loader import GraspingSlidingDataset

### doosan包
import rospy
import time
import threading, time

import yaml
import time
import numpy as np
import math

### seed灵巧手包
import std_msgs.msg
from seed_robotics.msg import JointListSetSpeedPos ,JointSetSpeedPos,SetShutdownCond

# doosan包
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../doosan-robot/common/imp")) ) # get import path : DSR_ROBOT.py 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


### 检查 CUDA 是否可用
cuda_avail = torch.cuda.is_available()

######################################################################################### 相机类
class RealsenseImageSaver:
    def __init__(self, save_directory=''):
        self.pipeline = rs.pipeline()
        ## Configure the pipeline to stream color images
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.save_directory = save_directory
        self.pipeline_started = False
        self.frame_count = 0

        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

    def start_stream(self):
        if not self.pipeline_started:
            self.pipeline.start(self.config)
            self.pipeline_started = True

    def stop_stream(self):
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False

    def save_frames(self, num_frames=8, prefix=''):
        # Create the directory based on the prefix
        directory = os.path.join(self.save_directory, prefix, 'rgb')
        os.makedirs(directory, exist_ok=True)

        try:
            self.start_stream()
            while self.frame_count < num_frames:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                image = Image.fromarray(color_image)
                image_path = os.path.join(directory, f'{prefix}_image_{self.frame_count}.jpg')
                image.save(image_path)
                self.frame_count += 1
                print(f'Saved {image_path}')
        finally:
            self.stop_stream()

######################################################################################### 机械臂控制类
class RobotController:
    def __init__(self):
        # rospy.init_node('single_robot')
        rospy.on_shutdown(self.shutdown)
        self.pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)
        self.set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
        self.set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        self.init_subscriber()

        # Initialize task speed and acceleration
        self.set_velx(10, 10)
        self.set_accx(30, 30)

    def init_subscriber(self):
        self.subscriber_thread = threading.Thread(target=self.thread_subscriber)
        self.subscriber_thread.daemon = True
        self.subscriber_thread.start()

    def thread_subscriber(self):
        rospy.Subscriber('/' + ROBOT_ID + ROBOT_MODEL + '/state', RobotState, self.msgRobotState_cb)
        rospy.spin()

    def shutdown(self):
        print("Shutdown time!")
        self.pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
        return 0

    @staticmethod
    def msgRobotState_cb(msg):
        pass  # Implement the callback function as needed

    def move_joint(self, target_position, vel=20, acc=30):
        movel(target_position, vel=vel, acc=acc)

    def set_velx(self, vel_trans, vel_rot):
        set_velx(vel_trans, vel_rot)

    def set_accx(self, acc_trans, acc_rot):
        set_accx(acc_trans, acc_rot)
    
    '''''
    机械臂实例化调用例子

    robot = RobotController()
    
    # Define the target joint position
    target_joint_position = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    
    # Call the move_joint function to move to the defined position
    robot.move_joint(target_joint_position, vel=20, acc=5)
    
    rospy.spin()
    '''
    
######################################################################################### 灵巧手控制类
class DexterousHandController:
    def __init__(self):
        # Initialize a ROS Node
        # self.init_ros_node()
        # rospy.init_node('Joint_Speed_Position', anonymous=True)

        
        # Initialize a Publisher to the 'speed_position' Topic
        self.pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=10)
        
        # Initialize a list of 8 JointSetSpeedPos messages
        self.joint_message_list = [JointSetSpeedPos() for _ in range(8)]
        
        # Initialize an instance of JointListSetSpeedPos
        self.message_to_send = JointListSetSpeedPos()
        
        # Declaring a list to store the names of the joints
        self.joint_names = ['r_w_rotation', 'r_w_flexion', 'r_w_adduction', 'r_th_adduction', 
                            'r_th_flexion', 'r_ix_flexion', 'r_middle_flexion', 'r_ring_ltl_flexion']
        
        # Declaring default target position and speed
        self.target_position_list = [0, 100, 1700, 4095, 0, 0, 0, 0]
        self.target_speed_list = [10, 10, 10, 10, 1000, 10, 10, 10]
        self.joint_position = [2048] * 8

        # 发送消息来失能温度和过载
        self._disable_shutdown_conditions()
        
        # Populate the joint_message_list with initial values
        self._update_joint_messages()

        # Sleep for 1 second to ensure ROS initialization
        time.sleep(1)

    # def init_ros_node(self):
    #     if not rospy.core.is_initialized():
    #         rospy.init_node('Joint_Speed_Position_Setter', anonymous=True)

    def _update_joint_messages(self):
        """ Update the JointSetSpeedPos messages with current target positions and speeds """
        for name, position, speed, joint in zip(self.joint_names, self.target_position_list, self.target_speed_list, self.joint_message_list):
            joint.name = name
            joint.target_pos = position
            joint.target_speed = speed
        
        # Fill the JointListSetSpeedPos message with updated joint messages
        self.message_to_send.joints = self.joint_message_list

    def _disable_shutdown_conditions(self):
        message_to_send = SetShutdownCond()
        message_to_send.name = self.joint_names  # 设置关节名称
        message_to_send.temperature = False  # 失能温度
        message_to_send.overload = False  # 失能过载
        # self.shutdown_pub.publish(message_to_send)

    def set_target_positions(self, positions):
        """ Set new target positions and publish the message """
        if len(positions) != len(self.target_position_list):
            raise ValueError(f"Expected {len(self.target_position_list)} positions, got {len(positions)}")
        
        self.target_position_list = positions
        self._update_joint_messages()
        self.publish()

    def set_grasp_position(self, grasp_type):
        if grasp_type == 'pinch':
            self.joint_position = [2048, 2048, 2048, 3000, 3000, 1500, 1500, 1500]
        elif grasp_type == 'power':
            self.joint_position = [2048, 4095, 4095, 4095, 4095, 4095, 4095, 4095]
        elif grasp_type == 'power':
            self.joint_position = [2048, 3000, 3000, 2000, 2000, 2000, 2000, 2000]
        elif grasp_type == 'power':
            self.joint_position = [2048, 2500, 2500, 2500, 2500, 2500, 2500, 2500]


    def publish(self):
        """ Publish the joint position and speed commands """
        self.pub.publish(self.message_to_send)

    ''''
    灵巧手实例化调用例子

    controller = DexterousHandController()
    save_dir
    # Change target positions
    new_positions = [0, 200, 1500, 3000, 100, 500, 700, 800]
    controller.set_target_positions(new_positions)
    
    # Add some delay before exiting
    time.sleep(2)
    '''''

##################################################################################### 滑移检测模型预测函数
def test_module(params, model_path, data_path, grasp_configuration_tensor):

    # 图片处理
    transform_rgb = transforms.Compose([
        transforms.Resize((120, 160)),
        transforms.ToTensor()
    ])

    transform_tactile = transforms.Compose([
        transforms.Resize((150, 200)),
        transforms.ToTensor()
    ])


    # 检查是否使用 GPU
    if params['use_gpu'] == 1 and cuda_avail:
        device = torch.device("cuda:0")
        use_gpu = True
    else:
        device = torch.device("cpu")
        use_gpu = False

    # 加载模型
    if params['Modality'] == "Combined":
        NN_model, model_params = model_factory.get_model(params, use_gpu)
    
    # 加载预训练模型权重
    state_dict = torch.load(model_path, map_location=device)
    NN_model.load_state_dict(state_dict['model'])
    
    if use_gpu:
        NN_model = NN_model.cuda()

    NN_model.eval()
    
    # 预处理输入图像
    # data_path = '/home/xxb/Downloads/data'
    dataset = GraspingSlidingDataset(data_path, transform_rgb=transform_rgb, transform_tactile=transform_tactile)

    # 创建DataLoader
    dataloader = DataLoader(dataset, batch_size=1, shuffle=False)
    data = next(iter(dataloader))

    # 运行模型推理
    with torch.no_grad():
        if params['Modality'] == "Combined":
            output = NN_model(data[0], data[1], data[2], data[3], grasp_configuration_tensor)
        _, predicted = torch.max(output.data, 1)
    
    # 输出预测结果
    print(f"Output: {output}")
    print(f"Predicted: {predicted}")

    # 提取 output 中的最大值索引（类别）
    output_class = torch.argmax(output, dim=1).item()

    # 提取 predicted 中的值
    predicted_class = predicted.item()

    # 打印结果
    print(f"Output class: {output_class}")
    print(f"Predicted class: {predicted_class}")
    return output_class, predicted_class

####################################################################################resNet34模型预测函数
def predict_image_class(img_path,model_path,class_indices_path,device=None):
    if device is None:
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    data_transform = transforms.Compose(
        [transforms.Resize(256),
         transforms.CenterCrop(224),
         transforms.ToTensor(),
         transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

    # load image
    assert os.path.exists(img_path), "file: '{}' dose not exist.".format(img_path)
    img = Image.open(img_path)
    plt.imshow(img)
    # [N, C, H, W]
    img = data_transform(img)
    # expand batch dimension
    img = torch.unsqueeze(img, dim=0)

    # read class_indict
    assert os.path.exists(class_indices_path), "file: '{}' dose not exist.".format(class_indices_path)

    with open(class_indices_path, "r") as f:
        class_indict = json.load(f)

    # create model
    model = resnet34(num_classes=3).to(device)

    # load model weights
    assert os.path.exists(model_path), "file: '{}' dose not exist.".format(model_path)
    model.load_state_dict(torch.load(model_path, map_location=device))

    # prediction
    model.eval()
    with torch.no_grad():
        # predict class
        output = torch.squeeze(model(img.to(device))).cpu()
        predict = torch.softmax(output, dim=0)
        predict_cla = torch.argmax(predict).numpy()

    print_res = "class: {}   prob: {:.3}".format(class_indict[str(predict_cla)],
                                                 predict[predict_cla].numpy())
    plt.title(print_res)
    for i in range(len(predict)):
        print("class: {:10}   prob: {:.3}".format(class_indict[str(i)],
                                                  predict[i].numpy()))
    # plt.show()
    return class_indict[str(predict_cla)]

    '''
    使用示例1
    img_path = "../egg.jpeg"
    model_path = "./resNet34.pth"
    class_indices_path = './class_indices.json'

    predicted_class = predict_image_class(img_path, model_path, class_indices_path)
    print(f"The predicted class is: {predicted_class}")   ## 输出:The predicted class is: egg

    使用示例2
    current_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(current_dir, "egg.jpeg")
    model_path = os.path.join(current_dir, "resnet34", "resnet34.pth")
    class_indices_path = os.path.join(current_dir, 'class_indices.json')

    predicted_class = predict_image_class(img_path, model_path, class_indices_path)
    print(f"The predicted class is: {predicted_class}")
    '''

###################################################################################灵巧手抓取构型 Function to generate grasp configuration
def generate_grasp_configuration(predicted_class):
    grasp_configurations = {
        "egg": [0, 500, 1700, 4000, 0, 0, 0, 0],
        "paper cup": [100, 600, 1800, 4100, 50, 50, 50, 50],
        "tomato": [200, 700, 1900, 4200, 100, 100, 100, 100],
        # Add more configurations as needed
    }

    return grasp_configurations.get(predicted_class, [0, 0, 0, 0, 0, 0, 0, 0])

###################################################################################灵巧手抓取关节
def adjust_grasp_joint(predicted_class, target_position_list1):
    if predicted_class == 1:
        return target_position_list1
    elif predicted_class == 0:
        return [x + 50 for x in target_position_list1]
    elif predicted_class == 2:
        return [x - 50 for x in target_position_list1]
    else:
        return [0, 0, 0, 0, 0, 0, 0, 0]  # 默认返回一个全零的配置
############################################################################################ main 函数
def main():

    ## 获取当前文件的目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ## 实例化对象
    # save_dir = '/home/xxb/DeformableObjectsGrasping-master/src/seed_robotics/user_samples/data'
    save_dir = os.path.join(current_dir, 'data')
    realsense_saver = RealsenseImageSaver(save_dir)     ## 实例化相机类

    # rospy.init_node('robot_and_dexterous_hand_controller', anonymous=True)  ##初始化机械臂和灵巧手节点
    # robot_controller = RobotController()    ## 实例化机械臂
    # hand_controller = DexterousHandController()    ## 实例化灵巧手类

    #### 1 相机拍摄一张图片,对桌面物体分类,生成灵巧手大致抓取构型

    ## 1.1 相机拍摄图片并存储在指定目录
    # realsense_saver.save_frames(num_frames=1, prefix='classification')
    # ## 1.2 物体分类并输出预测类别
    img_dir = os.path.join(current_dir, 'data/classification/rgb')
    img_files = [f for f in os.listdir(img_dir) if os.path.isfile(os.path.join(img_dir, f))]

    if not img_files:
        raise FileNotFoundError(f"No image files found in directory: {img_dir}")

    img_path = os.path.join(img_dir, img_files[0])
    model_path = os.path.join(current_dir, 'resNet34/resNet34.pth')
    class_indices_path = os.path.join(current_dir, 'resNet34/class_indices.json')
    predicted_class = predict_image_class(img_path, model_path, class_indices_path)  ## predicted_class = egg/paper cup/tomato
    print(f"The predicted class is: {predicted_class}")
    ## 1.3 根据预测类别生成灵巧手抓取构型
    grasp_configuration = generate_grasp_configuration(predicted_class)
    print(f"The grasp configuration is: {grasp_configuration}")
    # hand_controller.set_target_positions(grasp_configuration)

    #### 2 机械臂和灵巧手达到指定位置1    
    # goal_pose1 = [725,112,345,6,107,-38]  # 示例位置
    # robot_controller.move_joint(goal_pose1,vel=20, acc=5)
    # time.sleep(2)

    #### 3 捏、轻提后，各采集视觉和触觉图片，总共32张图片，同时返回捏时灵巧手的8个关节数据
    target_position_list1 = [x + 500 for x in grasp_configuration] ##捏
    # hand_controller.set_target_positions(target_position_list1)
    # realsense_saver.save_frames(num_frames=8, prefix='grasping')
    # time.sleep(1)
    
    # goal_pose2 = [725,112,345,6,107,-38]  ##轻提
    # robot_controller.move_joint(goal_pose2,vel=20, acc=5)
    # realsense_saver = RealsenseImageSaver(save_dir)
    # realsense_saver.save_frames(num_frames=8, prefix='sliding')
    # time.sleep(1)

    # robot_controller.move_joint(goal_pose1,vel=20, acc=5)  ##机械臂退回到指定位置1
    # time.sleep(2)
    # hand_controller.set_target_positions(grasp_configuration)  ##灵巧手松手

    #### 4 将32张图片和8个关节数据作为输入, 预测ViViT模型输出
    # 加载配置文件
    with open('/home/xxb/DeformableObjectsGrasping-master/src/grasping_framework/config_cluster.yaml', 'r') as file:
        params = yaml.safe_load(file)
    # 模型路径
    model_path = '/home/xxb/DeformableObjectsGrasping-master/src/grasping_framework/Trained_Model/vivit_fdp_two/25_07_2024__16_45_01/vivit_fdp_two5.pt'
    # 输入图像路径
    data_path = save_dir
    # 8个关节转换成张量
    grasp_configuration_tensor = torch.tensor(grasp_configuration, dtype=torch.float32)
    grasp_configuration_tensor_2d = grasp_configuration_tensor.unsqueeze(0)

    # 测试图像
    output_class, predicted_class = test_module(params, model_path, data_path, grasp_configuration_tensor_2d)

    #### 5 ViViT预测结果作为输入, 灵巧手关节作为输出, 同时机械臂提起物体
    grasp_joint = adjust_grasp_joint(predicted_class, target_position_list1)
    print(f"The grasp configuration is: {grasp_joint}")
    # hand_controller.set_target_positions(grasp_joint)
    time.sleep(2)

    # goal_pose3 = [725,112,345,6,107,-38]  ##轻提
    # robot_controller.move_joint(goal_pose3,vel=20, acc=5)


if __name__ == "__main__":
    main()