#!/usr/bin/env python3
# 实现功能：
# 获取实时的指尖力数据，当距离传感器测量到达指定的距离后，发出指令让手指开始闭合，在抓握过程中自适应调整手指开合角度，来保证抓握的质量。

import rospy
import std_msgs.msg
from seed_robotics.msg import *
from sensor_pkg.msg import *
import time

# 规定期望力大小及范围
SENSOR_FORCE_TOLERANCE = 30 #mN
SENSOR_FORCE_TARGET = 30 #mN
TIME_DELAY       = 0.02 #seconds
POSITION_CHANGE  = 1

ACCEPTABLE_HIGH = SENSOR_FORCE_TARGET + SENSOR_FORCE_TOLERANCE
ACCEPTABLE_LOW  = SENSOR_FORCE_TARGET - SENSOR_FORCE_TOLERANCE
# Control class to be able to change the IR sensor value in the callback function
# Including 2 flags to control the main loop
class Control:
    def __init__(self):
        self.IR_sensor_value = 254
        self.start_flag      = False
        self.step2_flag      = False


# Initialize an instance of the Control class
control = Control()
# Initialize variables to contunuisly store to incoming informations
joint_list = [LoneJoint() for i in range (5)] 
sensor_list = []         


# 定义五个关节名称：r_th_adduction（拇指内收）, r_th_flexion（拇指屈伸）, r_ix_flexion, r_middle_flexion, r_ring_ltl_flexion
# 闭合手指 4095表示最大闭合程度
names_step_1            = ['r_th_adduction','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion']    # Name of the joints we want to move first
target_positions_step_1 = [2595, 2595, 2595, 2595]                # Maximum position value : closed position
target_speeds_step_1    = [20, 20, 20, 20]                         # Speed 0 : Highest speed. Speed 50 : Low speed
# 控制大拇指张开
names_step_2            = ['r_th_flexion']                        # Name of the joint to move on the 2nd step of the hand closing
target_positions_step_2 = [2500]                                  # Maximum position value : closed position
target_speeds_step_2    = [20]                                     # Speed 50 : Low speed
# 张开手指 0表示完全张开
names_step_3            = ['r_th_adduction','r_th_flexion','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_3 = [0, 0, 0, 0, 0]                         # Minimum position value : open position
target_speeds_step_3    = [10, 10, 10, 10, 10]                    # Speed 10 : very low speed

names_step_stiff = ['r_th_adduction','r_th_flexion','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion']
target_stiffness =      [8,8,5,5,5]


def getNameFromSensorID(id):
    if id == 0:                 #大拇指
        return 'r_th_flexion'
    elif id == 1:               #食指
        return 'r_ix_flexion'
    elif id == 2:               #中指
        return 'r_middle_flexion'
    elif id == 3 or id == 4:           #无名指 / 小拇指
        return 'r_ring_ltl_flexion'
    else:
        rospy.logwarn("Couldn't match sensor ID %d with its joint, joint name set to 'None'" % id)
        return 'None'


def buildSpeedPosMsg(names,target_positions,target_speeds):
    joint_list_msg = [JointSetSpeedPos() for i in range(len(names))]
    # 用到关节名称，目标位置，目标速度
    for name, position, speed, joint in zip(names, target_positions, target_speeds, joint_list_msg):
        joint.name = name
        joint.target_pos = position
        joint.target_speed = speed
    final_message = JointListSetSpeedPos()
    final_message.joints = joint_list_msg
    return final_message

def buildStiffnessMsg(names,stiffness_value):
    joint_message_list = [SetStiffness() for i in range (6)]

    for name, stiffness, joint in zip(names, stiffness_value, joint_message_list):
        joint.name = name
        joint.stiffness = stiffness
    
    message_to_send = JointListSetStiffness()
    message_to_send.joints = joint_message_list
    return message_to_send


def jointsCallback(joints_data):
    for joint in joints_data.joints:
        if joint.name == 'r_th_adduction':
            joint_list[0] = joint
        if joint.name == 'r_th_flexion':
            joint_list[1] = joint
        if joint.name == 'r_ix_flexion':
            joint_list[2] = joint
        if joint.name == 'r_middle_flexion':
            joint_list[3] = joint
        if joint.name == 'r_ring_ltl_flexion':
            joint_list[4] = joint

# IR测距传感器
def mainBoardCallback(main_board_data):
    for board in main_board_data.boards :
        if board.id == 30:
            control.IR_sensor_value = board.palm_IR_sensor
    #print("IR Sensor value = %d" % IR_sensor_value)

def sensorCallback(sensor_data):
    if len(sensor_list) == 0:
        for sensor in sensor_data.data:
            sensor_list.append(sensor)
    else:
        for index, sensor in enumerate(sensor_data.data):
            sensor_list[index] = sensor

# ------------------------------------------Initialize a ROS Node----------------------------------------------------

rospy.init_node('Joint_listener', anonymous = False)
rospy.Subscriber("R_Joints", AllJoints, jointsCallback)
rospy.Subscriber('R_Main_Boards', AllMainBoards, mainBoardCallback)
rospy.Subscriber('R_AllSensors',AllSensors,sensorCallback)
pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size = 10)
pub1 = rospy.Publisher('R_stiffness', JointListSetStiffness, queue_size = 10)


# 定义一个函数：根据当前位置，构建并发布一个消息
def stopStressing(joint):
    target_pos = joint.present_position
    joints = [JointSetSpeedPos()]
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1         # If targert speed = -1, then the parameter will be ignored
    message = JointListSetSpeedPos()
    message.joints = joints
    pub.publish(message)

# 根据当前位置和位置变化量，减小目标位置,
def decreaseStress(joint,pos_change):
    if joint.present_position < 201:
        print("Trying to decrease pos on joint %s that already has present position to %d" % (joint.name,joint.present_position))
        return
    target_pos = joint.present_position - pos_change
    joints = [JointSetSpeedPos()]
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1         # If targert speed = -1, then the parameter will be ignored
    message = JointListSetSpeedPos()
    message.joints = joints
    pub.publish(message)

# 根据当前位置和位置变化量，增加目标位置
def increaseStress(joint,pos_change):
    if joint.present_position > 2894:
        list_joints_too_far.append(joint.name)
        print("Trying to increase pos on joint %s that already has present position to %d" % (joint.name,joint.present_position))
        return
    target_pos = joint.present_position + pos_change
    joints = [JointSetSpeedPos()]
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1         # If targert speed = -1, then the parameter will be ignored
    message = JointListSetSpeedPos()
    message.joints = joints
    pub.publish(message)


def decreaseStiffness(joint,stiff_change):
    target_stiffness = joint.stiffness- stiff_change
    joints = [SetStiffness()]
    joints[0].name = joint.name
    joints[0].stiffness = target_stiffness
    message1 = JointListSetStiffness()
    message1.joints = joints
    pub1.publish(message1)




def compute_stiffness_change(abs_val):
   
    gain = (int(abs(abs_val - SENSOR_FORCE_TARGET)))/10
    if gain > 5:
        return 5
    else:
        return gain
  



def computeGain(abs_val):
    gain = int(abs(abs_val - SENSOR_FORCE_TARGET))
    if gain > 30:
        return 30
    else:
        return gain

list_joints_too_far = []
time.sleep(1)



# 主循环
while not rospy.is_shutdown():
    if control.start_flag is False:
        # 如果IR传感器数值小于20，则发布消息称可以开始执行 names_step_1 （四根手指开始抓握）
        if control.IR_sensor_value < 40:
            message = buildSpeedPosMsg(names_step_1,target_positions_step_1,target_speeds_step_1)
            print(message)
            pub.publish(message)
            
            control.start_flag = True
            time.sleep(1)

    elif control.step2_flag is False:
        # 发布消息开始 names_step_2 （闭合大拇指）
        message = buildSpeedPosMsg(names_step_2,target_positions_step_2,target_speeds_step_2)
        print(message)
        pub.publish(message)

        control.step2_flag = True
        time.sleep(1)

    if control.step2_flag is True:
        while not rospy.is_shutdown():
            # 一直检测每个传感器的压力值有没有到阈值
            message1 = buildStiffnessMsg(names_step_stiff, target_stiffness)
            print(message1)
            pub1.publish(message1)

            for sensor in sensor_list:
                joint_name = getNameFromSensorID(sensor.id)
                if sensor.id == 3:
                    abs_val = (sensor.abs + sensor_list[4].abs)/2
                if sensor.id == 4:
                    abs_val = (sensor.abs + sensor_list[3].abs)/2
                else:
                    abs_val = sensor.abs

                # 如果超过了规定的压力值，获取相应关节的名字
                if abs_val > ACCEPTABLE_HIGH :
                    corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                    if corresponding_joint:
                        gain = computeGain(abs_val)
                        gain_stiff = compute_stiffness_change(abs_val)
                    # 减小目标位置（张开一点）
                        decreaseStress(corresponding_joint[0],gain)
                        decreaseStiffness(corresponding_joint[0],gain_stiff)

                        time.sleep(TIME_DELAY)
                elif abs_val < ACCEPTABLE_LOW:
                    # 如果值没有到阈值
                    if not (joint_name in list_joints_too_far):
                        corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                        if corresponding_joint:
                            gain = computeGain(abs_val)
                    # 加大目标位置 （收缩一点）
                            increaseStress(corresponding_joint[0],gain)
                            time.sleep(TIME_DELAY)
                else:
                    # 如果压力值正好在规定范围内
                    corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                    if corresponding_joint:
                        stopStressing(corresponding_joint[0])
                        time.sleep(TIME_DELAY)
