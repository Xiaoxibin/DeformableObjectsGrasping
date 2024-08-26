#!/usr/bin/env python3


import rospy
import std_msgs.msg
from seed_robotics.msg import *
from sensor_pkg.msg import *
import time
import tkinter as tk
import datetime
import os


SENSOR_FORCE_TOLERANCE = 30 #mN
SENSOR_FORCE_TARGET = 30 #mN
TIME_DELAY       = 0.02 #seconds
POSITION_CHANGE  = 1

ACCEPTABLE_HIGH = SENSOR_FORCE_TARGET + SENSOR_FORCE_TOLERANCE
ACCEPTABLE_LOW  = SENSOR_FORCE_TARGET - SENSOR_FORCE_TOLERANCE


root = tk.Tk()
root.title("Sensor Data Display")

def update_display(force_string):
    # 清除之前的数据显示
    display_label.config(text="")
    # 更新为新的数据
    display_label.config(text=force_string)

# 创建一个标签来显示数据
display_label = tk.Label(root, text="", width=50, anchor='w')
display_label.pack()

class Control:
    def __init__(self):
        self.IR_sensor_value = 254
        self.start_flag      = False
        self.step2_flag      = False


# Initialize an instance of the Control class
control = Control()


joint_list = [LoneJoint() for i in range (5)]  
sensor_list = []                             

names_step_1            = ['r_th_adduction','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion']                # Name of the joints we want to move first
target_positions_step_1 = [2000, 2000, 2000, 2000]                                                    # Maximum position value : closed position
target_speeds_step_1    = [20, 20, 20, 20]                                                           # Speed 0 : Highest speed. Speed 50 : Low speed

names_step_2            = ['r_th_flexion']                                                      # Name of the joint to move on the 2nd step of the hand closing
target_positions_step_2 = [2500]                                                                # Maximum position value : closed position
target_speeds_step_2    = [10]                                                                  # Speed 50 : Low speed

names_step_3            = ['r_th_adduction','r_th_flexion','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_3 = [0, 0, 0, 0, 0]                                                          # Minimum position value : open position
target_speeds_step_3    = [10, 10, 10, 10, 10]                                                      # Speed 10 : very low speed


def getNameFromSensorID(id):
    if id == 0:
        return 'r_th_flexion'
    elif id == 1:
        return 'r_ix_flexion'
    elif id == 2:
        return 'r_middle_flexion'
    elif id == 3 or id == 4:
        return 'r_ring_ltl_flexion'
    else:
        rospy.logwarn("Couldn't match sensor ID %d with its joint, joint name set to 'None'" % id)
        return 'None'

# Define a function to fill the message 'final_message' to send based on lists of names, target position values and target speed values
def buildSpeedPosMsg(names,target_positions,target_speeds):
    # Initialize a list of JointSetSpeedPos messages, the length of the number of joints we want to send instruction to
    joint_list_msg = [JointSetSpeedPos() for i in range(len(names))]
    # Fill up that list with the informations about name, position and speed that are listed above
    for name, position, speed, joint in zip(names, target_positions, target_speeds, joint_list_msg):
        joint.name = name
        joint.target_pos = position
        joint.target_speed = speed
    # Declare, fill up and return an instance of JointListSetSpeedPos message, ready to be sent
    final_message = JointListSetSpeedPos()
    final_message.joints = joint_list_msg
    return final_message


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

def mainBoardCallback(main_board_data):
    for board in main_board_data.boards :
        if board.id == 30:
            control.IR_sensor_value = board.palm_IR_sensor
    #print("IR Sensor value = %d" % IR_sensor_value)

def initialize_file_name():
    now = datetime.datetime.now()
    filename = f"current_finger_force_{now.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
    return os.path.join('/home/ubuntu/catkin_ws/src/data/real/', filename)

# 全局变量，用于存储文件路径
file_path = initialize_file_name()

def write_to_file(force_string):
    with open(file_path, 'a') as f:  # 使用全局的文件路径
        f.write(force_string + '\n')


def sensorCallback(sensor_data):
    # 处理传感器数据并更新sensor_list
    if len(sensor_list) == 0:
        for sensor in sensor_data.data:
            sensor_list.append(sensor)  # 假设sensor是一个对象，其中包含fz属性
    else:
        for index, sensor in enumerate(sensor_data.data):
            sensor_list[index] = sensor

    print(sensor_list)
    force_values = [sensor.fz for sensor in sensor_list[:5]]  # 获取前5个sensor的fz属性值
    force_string = ' '.join(map(str, force_values))  # 将数值转换为字符串并用空格连接


    # 写入文件
    write_to_file(force_string)



rospy.init_node('Joint_listener', anonymous = False)
rospy.Subscriber("R_Joints", AllJoints, jointsCallback)
rospy.Subscriber('R_Main_Boards', AllMainBoards, mainBoardCallback)
rospy.Subscriber('R_AllSensors',AllSensors,sensorCallback)
pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size = 10)


def stopStressing(joint):
    # Getting the joint's present position
    target_pos = joint.present_position
    # Declare a list of 1 JointSetSpeedPos element
    joints = [JointSetSpeedPos()]
    # Fill the JointSetSpeedPos instance with joint's name, new target position and target_speed
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1         # If targert speed = -1, then the parameter will be ignored
    message = JointListSetSpeedPos()
    message.joints = joints
    pub.publish(message)

def decreaseStress(joint,pos_change):
    # Getting the joint's present position
    if joint.present_position < 201:
        print("Trying to decrease pos on joint %s that already has present position to %d" % (joint.name,joint.present_position))
        return
    target_pos = joint.present_position - pos_change
    joints = [JointSetSpeedPos()]
    # Fill the JointSetSpeedPos instance with joint's name, new target position and target_speed
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1       

    message = JointListSetSpeedPos()
    # Fill that message and publish it
    message.joints = joints
    pub.publish(message)

def increaseStress(joint,pos_change):
    # Getting the joint's present position
    if joint.present_position > 3894:
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

def computeGain(abs_val):
    gain = int(abs(abs_val - SENSOR_FORCE_TARGET)/10)
    if gain > 5:
        return 5
    else:
        return gain

# Declaring a empty list to store future stressed joints
list_joints_too_far = []

# Sleeping for 1sec to let ROS initialize
time.sleep(1)

# Main Loop
while not rospy.is_shutdown():

    if control.start_flag is False:
   
        if control.IR_sensor_value < 40:

            message = buildSpeedPosMsg(names_step_1,target_positions_step_1,target_speeds_step_1)
            print(message)
            pub.publish(message)

            control.start_flag = True

            time.sleep(1)

    elif control.step2_flag is False:

        message = buildSpeedPosMsg(names_step_2,target_positions_step_2,target_speeds_step_2)
        print(message)
        pub.publish(message)
 
        control.step2_flag = True
        time.sleep(1)

    if control.step2_flag is True:
        while not rospy.is_shutdown():
            # Continuously check each sensor's value to see if the value is above the threshold
            for sensor in sensor_list:
                joint_name = getNameFromSensorID(sensor.id)
 
                if sensor.id == 3:
                    abs_val = (sensor.abs + sensor_list[4].abs)/2
                if sensor.id == 4:
                    abs_val = (sensor.abs + sensor_list[3].abs)/2
                else:
                    abs_val = sensor.abs
                
                
                if abs_val > ACCEPTABLE_HIGH :
                # If the value is above the threshold, get the corresonding joint's name
                    corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                    if corresponding_joint:
                    # Compute the gain
                        gain = computeGain(abs_val)
                    # Set its target position to a lower value
                        decreaseStress(corresponding_joint[0],gain)
                        time.sleep(TIME_DELAY)
                elif abs_val < ACCEPTABLE_LOW:
                # If the value is below the threshold
                    if not (joint_name in list_joints_too_far):
                # If that joint is not already at its minimum position
                # Find its name
                        corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                        if corresponding_joint:
                    # Compute the gain
                            gain = computeGain(abs_val)
                    # Increasae its target position
                            increaseStress(corresponding_joint[0],gain)
                            time.sleep(TIME_DELAY)
                else:
                    # If the value is inside our wanted pressure interval
                    corresponding_joint = [joint for joint in joint_list if joint.name == joint_name]
                    if corresponding_joint:
                        # Set its target position to its current position
                        stopStressing(corresponding_joint[0])
                        time.sleep(TIME_DELAY)
