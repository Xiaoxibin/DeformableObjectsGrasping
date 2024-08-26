#!/usr/bin/env python3

# User sample to clear the stiffness on one or several joints on a Right Hand

import rospy
import std_msgs.msg
from seed_robotics.msg import *
import time


rospy.init_node('Set_Joints_Stiffness', anonymous = True)
pub = rospy.Publisher('R_stiffness', JointListSetStiffness, queue_size = 10)
joint_message_list = [SetStiffness() for i in range (6)]
message_to_send = JointListSetStiffness()
joint_names = ['r_w_rotation','r_w_flexion','r_th_adduction','r_th_flexion','r_ix_flexion','r_ring_ltl_flexion']
# Note : 值在1-9之间
stiffness_values_list = [9, 9, 9, 9, 9, 9]

for name, stiffness, joint in zip(joint_names, stiffness_values_list, joint_message_list):
    joint.name = name
    joint.stiffness = stiffness


message_to_send.joints = joint_message_list
time.sleep(1)
pub.publish(message_to_send)
