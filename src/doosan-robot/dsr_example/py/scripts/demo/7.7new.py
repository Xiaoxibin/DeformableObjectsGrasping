#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py demo]  muliti robot sync test
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import time
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 
import yaml
import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import transforms3d as tf3d

# for single robot 
import DR_init
DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "m1509"
from DSR_ROBOT import *

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop_r1.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb_r1(msg):
    msgRobotState_cb_r1.count += 1

    if (0==(msgRobotState_cb_r1.count % 100)): 
        rospy.loginfo("________ ROBOT[1] STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        
        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb_r1.count = 0

def msgRobotState_cb_r2(msg):
    msgRobotState_cb_r2.count += 1

    if (0==(msgRobotState_cb_r2.count % 100)): 
        rospy.loginfo("________ ROBOT[2] STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        
        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))

def thread_subscriber_r1(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r1)
    rospy.spin()
    #rospy.spinner(2)    

def thread_subscriber_r2(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r2)
    rospy.spin()
    #rospy.spinner(2)    

# 当前末端位姿
def current_pos():
    #with open('/home/ubuntu/catkin_ws/src/dmp_generate/src/Collector/collect_pos.txt', 'w') as f:
    with open('/home/hrc/Projects/catkin_ws/src/doosan-robot/realdata/collect_pos.txt', 'w') as f:
           while True:
                pos = r1.get_current_posx()  # 读取位置信息
                current_po = pos[0]  # 选择位姿
                current_pos_str = ' '.join(map(str, current_po))  
                # print('当前位姿：',current_pos_str)
                f.write(current_pos_str + '\n') 

def do_pos():
    r1.set_digital_output(2, OFF)
    r1.mwait(time=1)            
    print('Doosan starts moving to initial pose!')
    print("------------------------------------------------------------------------------")
    # initialState, sol = r1.get_current_posx()

    start_pos = posx(313.96,-52.92,355.63,172.29,-125.77,173.00)
    r1.movel(start_pos, vel=20, acc=10)
    
    print('Doosan has moved to initial pose!')
    print("------------------------------------------------------------------------------")
    time.sleep(3)

    # TF matrix of tool -> base
    ds_cur_pose, sol = r1.get_current_posx() # (x, y, z, A, B, C)
    pose_vec = ds_cur_pose
    translation1 = np.array([pose_vec[0] * 0.001, pose_vec[1] * 0.001, pose_vec[2] * 0.001])
    rot = tf3d.euler.euler2quat(math.radians(pose_vec[3]), math.radians(pose_vec[4]), math.radians(pose_vec[5]), 'rzyz')
    rotation1 = tf3d.quaternions.quat2mat([rot[0], rot[1], rot[2], rot[3]])
    tool_in_base = tf3d.affines.compose(translation1, rotation1, [1, 1, 1])
    print("current arm pose: \n", tool_in_base)
    print("------------------------------------------------------------------------------")

    # TF matrix of camera -> tool
    # translation2 = np.array([0.016631, -0.0815948, -0.0944967])            
    translation2 = np.array([-0.10562 , -0.00981357, -0.0786502])
    rotation2 = tf3d.euler.euler2mat(math.radians(-2.69979), math.radians(-1.46317), math.radians(-90.3172))
    camera_in_tool = tf3d.affines.compose(translation2, rotation2, [1, 1, 1])
    print("camera in tool pose: \n", camera_in_tool)
    print("------------------------------------------------------------------------------")

    # compute camera pose in arm base coordinate sysytem
    camera_in_base = np.dot(tool_in_base, camera_in_tool)
    camera_base_euler = [math.degrees(float(i)) for i in tf3d.euler.mat2euler(camera_in_base)]
    print("camera_in_base:\n", camera_in_base)
    print("------------------------------------------------------------------------------")

    # TF matrix of obj -> camera
    predicted_results_root = "/home/hrc/Projects/FoundationPose/pose.yaml"
    with open(predicted_results_root, 'r') as f:
        file_content = f.read()
    results_meta = yaml.load(file_content, yaml.FullLoader)
    rotation3_q = results_meta['Rotation'][0] # qx, qy, qz, w
    rotation3 = tf3d.quaternions.quat2mat([rotation3_q[3], rotation3_q[0], rotation3_q[1], rotation3_q[2]])
    translation3 = results_meta['Translation'][0] # x, y, z
    obj_in_camera = tf3d.affines.compose(translation3, rotation3, [1, 1, 1])
    # obj_camera_euler = [math.degrees(float(i)) for i in tf3d.euler.mat2euler(obj_in_camera)]
    print("obj_in_camera: \n", obj_in_camera)
    print("------------------------------------------------------------------------------")

    # Compute TF matrix of obj -> base
    obj_in_base = np.dot(camera_in_base, obj_in_camera)
    obj_base_translation = obj_in_base[0:3, 3]
    obj_base_euler = [math.degrees(float(i)) for i in tf3d.euler.mat2euler(obj_in_base, 'rzyz')]
    print("obj in arm_base pose: \n", obj_in_base)
    print("------------------------------------------------------------------------------")

    # Move the gripper to the goal pose
    obj_base_translation = obj_base_translation * 1000
    goal_pose = np.concatenate((obj_base_translation, obj_base_euler))
    goal_pose = list(goal_pose)
    print("obj_base_translation: ", obj_base_translation)
    print("obj_base_euler: ", obj_base_euler)
    print("goal_pose: ", goal_pose)
    

##################
    # 没有用视觉检测之前的
    # # object1 
    # grasp_pos1 = posj(-9.29,40.28,69.35,38.65,27.15,-43.09)
    # # object2 
    # grasp_pos2 = posj(-16.77,40.08,68.03,24.00,27.15,-43.09)
    # # object2 
    # grasp_pos3 = posj(-7.21, 50.89, 51.74, 24.00, 28.29, -43.29)
    
    via_point1 = posx(441.93, -19.35, 409.16, 176.58, -123.61, 168.57)
    via_point2 = posx(584.35, -64.11, 457.23, 174.49, -127.11, 169.27)
    # via_point3 = posx(794.05,-2.81,325.82,10.16,119.02,0.64)

    goal_pose = posx(goal_pose[0]-40, goal_pose[1]-50, goal_pose[2]+10, 3.70, 129.310, 6.02)
    print("goal_pose: ", goal_pose)

    via_point4 = posx(615.74,-169.02,368.58,171.37,-121.52,-175.40)
    place_pos = posx(822.56, -418.28, 98.34, 164.74, -128.20, 178.29)
    
    r1.movel(via_point1, v=10, a=5, r=50)
    print("robot has reached near the via_point1! ")
    r1.movel(via_point2, v=10, a=5, r=100)
    print("robot has reached near the via_point2! ")
    print("now is ready to grasp the object! ")
    #r1.movel(via_point3, v=10, a=5, r=50)
    #print("robot has reached near the via_point3! ")
    r1.movel(goal_pose, v=10, a=5, ra=DR_MV_MOD_REL)
    print("now have grasped the object! ")
    time.sleep(2)

    r1.movel(via_point4, v=10, a=5, ra=DR_MV_MOD_REL)
    # r1.movel(place_pos, v=10, a=10,ra=DR_MV_MOD_REL)


    # movej(Q4, v=10, a=5, ra=DR_MV_RA_OVERRIDE)
    # movej(Q5, v=10, a=5, ra= DR_MV_RA_OVERRIDE)
    # r1.movej(start_pos, v=5, a=5)
    r1.mwait(time=1)




if __name__ == "__main__":
    rospy.init_node('m1509x1_amove_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m1509"

    r1 = CDsrRobot(robot_id1,robot_model1)

    pub_stop_r1 = rospy.Publisher('/'+ robot_id1 + robot_model1 +'/stop', RobotStop, queue_size=10)
  
    
    thread1 = threading.Thread(target=current_pos)
    #thread2 = threading.Thread(target=current_joint)
    thread3 = threading.Thread(target=do_pos)

    thread1.start()
    #thread2.start()
    thread3.start()

    thread1.join()
    #thread2.join()
    thread3.join()

