#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py example gripper] gripper test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)   

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 
import yaml
import time
from util import quaternion_matrix, rv2rm, rm2rpy
import numpy as np
import Doosan_grasp_util

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


def robotiq_2f_open():
    pass
    #srv_robotiq_2f_open()

def robotiq_2f_close():
    pass
    #srv_robotiq_2f_close()

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
    return 0

# convert list to Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res
 
if __name__ == "__main__":
    #----- set target robot --------------- 
    my_robot_id    = "dsr01"
    my_robot_model = "m1013"
    SET_ROBOT(my_robot_id, my_robot_model)

    rospy.init_node('pick_and_place_simple_py')
    rospy.on_shutdown(shutdown)


    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    
    #print 'wait services'
    #rospy.wait_for_service('/'+ROBOT_ID +ROBOT_MODEL+'/drl/drl_start')

    srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)
    
    # p0 = posj(0, 0, 0, 0, 0, 0)
    # p1 = posj(0, 0, 90, 0, 90, 0)
    # p2 = posj(180, 0, 90, 0, 90, 0)

    # x1 = posx(0, 0, -200, 0, 0, 0)
    # x2 = posx(0, 0, 200, 0, 0, 0)
    # velx = [50, 50]
    # accx = [100, 100]

    # while not rospy.is_shutdown():
    #     movej(p0, vel=60, acc=30)
    #     print("movej(p0)")
    #     wait(1)

    #     movej(p1, vel=60, acc=30)
    #     print("movej(p1)")
    #     wait(1)

    #     movel(x1, velx, accx, time=2, mod=DR_MV_MOD_REL)
    #     print("movel(x1)")
    #     wait(1)

    #     srv_robotiq_2f_move(0.8) #close
    #     #robotiq_2f_close()
    #     rospy.sleep(1)

    #     movel(x2, velx, accx, time=2, mod=DR_MV_MOD_REL)
    #     print("movel(x2)")
    #     wait(1)

    #     movej(p2, vel=60, acc=30)
    #     print("movej(p2)")
    #     wait(1)

    #     movel(x1, velx, accx, time=2, mod=DR_MV_MOD_REL)
    #     print("movel(x1)")
    #     wait(1)

    #     #robotiq_2f_open()
    #     srv_robotiq_2f_move(0) #open
    #     rospy.sleep(1)
    #     movel(x2, velx, accx, time=2, mod=DR_MV_MOD_REL)
    #     print("movel(x2)")
    #     wait(1)
    print('Doosan starts moving to initial pose!')
    print('******************************************')
    initialState = [-550.157,9.519,803.712,12.301,-75.693,19.404]
    movel(initialState, vel=200, acc=400)

    print('Doosan has reached the initial pose!')
    print('******************************************')
    time.sleep(2)

    # TF matrix of tool -> base
    pose_vec = [-550.157,9.519,803.712,12.301,-75.693,19.404] # ndarray (x, y, z, rx, ry, rz)

    translation1 = np.array([pose_vec[0], pose_vec[1], pose_vec[2]])
    rotation1 = rv2rm(pose_vec[3], pose_vec[4], pose_vec[5])

    trans_mat1 = np.zeros([4, 4])
    trans_mat1[0:3, 0:3] = rotation1
    trans_mat1[0:3, 3] = translation1
    trans_mat1[3, 3] = 1

    # TF matrix of camera -> tool
    calibration_results_root = "/home/hrc/.ros/easy_handeye/ur5_realsense_handeyecalibration_eye_on_hand.yaml"
    with open(calibration_results_root, 'r') as f:
        file_content = f.read()
    calib_results = yaml.load(file_content, yaml.FullLoader)

    transform = calib_results['transformation']
    translation2 = np.array([transform['x'], transform['y'], transform['z']])
    rotation2 = np.array([transform['qw'], transform['qx'], transform['qy'], transform['qz']])
    trans_mat2 = quaternion_matrix(rotation2)
    trans_mat2[0:3, 3] = translation2

    # TF matrix of obj -> camera
    item = results_meta['classID'][0]
    rotation3 = results_meta['Rotation'][0] #w, qx, qy, qz
    translation3 = results_meta['Translation'][0] #x, y, z
    trans_mat3 = quaternion_matrix(rotation3)
    trans_mat3[0:3, 3] = translation3

    # Compute TF matrix of obj -> base
    trans_mat4 = np.dot(trans_mat1, trans_mat2)
    trans_mat = np.dot(trans_mat4, trans_mat3)

    # Move the gripper to the goal pose
    goal_translation = trans_mat[0:3, 3]
    goal_rotation_matrix = trans_mat[0:3,0:3]
    goal_rotation = rm2rpy(goal_rotation_matrix)
   
    # ------------------------------------------
    # goal_rotation = np.array([0.065, 3.152, -0.053])
    goal_pose = np.append(goal_translation, goal_rotation)
    # move_to_tcp(goal_pose, tool_vel=0.10)
    movel(goal_pose, vel=200, acc=400)

    print('good bye!')
