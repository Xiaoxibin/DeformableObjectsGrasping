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
import numpy as np
import transforms3d as tf3d
import math
from scipy.spatial.transform import Rotation as R

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
#import DR_init
#DR_init.__dsr__id = "dsr01"
#DR_init.__dsr__model = "m1013"
#from DSR_ROBOT import *

# for mulit robot 
########from DSR_ROBOT_MULTI import *
from DSR_ROBOT import *

_EPS = np.finfo(float).eps * 4.0

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop_r1.publish(stop_mode=STOP_TYPE_QUICK)
    # pub_stop_r2.publish(stop_mode=STOP_TYPE_QUICK)
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
# msgRobotState_cb_r2.count = 0

def thread_subscriber_r1(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r1)
    rospy.spin()
    #rospy.spinner(2)    

def thread_subscriber_r2(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r2)
    rospy.spin()
    #rospy.spinner(2)    

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

if __name__ == "__main__":
    rospy.init_node('m1509x1_amove_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m1509"

    r1 = CDsrRobot(robot_id1, robot_model1)
    pub_stop_r1 = rospy.Publisher('/'+ robot_id1 + robot_model1 +'/stop', RobotStop, queue_size=10)                   

    # t1 = threading.Thread(target=thread_subscriber_r1, args=(robot_id1, robot_model1))
    # t1.daemon = True 
    # t1.start()
    #----------------------------------------------------------------------
    x1, sol = r1.get_current_posx()
    q1 = r1.get_current_posj()
    print("current_pose_x: ", x1)
    print("current_pose_j: ", q1)
    

