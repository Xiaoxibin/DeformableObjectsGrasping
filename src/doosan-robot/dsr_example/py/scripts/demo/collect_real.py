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

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 


from DSR_ROBOT import *

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
        
msgRobotState_cb_r1.count = 0

def msgRobotState_cb_r2(msg):
    msgRobotState_cb_r2.count += 1

    if (0==(msgRobotState_cb_r2.count % 100)): 
        rospy.loginfo("________ ROBOT[2] STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        

msgRobotState_cb_r2.count = 0

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
                print('当前位姿：',current_pos_str)
                f.write(current_pos_str + '\n')  

last_write_time = 0
# 当前六个关节角度   
def current_joint():
    global last_write_time
    with open('/home/hrc/Projects/catkin_ws/src/doosan-robot/realdata/collect_joint.txt', 'w') as f:
    #with open('/home/ubuntu/catkin_ws/src/dmp_generate/src/Collector/collect_joint.txt', 'w') as f:
        while(True):
            current_time = time.time()
            j_trq2 = r1.get_current_posj()
            if (current_time - last_write_time) >= 0.05:
                print('当前关节角：',j_trq2)
                current_joint_str = ' '.join(map(str, j_trq2))  
                f.write(current_joint_str + '\n')
                last_write_time = current_time

def do_pos():
    r1.set_ref_coord(DR_WORLD)
    
    start_pos = posj(-19.20,-23.68,118.18,47.34,32.90,-64.81)
    via_point1 = posj(-6.59,-11.06,99.08,60.81,17.78,-64.81)
    via_point2 = posj(-9.86,0.50,90.13,48.39,26.16,-55.93)
    via_point3 = posj(-9.83,22.37,75.95,39.08,27.53,-43.09)
    
    
    # object1 
    grasp_pos1 = posj(-9.29,40.28,69.35,38.65,27.15,-43.09)
    # object2 
    grasp_pos2 = posj(-16.77,40.08,68.03,24.00,27.15,-43.09)
    # object2 
    grasp_pos3 = posj(-7.21, 50.89, 51.74, 24.00, 28.29, -43.29)

    r1.movej(start_pos, v=5, a=5)
    r1.movej(via_point1, v=5, a=5, r=150)
    r1.movej(via_point2, v=5, a=5, r=100)
    r1.movej(via_point3, v=5, a=0.5, r=50)
    r1.movej(grasp_pos3, v=5, a=0.5, ra=DR_MV_RA_OVERRIDE)
    # movej(Q4, v=10, a=5, ra=DR_MV_RA_OVERRIDE)
    # movej(Q5, v=10, a=5, ra= DR_MV_RA_OVERRIDE)
    r1.movej(start_pos, v=5, a=5)


    
if __name__ == "__main__":
    #----- set target robot --------------- 
    rospy.init_node('m1509x2_amove_py')
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