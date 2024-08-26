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

# for single robot 
#import DR_init
#DR_init.__dsr__id = "dsr01"
#DR_init.__dsr__model = "m1013"
#from DSR_ROBOT import *

# for mulit robot 
########from DSR_ROBOT_MULTI import *
from DSR_ROBOT import *

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

if __name__ == "__main__":
    rospy.init_node('m1509x2_amove_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m1509"
    # robot_id2 = "dsr02"; robot_model2 = "m1509"

    r1 = CDsrRobot(robot_id1,robot_model1)
    # r2 = CDsrRobot(robot_id2,robot_model2)

    pub_stop_r1 = rospy.Publisher('/'+ robot_id1 + robot_model1 +'/stop', RobotStop, queue_size=10)           
    # pub_stop_r2 = rospy.Publisher('/'+ robot_id2 + robot_model2 +'/stop', RobotStop, queue_size=10)           

    #t1 = threading.Thread(target=thread_subscriber_r1, args=(robot_id1, robot_model1))
    #t1.daemon = True 
    #t1.start()

    #t2 = threading.Thread(target=thread_subscriber_r2, args=(robot_id2, robot_model2))
    #t2.daemon = True 
    #t2.start()

    #----------------------------------------------------------------------
    JReady1 = posj(0, 30, -90, 0, -30, 0)
    JReady2 = posj(0, -30, 90, 0 , 30, 0)

    J00 = posj(-180, 0, -145, 0, -35, 0)


    J01r = posj(-180.0, 71.4, -145.0, 0.0, -9.7, 0.0)
    J02r = posj(-180.0, 67.7, -144.0, 0.0, 76.3, 0.0)
    J03r = posj(-180.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    J04r = posj(-90.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    J04r1 = posj(-90.0, 30.0, -60.0, 0.0, 30.0, -0.0)
    J04r2 = posj(-90.0, -45.0, 90.0, 0.0, -45.0, -0.0)
    J04r3 = posj(-90.0, 60.0, -120.0, 0.0, 60.0, -0.0)
    J04r4 = posj(-90.0, 0.0, -0.0, 0.0, 0.0, -0.0)

    J05r = posj(-144.0, -4.0, -84.8, -90.9, 54.0, -1.1)

    J07r = posj(-152.4, 12.4, -78.6, 18.7, -68.3, -37.7)
    J08r = posj(-90.0, 30.0, -120.0, -90.0, -90.0, 0.0)

    JEnd = posj(0.0, -12.6, 101.1, 0.0, 91.5, -0.0)

    dREL1 = posx(0, 0, 350, 0, 0, 0)
    dREL2 = posx(0, 0, -350, 0, 0, 0)

    velx = [0, 0]
    accx = [0, 0]

    vel_spi = [400, 400]
    acc_spi = [150, 150]


    amp = [0, 0, 0, 30, 30, 0]
    period = [0, 0, 0, 3, 6, 0]

    x01 = [423.6, 334.5, 651.2, 84.7, -180.0, 84.7]
    x02 = [423.6, 34.5, 951.2, 68.2, -180.0, 68.2]
    x03 = [423.6, -265.5, 651.2, 76.1, -180.0, 76.1]
    x04 = [423.6, 34.5, 351.2, 81.3, -180.0, 81.3]

    JMediumLeft = posj(0, 0, 90, 90, 0, 0)
    JMediumRight = posj(0, 0, -90, -90, 0, 0)


    J1 = posj(8.18, -42.45, -53.36, 1.26, -81.82, -0.00)
    J2 = posj(8.18, 42.45, 53.36, -1.26, 81.82, 0.00)

    J3 = posj(-7.41, -47.44, -40.60, 6.47, -81.78, 4.75)
    J4 = posj(-7.41, 47.44, 40.60, -6.47, 81.78, -4.75)


    # men kou shi 222222
    # init pos #######################################################################################
    # r2.amovej(JReady2, 40, 40)
    r1.amovej(JReady1, 40, 40)
    
    r1.set_digital_output(2,OFF)
    # r2.set_digital_output(2,OFF)

    r1.mwait(time=1)
    # r2.mwait(time=1)

    # ######### 22222 ##################################################

    # movej to grasp position
    p1 = posj(9.37, -12.79, -102.43, -5.01, -46.23, -79.43)
    p2 = posj(-1, 25.18, 81.32, 3.89, 71.4, -1.73)

    r1.movej(p1, 30, 30)
    # r2.movej(p2, 30, 30)

    # start to grasp
    r1.set_digital_output(2,ON)
    # r2.set_digital_output(2,ON)
    time.sleep(4)

    # zhong jian wei zhi 
    p2_5 = posj(10.32, -10.31, -79.21, -8.23, -77.31, -76.42)
    r1.movej(p2_5, 30, 30)

    p3 = posj(-10.81, 18.19, 72.79, 0.16, 81.53, -14.78)
    # r2.movej(p3, 30, 30)
    time.sleep(1)

    p4 = posj(-15.63, 47.53, 43.60, 0.17, 81.53, -14.78)
    # r2.movej(p4, 30, 30)
    time.sleep(1)

    # r2.set_digital_output(2,OFF)
    time.sleep(2)

    p3 = posj(-10.81, 18.19, 72.79, 0.16, 81.53, -14.78)
    # r2.movej(p3, 30, 30)
    time.sleep(1)

    p5 = posj(17.90, -48.70, -32.39, -2.86, -85.27, -76.42)
    r1.movej(p5, 30, 30)
    time.sleep(1)
   
    r1.set_digital_output(2,OFF)

    # zhong jian wei zhi 
    p2_5 = posj(10.32, -10.31, -79.21, -8.23, -77.31, -76.42)
    r1.movej(p2_5, 30, 30)

    p6 = posj(9.41, 36.98, 64.47, 1.69, 78.32, 11.20)
    # r2.movej(p6, 30, 30)
    time.sleep(1)

    # r2.set_digital_output(2,ON)
    time.sleep(4)
    
    p3 = posj(-10.81, 18.19, 72.79, 0.16, 81.53, -14.78)
    # r2.movej(p3, 30, 30)
    time.sleep(1)

    p7 = posj(-17.89, 39.18, 47.69, 5.83, 72.54, -20.88)
    # r2.movej(p7, 30, 30)
    time.sleep(1)
    

    # r2.set_digital_output(2,OFF)
    time.sleep(1)

    # r2.movej(JReady2, 40, 40)
    r1.movej(JReady1, 40, 40)
    
    r1.set_digital_output(2, OFF)
    # r2.set_digital_output(2,OFF)
    
    print('good bye!')





