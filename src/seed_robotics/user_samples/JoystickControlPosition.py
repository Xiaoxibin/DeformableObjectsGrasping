#!/usr/bin/env python3

import rospy
from seed_robotics.msg import JointListSetSpeedPos, SetShutdownCond, JointSetSpeedPos
import time
import pygame
import csv
from datetime import datetime

class JoystickControl:
    def __init__(self):
        rospy.init_node('Joystick_Control', anonymous=True)
        self.pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=10)
        self.shutdown_pub = rospy.Publisher('R_shutdown_condition', SetShutdownCond, queue_size=10)

        self.joint_names = ['r_w_rotation', 'r_w_flexion', 'r_w_adduction', 'r_th_adduction', 'r_th_flexion',
                            'r_ix_flexion', 'r_middle_flexion', 'r_ring_ltl_flexion']

        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()

        self.target_positions = [2048] * 8
        self.target_speeds = [0] * 8

        # 发送消息来失能温度和过载
        self.disable_shutdown_conditions()

        self.update_message()

        # Initialize the CSV file
        self.init_csv()

    def init_csv(self):
        self.csv_file = open('joint_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = ['timestamp'] + [f'target_position_{i+1}' for i in range(8)]
        self.csv_writer.writerow(header)

    def update_message(self):
        message_to_send = JointListSetSpeedPos()
        message_to_send.joints = []

        for name, position, speed in zip(self.joint_names, self.target_positions, self.target_speeds):
            joint_msg = JointSetSpeedPos()
            joint_msg.name = name
            joint_msg.target_pos = position
            joint_msg.target_speed = speed
            message_to_send.joints.append(joint_msg)

        self.pub.publish(message_to_send)

    def disable_shutdown_conditions(self):
        message_to_send = SetShutdownCond()
        message_to_send.name = 'r_ring_ltl_flexion'  # 设置关节名称
        message_to_send.temperature = False  # 失能温度
        message_to_send.overload = False  # 失能过载
        self.shutdown_pub.publish(message_to_send)

    def log_data(self):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # Format the timestamp
        data = [timestamp] + self.target_positions
        self.csv_writer.writerow(data)
        self.csv_file.flush()  # Ensure data is written to the file immediately
        print(f"{timestamp},{','.join(map(str, self.target_positions))}")  # Print the joint data to the console

    def run(self):
        rate = rospy.Rate(5)  # Set the rate to 5 Hz
        try:
            while not rospy.is_shutdown():
                pygame.event.pump()
                axis_values = [self.joystick.get_axis(i) for i in range(self.num_axes)]

                # 控制最后三个关节的位置，使用手柄第一个轴
                for i in range(5, 8):
                    self.target_positions[i] = int((axis_values[0] + 1) / 2 * 4095)
                    self.target_positions[i] = min(max(self.target_positions[i], 0), 4095)
                for i in range(3, 5):
                    self.target_positions[i] = int((axis_values[i] + 1) / 2 * 4095)
                    self.target_positions[i] = min(max(self.target_positions[i], 0), 4095)

                self.update_message()
                self.log_data()
                rate.sleep()  # Sleep to maintain the desired rate
        except KeyboardInterrupt:
            self.csv_file.close()
            pass

if __name__ == "__main__":
    joystick_control = JoystickControl()
    joystick_control.run()
