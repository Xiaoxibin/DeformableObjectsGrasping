import ctypes
import datetime
import logging
import os
import time
from typing import List

import matplotlib.pyplot as plt
import numpy as np


class Doosan:
    NUM_JOINTS = 6

    def __init__(self, address, home=None,
                 libpath='/media/zjb/extend/zjb/pythonCodes/RobotGPT/real_robot/libdoosan.so'):
        """
        初始化斗山实体机器人控制类代码。
        :param address: 连接斗山机械臂的IP地址 "192.168.5.110"
        :param home: 斗山机械臂的初始位置
        :param libpath: 斗山机械臂的底层c++代码库
        """
        if home is None:
            home = [0, 0, 90, 0, 0, -270]
        assert isinstance(address, str), "The address must be a string!"

        self._address = bytes(address, 'utf-8')
        self._libpath = libpath
        self._home = home

        self._isInit = False
        self._isInitLog = False
        self._isLoop = None

        self._doosan = None

        self._Log('')

        self.Initialize()

    def MoveJoint(self, jointsList: List, velocity, acceleration):
        """
        基于关节角的机械臂运动。
        :param jointsList: 列表：控制关节角达到的目标位置。
        :param velocity: 浮点数：关节角速度。
        :param acceleration: 浮点数：关节角加速度。
        :return: 返回是否进行成功。
        """
        if isinstance(velocity, int):
            velocity = float(velocity)
        if isinstance(acceleration, int):
            acceleration = float(acceleration)

        jointsList_ = self.__ctypes_encode(jointsList)
        velocity_ = self.__ctypes_encode(velocity)
        acceleration_ = self.__ctypes_encode(acceleration)

        flag = self._doosan.robotMoveJoint(jointsList_, velocity_, acceleration_)
        print(f"[REAL] Move joint to {jointsList} successfully!")
        self._Log(f"Move joint {jointsList}, velocity: {velocity}, acceleration: {acceleration}.")
        return flag

    def MoveLine(self, poseture: List, velocity, acceleration):
        """
        基于末端位姿的机械臂运动。
        :param poseture: 列表：控制末端位姿达到的目标位置。
        :param velocity: 列表：关节角速度。
        :param acceleration: 列表：关节角加速度。
        :return: 返回是否进行成功。
        """
        poseture_ = self.__ctypes_encode(poseture)
        velocity_ = self.__ctypes_encode(velocity)
        acceleration_ = self.__ctypes_encode(acceleration)

        flag = self._doosan.robotMoveEndEffector(poseture_, velocity_, acceleration_)

        self._Wait(2)
        print(f"[REAL] Move line to {poseture} successfully!")
        self._Log(f"Move line {poseture}, velocity: {velocity}, acceleration: {acceleration}")
        return flag

    def grasp(self, onoff: bool):
        if onoff:
            # grasp
            self.GripperControl(True)
            self._Log("Grasp!")
        else:
            # release
            self.GripperControl(False)
            self._Log("Release!")

    def ReadJoint(self):
        """
        读取当前时刻关节角度值。
        :return: 关节角度值的numpy格式。
        """
        temp = self._doosan.robotReadJoint()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadJoint, {results}")
        print(f"[REAL] Read joint {results}")
        return results

    def ReadEndEffector(self):
        """
        读取当前时刻机械臂末端位姿。
        :return: 机械臂末端位姿的numpy格式。
        """
        temp = self._doosan.robotReadEndEffector()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadEndEffector, {results}")
        print(f"[REAL] Read end effector {results}")
        return results

    def ReadJointTorque(self):
        """
        读取当前时刻机械臂各个关节的力矩。
        :return: 机械臂各个关节力矩的numpy格式。
        """
        temp = self._doosan.robotReadJointTorque()
        results = self.__ctypes_decode(temp)
        results = np.round(results, 2)
        self._Log(f"Get the result, ReadJointTorque, {results}")
        print(f"[REAL] Read joint torque {results}")
        return results

    def GripperControl(self, openoff: bool):
        assert isinstance(openoff, bool), "[REAL] The variable input must be bool like!"
        self._doosan.robotGripperControl(ctypes.c_bool(openoff))
        self._Log(("Close " if openoff else "Open ") + "the gripper")
        print(("[REAL] Close " if openoff else "Open ") + "the gripper")
        pass

    def _Wait(self, times: int):
        self._doosan.robotWait(ctypes.c_int(times))
        self._Log(f"Wait {times} seconds")
        print(f"[REAL] Wait {times} seconds")

    def _Legacy(self):
        """
        此函数是跨平台使用，不写！
        :return:
        """
        pass

    def Home(self, velocity=20.0, acceleration=40.0):
        goal = self._home

        goal_ = self.__ctypes_encode(self._home)
        velocity_ = self.__ctypes_encode(velocity)
        acceleration_ = self.__ctypes_encode(acceleration)

        flag = self._doosan.robotMoveJoint(goal_, velocity_, acceleration_)
        self.GripperControl(True)

        print("[REAL] Move home position successfully!")
        self._Log(f"Move home {goal}, velocity: {velocity}, acceleration: {acceleration}.")
        return flag

    def Initialize(self):
        """
        载入动态链接库。
        通过IP地址连接机械臂。
        终端显示连接成功。
        将初始化信息写入日志。
        将机器人移动到初始位置中。
        终端显示初始化成功。
        :return:
        """
        self._doosan = ctypes.CDLL(self._libpath)
        self._isInit = self._doosan.robotInitialize(self._address, len(self._address))
        print(f"[REAL] Welcome to Doosan Robot!\nYour address is: {self._address}")
        self._isInit = True

        self._Log(f"{self._address}")
        self._Log(f"{self._libpath}")
        self._Log(f"{self._home}(joint angles).")

        self.Home()

        self.__Variable_declaration()

    def QuitLoop(self):
        self._isLoop = self._doosan.robotQuitLoop()
        self._Log("User quit the loop")
        print("[REAL] Quit the loop!")
        pass

    def Disconnect(self):
        self._Log("Disconnected.")
        self._doosan.robotDisconnect()
        pass

    def _Log(self, infos: str):
        if self._isInit is False and self._isInitLog is False:
            folder_name = 'RunningInfos'
            path = os.path.join(os.getcwd(), folder_name)
            if not os.path.exists(path):
                os.makedirs(path)
            logging.basicConfig(
                filename=f'./{folder_name}/{datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}.runtime_logs',
                level=logging.DEBUG,
                format="%(asctime)s - %(levelname)s - %(message)s")
        else:
            logging.info(infos)
        pass

    def __ctypes_encode(self, obj):
        if isinstance(obj, int):
            temp = ctypes.c_int(obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        elif isinstance(obj, float):
            temp = ctypes.c_float(obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        elif isinstance(obj, list):
            temp = (ctypes.c_float * len(obj))(*obj)
            self._Log(f"Change the {obj} to the ctypes {temp}")
        else:
            self._Log(f"Wrong in {obj} changing to the ctypes!")
            raise "The value(s) you input maybe wrong!"
        return temp

    def __ctypes_decode(self, res) -> np.array:
        """
        这个函数只在 ReadJoint、ReadEndEffector 和 ReadJointTorque 中才能被使用！
        :param res: 调用动态链接库获得的结果
        :return:
        """
        array_pointer = ctypes.cast(res, ctypes.POINTER(ctypes.c_float))
        self._Log("Generate the ctypes.POINTER of {res} to array pointer")
        array = np.ctypeslib.as_array(array_pointer, shape=(self.NUM_JOINTS,))
        self._Log("Generate the numpy's results array")
        return array

    def __Variable_declaration(self):
        self._doosan.robotReadJoint.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadJoint return type: ctypes.POINTER")
        self._doosan.robotReadJointTorque.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadJointTorque return type: ctypes.POINTER")
        self._doosan.robotReadEndEffector.restype = ctypes.POINTER(ctypes.c_float)
        self._Log("Set ReadEndEffector return type: ctypes.POINTER")
        print("Some methods have been reset return types successfully!")

    def Read_plot_torques(self, num: int, ter: float, plot: bool):
        """
        用于画出各个关节力矩变化情况。
        :param num: 读取的次数；
        :param ter: 读取的间隔；
        :param plot: 是否绘制出来。
        :return:
        """
        folder_name = 'torque_record'
        _temp_list = []
        path = os.path.join(os.getcwd(), folder_name)
        if not os.path.exists(path):
            os.makedirs(path)
        fntxt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.txt'
        fnpng = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.png'
        fd = open(path + '/' + fntxt, 'w')

        index = 0
        self._Log("Start reading and ploting the torques...")
        while True:
            index += 1
            print(index)
            time.sleep(ter)
            res_list = self.ReadJointTorque().tolist()
            if plot:
                _temp_list.append(res_list)
            fd.write(str(res_list) + "\n")
            if index > num:
                break
        fd.close()
        self._Log("Reading and ploting the torques data saved")
        if plot:
            plt.figure()
            plt.plot(_temp_list)
            plt.legend(["1", "2", "3", "4", "5", "6"])
            plt.savefig(path + '/' + fnpng)
            self._Log("Reading and ploting the torques plot saved")
            plt.show()

    def keyborad_control(self):
        from pynput import keyboard

        # 监听键盘键入
        with keyboard.Events() as events:
            for event in events:

                # 监听esc键，释放esc键，停止监听。
                if event.key == keyboard.Key.esc:
                    print("[REAL] End.")
                    break
                else:
                    if isinstance(event, keyboard.Events.Press):
                        if event.key == keyboard.Key.right:
                            pos = self.ReadEndEffector().tolist()
                            pos[0] += 50.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.left:
                            pos = self.ReadEndEffector().tolist()
                            pos[0] -= 50.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.up:
                            pos = self.ReadEndEffector().tolist()
                            pos[1] += 50.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.down:
                            pos = self.ReadEndEffector().tolist()
                            pos[1] -= 50.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.page_up:
                            pos = self.ReadEndEffector().tolist()
                            pos[2] += 10.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.page_down:
                            pos = self.ReadEndEffector().tolist()
                            pos[2] -= 10.0
                            self.MoveLine(pos, [40.0, 40.0], [80.0, 80.0])
                        elif event.key == keyboard.Key.alt:
                            # grasp
                            self.grasp(True)
                            pass
                        elif event.key == keyboard.Key.space:
                            # release
                            self.grasp(False)
                            pass
                        else:
                            pass
