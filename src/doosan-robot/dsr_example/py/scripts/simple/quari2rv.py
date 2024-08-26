from util import quaternion_to_matrix,rm2rpy
import numpy as np
import time
import os
import yaml




def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion

    # 计算旋转矩阵的各个元素
    m00 = 1 - 2*y**2 - 2*z**2
    m01 = 2*x*y - 2*w*z
    m02 = 2*x*z + 2*w*y
    m10 = 2*x*y + 2*w*z
    m11 = 1 - 2*x**2 - 2*z**2
    m12 = 2*y*z - 2*w*x
    m20 = 2*x*z - 2*w*y
    m21 = 2*y*z + 2*w*x
    m22 = 1 - 2*x**2 - 2*y**2

    # 构建旋转矩阵
    rotation_matrix = np.array([[m00, m01, m02],
                                [m10, m11, m12],
                                [m20, m21, m22]])

    return rotation_matrix

def matrix_to_euler(matrix):
    """
    将旋转矩阵转换为欧拉角（yaw, pitch, roll）。
    返回的欧拉角单位为角度。
    """
    pitch = np.degrees(np.arcsin(-matrix[2, 0]))

    if np.cos(pitch) != 0:
        yaw = np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0]))
    else:
        yaw = 0.0

    roll = np.degrees(np.arctan2(matrix[2, 1], matrix[2, 2]))

    return yaw, pitch, roll


# calibration_root = "/home/hrc/Projects/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/predicted_results14.yaml"
# with open(calibration_root,'r') as f:
#     file_content = f.read()
# result_meta = yaml.load(file_content,yaml.FullLoader)
# print(result_meta)
# rotation = np.array(result_meta(['Rotation']))
# quaternion = np.array([result_meta[0],result_meta[1],result_meta[2],result_meta[3]])

# quaternion = [0.70906110287,-0.68111317304,0.1205468317,0.1370607]
# quaternion = [-0.23246611477,0.94373495127,0.0229469380,-0.234088199]
quaternion = [0.914331069178,-0.330743417,0.230869117,-0.03615160]
transmat =quaternion_to_matrix(quaternion)
print(transmat)
goal_rotation = matrix_to_euler(transmat)
print(goal_rotation)