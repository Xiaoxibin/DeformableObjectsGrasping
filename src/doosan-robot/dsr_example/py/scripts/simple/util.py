import numpy as np
import math

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

def rv2rm(rx, ry, rz):
    theta = np.linalg.norm([rx, ry, rz])
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    

    c = np.cos(theta)
    s = np.sin(theta)
    v = 1 - c

    R = np.zeros((3, 3))
    R[0][0] = kx * kx * v + c
    R[0][1] = kx * ky * v - kz * s
    R[0][2] = kx * kz * v + ky * s

    R[1][0] = ky * kx * v + kz * s
    R[1][1] = ky * ky * v + c
    R[1][2] = ky * kz * v - kx * s

    R[2][0] = kz * kx * v - ky * s
    R[2][1] = kz * ky * v + kx * s
    R[2][2] = kz * kz * v + c

    return R



def rm2rpy(matrix):
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





def rpy2rm(roll, pitch, yaw):
    # 将角度转换为弧度
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    # 计算正弦和余弦值
    cr = np.cos(roll_rad)
    sr = np.sin(roll_rad)
    cp = np.cos(pitch_rad)
    sp = np.sin(pitch_rad)
    cy = np.cos(yaw_rad)
    sy = np.sin(yaw_rad)

    # 构建旋转矩阵
    rotation_matrix = np.array([
        [cp*cy, -cp*sy, sp],
        [sr*sp*cy + cr*sy, -sr*sp*sy + cr*cy, -sr*cp],
        [-cr*sp*cy + sr*sy, cr*sp*sy + sr*cy, cr*cp]
    ])

    return rotation_matrix


def rm2rv(R):
    theta = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1) / 2)
    K = (1 / (2 * np.sin(theta))) * np.asarray([R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]])
    r = theta * K
    return r


def rv2rpy(rx, ry, rz):
    R = rv2rm(rx, ry, rz)
    rpy = rm2rpy(R)
    return rpy


def rpy2rv(roll, pitch, yaw):
    R = rpy2rm(roll, pitch, yaw)
    rv = rm2rv(R)
    return rv

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
    rotation_matrix = np.array([[m00, m01, m02,0],
                                [m10, m11, m12,0],
                                [m20, m21, m22,0],
                                [0,0,0,1]])

    return rotation_matrix
