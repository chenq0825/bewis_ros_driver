#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
四元数(Quaternion)与欧拉角(Euler Angles)转换工具
支持：Pitch(俯仰), Roll(横滚), Yaw(航向)

坐标系约定：
- X轴向前：Pitch为正时前端抬起（前仰）
- Y轴向右：Roll为正时向右倾斜（右翻）
- Z轴向上：Yaw为正时顺时针旋转（从正北开始）
"""

import numpy as np
import argparse
import sys

try:
    from geometry_msgs.msg import Quaternion
    from sensor_msgs.msg import Imu
except ImportError:
    print("Warning: ROS Python packages not found. Quaternion class will be unavailable.")
    Quaternion = None
    Imu = None


def quaternion_to_euler(qx, qy, qz, qw):
    """
    将四元数转换为欧拉角 (ZYX顺序，即RPY)

    参数:
        qx, qy, qz, qw: 四元数分量

    返回:
        (roll, pitch, yaw) 弧度
        - roll:  绕X轴旋转 (横滚角，右翻为正)
        - pitch: 绕Y轴旋转 (俯仰角，前仰为正)
        - yaw:   绕Z轴旋转 (航向角，0-360°，顺时针增加)
    """
    # 归一化四元数
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        print(f"Warning: Quaternion norm is too small: {norm}")
        return 0.0, 0.0, 0.0

    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Roll (X轴旋转) = atan2(2(qw*qx + qy*qz), 1 - 2(qx*qx + qy*qy))
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    roll = np.degrees(roll)

    # Pitch (Y轴旋转) = arcsin(2*(qw*qy - qx*qz))
    pitch = np.arcsin(2*(qw*qy - qx*qz))
    pitch = np.degrees(pitch)

    # Yaw (Z轴旋转) = atan2(2(qx*qy + qw*qz), 1 - 2(qy*qy + qz*qz))
    yaw = np.arctan2(2*(qx*qy + qw*qz), 1 - 2*(qy*qy + qz*qz))
    yaw = np.degrees(yaw)

    # 将yaw归一化到 [0, 360)
    if yaw < 0:
        yaw += 360.0

    return roll, pitch, yaw


def euler_to_quaternion(x, y, z):
    """
    将欧拉角转换为四元数 (ZYX顺序)

    参数:
        x: 俯仰角 Pitch (度)，前仰为正
        y: 横滚角 Roll (度)，右翻为正
        z: 航向角 Yaw (度)，0-360°，顺时针增加

    返回:
        (qx, qy, qz, qw): 四元数分量
    """
    # 转换为弧度
    pitch_rad = np.radians(x)  # X轴 = Pitch
    roll_rad = np.radians(y)   # Y轴 = Roll
    yaw_rad = np.radians(z)    # Z轴 = Yaw

    # 计算半角的sin/cos值
    # phi=roll, theta=pitch, psi=yaw
    cphi = np.cos(roll_rad * 0.5)
    sphi = np.sin(roll_rad * 0.5)
    ctheta = np.cos(pitch_rad * 0.5)
    stheta = np.sin(pitch_rad * 0.5)
    cpsi = np.cos(yaw_rad * 0.5)
    spsi = np.sin(yaw_rad * 0.5)

    # 计算四元数（与ROS tf::setRPY完全一致的公式）
    qw = cphi * ctheta * cpsi + sphi * stheta * spsi
    qx = sphi * ctheta * cpsi - cphi * stheta * spsi
    qy = cphi * stheta * cpsi + sphi * ctheta * spsi
    qz = cphi * ctheta * spsi - sphi * stheta * cpsi

    return qx, qy, qz, qw


def print_conversion(x, y, z, qx, qy, qz, qw):
    """打印转换结果"""
    print("=" * 60)
    print("欧拉角 (Euler Angles):")
    print(f"  X (Pitch, 俯仰):  {x:8.3f}°  前仰为正")
    print(f"  Y (Roll, 横滚):   {y:8.3f}°  右翻为正")
    print(f"  Z (Yaw, 航向):    {z:8.3f}°  0-360°，顺时针")
    print()
    print("四元数 (Quaternion):")
    print(f"  qx: {qx:10.7f}")
    print(f"  qy: {qy:10.7f}")
    print(f"  qz: {qz:10.7f}")
    print(f"  qw: {qw:10.7f}")
    print(f"  Norm: {np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw):.10f}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='四元数与欧拉角转换工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法：
  # 欧拉角转四元数 (X=Pitch, Y=Roll, Z=Yaw)
  python3 quaternion_euler_converter.py --euler 10 5 45

  # 四元数转欧拉角
  python3 quaternion_euler_converter.py --quat 0.01 0.02 0.03 0.999

  # 从ROS IMU消息读取
  rostopic echo /imu/topic --noarr | python3 quaternion_euler_converter.py --ros
        """)

    group = parser.add_mutually_exclusive_group(required=True)

    group.add_argument('--euler', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='欧拉角输入 (度): X(Pitch) Y(Roll) Z(Yaw)')
    group.add_argument('--quat', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='四元数输入: QX QY QZ QW')
    group.add_argument('--ros', action='store_true',
                        help='从标准输入读取ROS IMU消息 (格式: qx qy qz qw)')

    args = parser.parse_args()

    # 欧拉角转四元数
    if args.euler:
        x, y, z = args.euler  # X=Pitch, Y=Roll, Z=Yaw
        qx, qy, qz, qw = euler_to_quaternion(x, y, z)
        print_conversion(x, y, z, qx, qy, qz, qw)

        # 如果有ROS，创建示例消息
        if Quaternion is not None:
            q = Quaternion()
            q.x = qx
            q.y = qy
            q.z = qz
            q.w = qw
            print("\nROS Quaternion消息示例:")
            print(q)

    # 四元数转欧拉角
    elif args.quat:
        qx, qy, qz, qw = args.quat
        y, x, z = quaternion_to_euler(qx, qy, qz, qw)  # returns (roll, pitch, yaw)
        print_conversion(x, y, z, qx, qy, qz, qw)

    # 从ROS读取
    elif args.ros:
        print("从标准输入读取ROS IMU消息 (格式: qx qy qz qw)")
        print("输入示例: 0.01 0.02 0.03 0.999")
        print("按 Ctrl+D 退出\n")

        try:
            for line in sys.stdin:
                parts = line.strip().split()
                if len(parts) >= 4:
                    try:
                        qx, qy, qz, qw = map(float, parts[:4])
                        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
                        x, y, z = pitch, roll, yaw  # 转换为 XYZ 顺序

                        # 覆盖输出
                        print(f"\r[X:{x:7.2f}° Y:{y:7.2f}° Z:{z:7.2f}°]", end='')
                        sys.stdout.flush()
                    except ValueError as e:
                        print(f"\n解析错误: {e}")
        except KeyboardInterrupt:
            print("\n\n退出")


if __name__ == '__main__':
    main()
