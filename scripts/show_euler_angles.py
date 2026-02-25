#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时显示IMU话题的欧拉角
从ROS IMU消息订阅四元数并转换为欧拉角显示
"""

import rospy
import sys
import math
from sensor_msgs.msg import Imu


def quaternion_to_euler(qx, qy, qz, qw):
    """
    将四元数转换为欧拉角 (RPY顺序)
    返回: (roll, pitch, yaw) 度
    """
    # 归一化四元数
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        return 0.0, 0.0, 0.0

    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Roll (X轴旋转)
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    roll = math.degrees(roll)

    # Pitch (Y轴旋转)
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qx*qz))))
    pitch = math.degrees(pitch)

    # Yaw (Z轴旋转)
    yaw = math.atan2(2*(qx*qy + qw*qz), 1 - 2*(qy*qy + qz*qz))
    yaw = math.degrees(yaw)

    # 将yaw归一化到 [0, 360)
    if yaw < 0:
        yaw += 360.0

    return roll, pitch, yaw


class EulerAngleDisplay:
    """欧拉角显示器"""

    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.count = 0

    def imu_callback(self, msg):
        """IMU消息回调"""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        self.roll, self.pitch, self.yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.count += 1

        # 显示数据
        self.display()

    def display(self):
        """显示欧拉角"""
        # 使用ANSI转义码清除当前行
        sys.stdout.write('\033[K')
        sys.stdout.write('\r')

        # 格式化输出
        direction = ""
        if self.yaw >= 337.5 or self.yaw < 22.5:
            direction = "北 (N)"
        elif 22.5 <= self.yaw < 67.5:
            direction = "东北 (NE)"
        elif 67.5 <= self.yaw < 112.5:
            direction = "东 (E)"
        elif 112.5 <= self.yaw < 157.5:
            direction = "东南 (SE)"
        elif 157.5 <= self.yaw < 202.5:
            direction = "南 (S)"
        elif 202.5 <= self.yaw < 247.5:
            direction = "西南 (SW)"
        elif 247.5 <= self.yaw < 292.5:
            direction = "西 (W)"
        else:
            direction = "西北 (NW)"

        output = (f"\r[#{self.count:6d}] "
                 f"X:{self.pitch:7.2f}°  "
                 f"Y:{self.roll:7.2f}°  "
                 f"Z:{self.yaw:7.2f}°  "
                 f"[{direction}]")

        sys.stdout.write(output)
        sys.stdout.flush()

    def start(self):
        """启动订阅"""
        rospy.loginfo(f"Subscribing to {self.topic_name}...")
        rospy.loginfo("Press Ctrl+C to exit")

        # 创建订阅者
        rospy.Subscriber(self.topic_name, Imu, self.imu_callback)

        # 保持运行
        rospy.spin()


def main():
    rospy.init_node('show_euler_angles', anonymous=True)

    # 获取话题名称参数
    topic_name = rospy.get_param('~imu_topic', '/sensor1/imu')

    print("=" * 70)
    print("欧拉角实时显示工具")
    print("=" * 70)
    print(f"订阅话题: {topic_name}")
    print("坐标系约定:")
    print("  - X: 俯仰角(Pitch)，前仰为正")
    print("  - Y: 横滚角(Roll)，右翻为正")
    print("  - Z: 航向角(Heading)，0-360° 顺时针")
    print("=" * 70)

    display = EulerAngleDisplay(topic_name)
    display.start()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n\n退出")
