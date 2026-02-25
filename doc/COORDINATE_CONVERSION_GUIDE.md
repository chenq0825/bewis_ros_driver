# 四元数与欧拉角转换工具 - 完整使用指南

## 概述

本项目提供了三个工具来处理四元数(Quaternion)和欧拉角(Euler Angles)之间的转换：

1. **quaternion_euler_converter.py** - 命令行转换工具
2. **show_euler_angles.py** - ROS实时欧拉角显示工具
3. **view_euler_angles.launch** - 一键启动可视化

## 坐标系定义

| 轴 | ROS名称 | 物理含义 | 正方向 | 范围 |
|-----|---------|----------|--------|------|
| **X** | Pitch | 俯仰角 | **前仰为正** (前端抬起) | ±90° |
| **Y** | Roll | 横滚角 | **右翻为正** (向右倾斜) | ±180° |
| **Z** | Yaw | 航向角 | **顺时针增加** (从正北开始) | 0-360° |

```
        Z (上, 航向角)
        ↑
        |
        |___→ X (前, 俯仰角)
       /
      ↙
    Y (右, 横滚角)
```

**输出格式**：
```
[S1] X:   0.0° Y:   0.0° Z:   0.0°
```

- **X**: 俯仰角(Pitch)，前仰为正
- **Y**: 横滚角(Roll)，右翻为正
- **Z**: 航向角(Heading)，0-360°

## 工具1: 命令行转换工具

### 欧拉角转四元数

```bash
rosrun bewis_ros_driver quaternion_euler_converter.py --euler X Y Z
```

**参数说明**：X=Pitch(俯仰), Y=Roll(横滚), Z=Yaw(航向)

**示例**：
```bash
# 前仰10°，右翻5°，朝向东北45°
rosrun bewis_ros_driver quaternion_euler_converter.py --euler 10 5 45

# 后仰5°，左翻10°，朝向南180°
rosrun bewis_ros_driver quaternion_euler_converter.py --euler -5 -10 180
```

### 四元数转欧拉角

```bash
rosrun bewis_ros_driver quaternion_euler_converter.py --quat QX QY QZ QW
```

**示例**：
```bash
# 水平朝北 (qx=0, qy=0, qz=0, qw=1)
rosrun bewis_ros_driver quaternion_euler_converter.py --quat 0 0 0 1

# 从IMU消息读取四元数
rosrun bewis_ros_driver quaternion_euler_converter.py --quat 0.01 0.02 0.03 0.999
```

### 从ROS话题读取

```bash
# 方式1: 使用管道
rostopic echo /sensor1/imu --noarr | rosrun bewis_ros_driver quaternion_euler_converter.py --ros

# 方式2: 从文件读取
cat imu_data.txt | rosrun bewis_ros_driver quaternion_euler_converter.py --ros
```

## 工具2: ROS实时欧拉角显示

### 基本用法

```bash
# 使用默认话题 /sensor1/imu
rosrun bewis_ros_driver show_euler_angles.py

# 指定话题
rosrun bewis_ros_driver show_euler_angles.py _imu_topic:=/imu/data
```

### 输出格式

```
[#  1000] Roll:   5.23°  Pitch: -2.15°  Yaw:  45.67°  [东北 (NE)]
```

显示内容包括：
- **Roll**: 横滚角（右翻为正）
- **Pitch**: 俯仰角（前仰为正）
- **Yaw**: 航向角（0-360°，显示方向）

### 方向指示

| Yaw范围 | 方向 |
|---------|------|
| 337.5°-22.5° | 北 (N) |
| 22.5°-67.5° | 东北 (NE) |
| 67.5°-112.5° | 东 (E) |
| 112.5°-157.5° | 东南 (SE) |
| 157.5°-202.5° | 南 (S) |
| 202.5°-247.5° | 西南 (SW) |
| 247.5°-292.5° | 西 (W) |
| 292.5°-337.5° | 西北 (NW) |

## 工具3: 一键启动可视化

### 启动传感器+欧拉角显示

```bash
# 仅显示欧拉角
roslaunch bewis_ros_driver view_euler_angles.launch

# 同时启动RViz可视化
roslaunch bewis_ros_driver view_euler_angles.launch use_rviz:=true
```

### 仅启动传感器驱动

```bash
roslaunch bewis_ros_driver bewis_sensor.launch
```

## ROS话题说明

### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/sensor1/imu` | `sensor_msgs/Imu` | 传感器1的IMU数据（含四元数） |
| `/sensor1/imu_rpy` | `geometry_msgs/Vector3` | 传感器1的欧拉角数据（可选） |

### IMU消息结构

```cpp
sensor_msgs::Imu
├── header
│   ├── seq
│   ├── stamp
│   └── frame_id: "sensor1"
├── orientation  // 四元数
│   ├── x
│   ├── y
│   ├── z
│   └── w
└── orientation_covariance
```

## 代码集成示例

### C++中使用四元数

```cpp
#include <tf/transform_datatypes.h>

// 从欧拉角创建四元数
double roll = 5.0 * M_PI / 180.0;   // 右翻5°
double pitch = -10.0 * M_PI / 180.0; // 前仰10°
double yaw = 45.0 * M_PI / 180.0;   // 朝向45°

tf::Quaternion q;
q.setRPY(roll, pitch, yaw);

// 使用四元数
imu_msg.orientation.x = q.x();
imu_msg.orientation.y = q.y();
imu_msg.orientation.z = q.z();
imu_msg.orientation.w = q.w();

// 从四元数获取欧拉角
double r, p, y;
tf::Matrix3x3(q).getRPY(r, p, y);
// r, p, y 的单位是弧度
```

### Python中使用四元数

```python
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 四元数转欧拉角
qx, qy, qz, qw = 0.01, 0.02, 0.03, 0.999
(roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
# 返回弧度，需要转换为度
roll_deg = roll * 180 / 3.14159

# 欧拉角转四元数
(roll, pitch, yaw) = (5.0, -10.0, 45.0)  # 度
q = quaternion_from_euler(roll, pitch, yaw)  # 输入度
```

## 常见问题

### Q1: 为什么Yaw角度超过360°？

A: 四元数转欧拉角时，Yaw会被归一化到[0, 360)范围。如果需要连续角度（超过360°），需要自己维护角度累积。

### Q2: Pitch接近±90°时数据异常？

A: 这是**万向节锁**现象。当Pitch接近±90°时，Roll和Yaw变得不唯一。建议避免让传感器工作在±90°附近。

### Q3: 如何验证传感器数据是否正确？

A: 使用以下步骤验证：
1. 水平放置传感器，Pitch/Roll应该接近0°
2. 向右倾斜，Roll应为正值
3. 前端抬起，Pitch应为正值
4. 水平旋转，Yaw应从0°增加到360°

### Q4: 四元数和欧拉角哪个更好？

A:
- **四元数**：适合计算、插值、无万向节锁
- **欧拉角**：直观易读、便于调试显示

建议：内部使用四元数，显示时转换为欧拉角。

## 完整工作流程示例

```bash
# 1. 启动传感器驱动
roslaunch bewis_ros_driver bewis_sensor.launch

# 2. 在新终端查看欧拉角
rosrun bewis_ros_driver show_euler_angles.py

# 3. 在另一个终端查看原始数据
rostopic echo /sensor1/imu

# 4. 或者查看RViz可视化
roslaunch bewis_ros_driver view_euler_angles.launch use_rviz:=true
```

## 文件位置

| 文件 | 位置 |
|------|------|
| 转换工具 | `scripts/quaternion_euler_converter.py` |
| 显示工具 | `scripts/show_euler_angles.py` |
| Launch文件 | `launch/view_euler_angles.launch` |
| 说明文档 | `doc/COORDINATE_CONVERSION_GUIDE.md` |
| ROS集成 | `src/angle_process.cpp` (setRPY函数) |
