# Bewis ROS Driver

北微倾角传感器 ROS 驱动程序，用于读取传感器数据并通过 ROS 发布。

## 功能特性

- 支持北微倾角传感器串口数据读取（波特率 115200）
- 支持 1-3 个传感器同时连接
- 发布 IMU 数据到 `/sensor1/imu`, `/sensor2/imu`, `/sensor3/imu`
- 发布挖掘机计算角度到 `/digger_angle`
- 发布 TF 坐标变换（`base_link` → `sensor_1/2/3`）
- 支持 RViz 可视化

## 硬件要求

| 项目 | 说明 |
|------|------|
| 传感器 | 北微倾角传感器（BewiS） |
| 接口 | USB 转串口模块 |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |

## 依赖项

```bash
# ROS 串口库
sudo apt-get install ros-$(rosversion -d)-serial

# Eigen 数学库
sudo apt-get install libeigen3-dev

# 传感器消息类型
sudo apt-get install ros-$(rosversion -d)-sensor-msgs

# TF 坐标变换
sudo apt-get install ros-$(rosversion -d)-tf
```

## 串口配置

### 获取串口权限

创建 udev 规则文件：
```bash
sudo nano /etc/udev/rules.d/70-ttyusb.rules
```

添加以下内容：
```bash
KERNEL=="ttyUSB[0-9]*", MODE="0666"
KERNEL=="ttyACM[0-9]*", MODE="0666"
```

重新加载规则：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 检查串口设备

```bash
# 查看所有 USB 串口设备
ls -l /dev/ttyUSB*

# 查看设备详细信息
dmesg | grep ttyUSB
```

## 编译安装

```bash
# 进入工作空间
cd /home/qin/bewis_ws

# 编译
catkin_make

# 加载环境变量
source devel/setup.bash

# 添加到 .bashrc 永久生效（可选）
echo "source /home/qin/bewis_ws/devel/setup.bash" >> ~/.bashrc
```

## 快速开始

### 方法 1：使用 rosrun

```bash
# 终端 1：启动 ROS master
roscore

# 终端 2：启动传感器节点
source /home/qin/bewis_ws/devel/setup.bash
rosrun bewis_ros_driver bewis_sensor
```

### 方法 2：使用 launch 文件

```bash
source /home/qin/bewis_ws/devel/setup.bash
roslaunch bewis_ros_driver bewis_sensor.launch
```

**注意**：当前代码默认使用 `/dev/ttyUSB0`。如需修改串口，请编辑 [src/bewis_sensor.cpp](src/bewis_sensor.cpp#L40)。

## 详细测试步骤

### 第一步：验证硬件连接

```bash
# 检查串口设备是否被识别
ls -l /dev/ttyUSB0

# 输出示例：
# crw-rw-rw- 1 root dialout 188, 0 Feb 10 12:00 /dev/ttyUSB0
```

### 第二步：使用调试工具测试（不含 ROS）

```bash
# 直接读取串口数据并打印到终端
rosrun bewis_ros_driver test_serial
```

预期输出：
```
=== 北微传感器串口测试工具 ===
打开串口: /dev/ttyUSB0 @ 115200 baud
串口打开成功！

开始读取数据 (按 Ctrl+C 退出)...

[原始数据] 77 0D 01 00 00 12 34 00 56 78 XX XX XX XX
[解码] 传感器1 - X角度: 12.34°, Y角度: 56.78°
[统计] 已接收 50 个数据包, 平均频率: 10.0 Hz
```

### 第三步：运行 ROS 节点

```bash
# 终端 1
roscore

# 终端 2
source /home/qin/bewis_ws/devel/setup.bash
rosrun bewis_ros_driver bewis_sensor
```

预期输出：
```
[ INFO] [1696876800.000000]: === Bewis Sensor Driver Starting ===
[ INFO] [1696876800.000100]: Configured port: /dev/ttyUSB0
[ INFO] [1696876800.000200]: Threads created. Waiting for sensor data...
[ INFO] [1696876800.100000]: Serial Port /dev/ttyUSB0 initialized successfully
[ INFO] [1696876800.100100]: Sensor 0 thread started
[ INFO] [1696876800.100200]: Bewis Sensor Node Started
[S1] X:   0.0° Y:   0.0° Z:   0.0°
```

**控制台输出格式说明**：
- `S1`: 传感器1（Sensor 1）
- `X`: 俯仰角(Pitch)，前仰为正，单位：度
- `Y`: 横滚角(Roll)，右翻为正，单位：度
- `Z`: 航向角(Heading/Yaw)，0-360°，顺时针增加，单位：度

### 第四步：查看 ROS 话题

```bash
# 终端 3：查看话题列表
rostopic list

# 输出：
# /digger_angle
# /rosout
# /rosout_agg
# /sensor1/imu
# /sensor2/imu
# /sensor3/imu
# /tf
```

### 第五步：查看传感器数据

```bash
# 查看 IMU 数据
rostopic echo /sensor1/imu

# 查看计算角度
rostopic echo /digger_angle

# 查看数据频率
rostopic hz /sensor1/imu

# 查看话题信息
rostopic info /sensor1/imu
```

### 第六步：查看 TF 变换

```bash
# 实时查看 TF 变换
rosrun tf tf_echo base_link sensor_1

# 生成 TF 树图
rosrun tf view_frames
# 会生成 frames.pdf 文件
```

### 第七步：RViz 可视化

```bash
# 方法 1：使用 launch 文件（自动加载配置）
roslaunch bewis_ros_driver bewis_sensor.launch use_rviz:=true

# 方法 2：手动启动 RViz
rosrun rviz rviz -d /home/qin/bewis_ws/src/bewis_ros_driver/rviz/sensor_view.rviz
```

RViz 配置包含以下显示项：
- **Grid** - 参考网格平面
- **TF** - 坐标系变换树（显示 base_link → sensor_0/1/2）
- **Sensor1/Sensor2/Sensor3** - 传感器姿态箭头（红/绿/蓝色）
- **Sensor Axes** - 传感器坐标轴（RGB = XYZ）

## 数据包格式

北微传感器 SEC225 数据包（**14 字节**）：

| 字节索引 | 内容 | 说明 | 示例值 |
|----------|------|------|--------|
| 0 | 帧头1 | 固定值 0x77 | 0x77 |
| 1 | 帧头2 | 固定值 0x0D | 0x0D |
| 2 | 传感器ID | 传感器编号 | 0x01 |
| 3 | 保留 | 保留字节 | 0x00 |
| 4 | X符号 | X轴符号(0x10=负) | 0x00 |
| 5-6 | X角度 | X轴角度数据(俯仰) BCD码 | 0x12 0x34 |
| 7 | Y符号 | Y轴符号(0x10=负) | 0x00 |
| 8-9 | Y角度 | Y轴角度数据(横滚) BCD码 | 0x56 0x78 |
| 10-12 | Z角度 | Z轴角度数据(航向) | 0x03 0x13 0x55 |
| 13 | 校验和 | CRC校验 | - |

### 角度计算公式

**X/Y 角度**（俯仰/横滚）采用 BCD 码格式：
```
角度 = (高字节/16*10 + 高字节%16 + 低字节/16*10 + 低字节%16/100) * 符号位
```
示例：`0x12 0x34` = `12.34°`

**Z 角度**（航向角）采用十六进制转十进制格式：
```
角度 = hexByteToDecimal(byte10) * 100 + hexByteToDecimal(byte11) + hexByteToDecimal(byte12) / 100
```
示例：`0x03 0x13 0x55` = `313.55°`
其中：`0x03` → 3, `0x13` → 13, `0x55` → 55

### 关于航向角 (Z/Heading)

**传感器规格**：SEC225 是带电子罗盘的倾角传感器，可以测量三个轴向的角度：
- **X（俯仰角/Pitch）**：绕 Y 轴旋转，前仰为正
- **Y（横滚角/Roll）**：绕 X 轴旋转，右翻为正
- **Z（航向角/Heading/Yaw）**：绕 Z 轴旋转，0-360° 顺时针增加

**Z 角度数据来源**：传感器内置电子罗盘（磁力计），通过磁场方向确定航向。

**数据格式**：Z 角度占用 3 个字节（字节 10-12），采用十六进制转十进制编码格式，而非 X/Y 的 BCD 码格式。

## 发布的话题

| 话题名称 | 类型 | 说明 |
|----------|------|------|
| `/sensor1/imu` | sensor_msgs/Imu | 传感器1的IMU数据（含方向和加速度） |
| `/sensor2/imu` | sensor_msgs/Imu | 传感器2的IMU数据（含方向和加速度） |
| `/sensor3/imu` | sensor_msgs/Imu | 传感器3的IMU数据（含方向和加速度） |
| `/digger_angle` | std_msgs/Float64 | 挖掘机计算角度 |
| `/tf` | tf2_msgs/TFMessage | 坐标变换 |

### IMU 消息内容

```bash
$ rostopic echo /sensor1/imu
header:
  seq: 123
  stamp: 1234567890.123456789
  frame_id: "sensor_1"
orientation:  # 姿态四元数（从Pitch/Roll计算）
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:  # 角速度（倾角传感器无陀螺仪，均为0）
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration:  # 线性加速度（单位: m/s²）
  x: 0.0098   # X轴加速度
  y: -0.0196  # Y轴加速度
  z: 9.8066   # Z轴加速度（静止时约 1g = 9.8 m/s²）
```

## TF 坐标系

```
base_link (父坐标系)
  ├─ sensor_1 (传感器1，位移: x=0.0)
  ├─ sensor_2 (传感器2，位移: x=0.1)
  └─ sensor_3 (传感器3，位移: x=0.2)
```

## 项目结构

```
bewis_ros_driver/
├── include/bewis_ros_driver/
│   ├── angle_process.h      # 角度处理和TF发布
│   ├── message_extractor.h  # 串口数据解析
│   └── sensor_value.h       # 传感器数据全局变量
├── src/
│   ├── angle_process.cpp    # 角度计算实现
│   ├── message_extractor.cpp# 全局变量定义
│   ├── bewis_sensor.cpp     # 主程序入口
│   ├── test_serial.cpp      # 调试工具
│   └── wake_sensor.cpp      # 传感器唤醒工具
├── scripts/
│   ├── quaternion_euler_converter.py  # 四元数转换工具
│   └── show_euler_angles.py           # 欧拉角显示工具
├── launch/
│   ├── bewis_sensor.launch            # 启动文件
│   └── view_euler_angles.launch       # 可视化启动文件
├── rviz/
│   └── sensor_view.rviz      # RViz配置
├── doc/
│   ├── COORDINATE_CONVERSION_GUIDE.md  # 坐标转换指南
│   ├── BW-AH200技术手册.pdf
│   └── SEC225_datasheet.pdf
├── CMakeLists.txt           # 构建配置
├── package.xml              # 包描述
└── README.md                # 本文档
```

## 线程架构

```
main()
├── ros::init()           # ROS 初始化
├── Thread 1: read_serial_port()
│   ├── connect()         # 连接串口
│   └── msg_sentence0()   # 读取并解析数据
│       └── 更新全局变量 (sensorx1, sensory1, etc.)
└── Thread 2: sensor_data()
    ├── 创建 Publisher
    └── 循环发布数据
        ├── 读取全局变量
        ├── 计算 digger_angle
        ├── 发布 IMU 消息
        └── 发布 TF 变换
```

## 调试工具

### test_serial - 串口调试工具

不依赖 ROS，直接读取串口数据：

```bash
rosrun bewis_ros_driver test_serial
```

功能：
- 显示原始十六进制数据
- 解析并显示角度值
- 统计数据包频率

### rqt_console - 日志查看

```bash
rosrun rqt_console rqt_console
```

### rqt_graph - 节点图

```bash
rosrun rqt_graph rqt_graph
```

## 常见问题

### 问题 1：权限被拒绝

**错误信息**：
```
Unable to open port /dev/ttyUSB0: Permission denied
```

**解决方案**：
```bash
# 临时修复
sudo chmod 666 /dev/ttyUSB0

# 永久修复：配置 udev 规则（见上文）
```

### 问题 2：串口已被占用

**错误信息**：
```
Failed to open port: Serial port busy
```

**解决方案**：
```bash
# 查找占用进程
sudo lsof | grep ttyUSB0

# 杀死进程
sudo kill -9 <PID>
```

### 问题 3：没有数据输出

**排查步骤**：
1. 确认传感器已上电
2. 检查串口连接：`ls -l /dev/ttyUSB0`
3. 使用 `test_serial` 工具测试
4. 检查波特率设置（115200）
5. 使用 `cutecom` 或 `minicom` 查看原始数据

### 问题 4：编译错误

**错误信息**：
```
error: 'mutex' is not a member of 'std'
```

**解决方案**：确认 CMakeLists.txt 中启用了 C++11
```cmake
add_compile_options(-std=c++11)
```

### 问题 5：TF 数据不显示

**解决方案**：
```bash
# 检查 TF 话题
rostopic echo /tf -n1

# 使用 tf_monitor 监控
rosrun tf tf_monitor

# 检查是否有 /tf_static
rostopic hz /tf
```

## 数据记录与回放

### 记录数据

```bash
mkdir -p ~/bagfiles
rosbag record -O ~/bagfiles/sensor_data.bag \
    /sensor1/imu /sensor2/imu /sensor3/imu \
    /digger_angle /tf
```

### 回放数据

```bash
rosbag play ~/bagfiles/sensor_data.bag

# 另一终端查看
rostopic echo /sensor1/imu
```

### 检查 bag 文件

```bash
# 查看文件信息
rosbag info ~/bagfiles/sensor_data.bag
```

## 配置参数

### 修改串口设备

编辑 [src/bewis_sensor.cpp](src/bewis_sensor.cpp#L40)：

```cpp
// 修改为目标串口
std::string port0 = "/dev/ttyUSB1";  // 或其他串口
```

### 修改发布频率

编辑 [src/angle_process.cpp](src/angle_process.cpp#L144)：

```cpp
ros::Rate r(50);  // 修改为其他频率，如 100
```

## License

TODO

## 作者

sy (sy@todo.todo)

## 更新日志

- **2024-02-11**: 根据 SEC225 数据手册修复数据包格式（14→18字节），添加加速度数据输出到 IMU 消息
- **2024-02-10**: 代码重构，修复 ROS 初始化问题
- **初始版本**: 基础驱动实现
