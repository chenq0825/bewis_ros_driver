#ifndef SENSOR_VALUE_H
#define SENSOR_VALUE_H

#include <ros/ros.h>
#include <string>
#include <array>

// 传感器编号
extern int sen_num1;
extern int sen_num2;
extern int sen_num3;

// 传感器角度数据
extern double sensorx1;  // 传感器1 X轴角度（俯仰 Pitch）
extern double sensory1;  // 传感器1 Y轴角度（横滚 Roll）
extern double sensorz1;  // 传感器1 Z轴角度（航向 Heading）

extern double sensorx2;  // 传感器2 X轴角度
extern double sensory2;  // 传感器2 Y轴角度
extern double sensorz2;  // 传感器2 Z轴角度

extern double sensorx3;  // 传感器3 X轴角度
extern double sensory3;  // 传感器3 Y轴角度
extern double sensorz3;  // 传感器3 Z轴角度

#endif // SENSOR_VALUE_H
