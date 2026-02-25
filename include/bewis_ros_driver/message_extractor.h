#ifndef MESSAGE_EXTRACTOR_H
#define MESSAGE_EXTRACTOR_H

#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include "bewis_ros_driver/sensor_value.h"

// 串口对象数组（3个串口）
extern serial::Serial ser[3];
extern std::mutex serial_mutex[3];

// 数据包标识符
const std::string packet_1 = "0x77";
const std::string packet_2 = "0xd";

// 函数声明
void msg_sentence0(int test);
void msg_sentence1(int test);
void msg_sentence2(int test);
void rx_message_extractor(const std::string sentence);

/**
 * @brief 16进制转10进制
 * @param chr1 高位字节
 * @param chr2 低位字节
 * @param np 符号位（1或-1）
 * @return 转换后的十进制值
 */
inline double convert16to10(unsigned char chr1, unsigned char chr2, int np)
{
    int data1 = int(chr1);
    int data2 = int(chr2);

    double test2 = (int(data1 / 16 * 10) + data1 % 16 +
                   double(data2 / 16 * 10 + data2 % 16) / 100) * np;

    return test2;
}

/**
 * @brief 将十六进制字节当作十进制数字解析
 * @param hex 十六进制字节 (如 0x13 当作 "13" 解析)
 * @return 十进制数值
 *
 * 示例: 0x13 -> 13, 0x55 -> 55, 0x09 -> 9
 */
inline int hexByteToDecimal(unsigned char hex)
{
    int high = (hex >> 4) & 0x0F;  // 高4位
    int low = hex & 0x0F;          // 低4位
    return high * 10 + low;        // 当作十进制数字
}

/**
 * @brief 解析北微传感器数据（3轴：Pitch, Roll, Heading）
 * @param sensor_num 输出：传感器编号
 * @param anglex 输出：X轴角度 (Pitch)
 * @param angley 输出：Y轴角度 (Roll)
 * @param anglez 输出：Z轴角度 (Heading/航向角)
 * @param chrTemp 输入：原始数据缓冲区
 *
 * 数据格式（14字节）：
 * [0]=0x77, [1]=0x0d, [2]=传感器编号, [3]=状态,
 * [4]=X符号, [5][6]=X角度(Pitch), [7]=Y符号, [8][9]=Y角度(Roll)
 * [10][11][12]=Z角度(Heading), [13]=校验和
 *
 * 符号规则：
 * - chrTemp[4/7]=0x10 表示该轴原始输出为负值
 * - Pitch(X): 前仰为正，后仰为负
 * - Roll(Y):  右翻为正，左翻为负
 * - Heading(Z): 0-360°，顺时针增加（从正北开始）
 *
 * 注意：Heading角度字节10-12需要当作十进制数字解析
 *       anglez = byte10*100 + byte11 + byte12/100
 */
inline void DecodeIBEWISData(int *sensor_num, double *anglex, double *angley, double *anglez,
                             unsigned char chrTemp[])
{
    int np1 = 1;
    int np2 = 1;

    // 检查符号位（0x10表示负数）
    if(chrTemp[4] == 0x10)
        np1 = -1;
    if(chrTemp[7] == 0x10)
        np2 = -1;

    *sensor_num = int(chrTemp[2]);
    // 符号说明：前仰为正(Pitch)，右翻为正(Roll)
    // chrTemp[4/7]=0x10表示传感器输出的原始负值，需要取反以符合右手坐标系
    *anglex = convert16to10(chrTemp[5], chrTemp[6], np1);   // Pitch: 前仰为正
    *angley = convert16to10(chrTemp[8], chrTemp[9], np2);   // Roll: 右翻为正

    // 解析航向角（Heading）- 将字节10-12当作十进制数字解析
    *anglez = hexByteToDecimal(chrTemp[10]) * 100.0 +
              hexByteToDecimal(chrTemp[11]) +
              hexByteToDecimal(chrTemp[12]) / 100.0;
}

/**
 * @brief 连接串口设备
 * @param port 串口设备路径（如 "/dev/ttyUSB1"）
 * @param test 串口索引（0-2）
 */
inline void connect(std::string port, int test)
{
    try
    {
        ser[test].setPort(port);
        ser[test].setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);  // 50ms 超时，匹配传感器 60ms 发送频率
        ser[test].setTimeout(to);
        ser[test].open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port " << port << ": " << e.what());
        ros::Duration(5).sleep();
        return;
    }

    if(ser[test].isOpen())
    {
        ROS_INFO_STREAM("Serial Port " << port << " initialized successfully");
    }
    else
    {
        ROS_ERROR_STREAM("Could not initialize serial port " << port);
    }
}

/**
 * @brief 通用串口消息读取函数
 * @param test 串口索引（0-2）
 * @param buffer 数据缓冲区
 * @param sensor_num 输出：传感器编号
 * @param anglex 输出：X轴角度 (Pitch)
 * @param angley 输出：Y轴角度 (Roll)
 * @param anglez 输出：Z轴角度 (Heading)
 * @return 是否成功读取数据
 */
inline bool read_sensor_data(int test, unsigned char *buffer,
                             int &sensor_num, double &anglex, double &angley, double &anglez)
{
    std::lock_guard<std::mutex> lock(serial_mutex[test]);

    size_t available = ser[test].available();
    if(available == 0)
        return false;

    // 读取数据
    size_t n = ser[test].read(buffer, 28);
    if(n == 0)
        return false;

    static unsigned short usRxLength[3] = {0, 0, 0};
    static unsigned char chrBuffer[3][1000];

    // 添加到接收缓冲区
    for(size_t i = 0; i < n && usRxLength[test] < 1000; i++)
    {
        chrBuffer[test][usRxLength[test]++] = buffer[i];
    }

    // 查找并解析数据包（14字节）
    while(usRxLength[test] >= 14)
    {
        // 查找数据包头 0x77 0x0d
        if(chrBuffer[test][0] != 0x77 || chrBuffer[test][1] != 0x0d)
        {
            // 移除第一个字节，继续查找
            for(int i = 1; i < usRxLength[test]; i++)
                chrBuffer[test][i - 1] = chrBuffer[test][i];
            usRxLength[test]--;
            continue;
        }

        // 找到完整数据包（14字节）
        unsigned char chrTemp[14];
        memcpy(chrTemp, chrBuffer[test], 14);

        int num;
        double ax, ay, az;
        DecodeIBEWISData(&num, &ax, &ay, &az, chrTemp);

        sensor_num = num;
        anglex = ax;
        angley = ay;
        anglez = az;

        // 移除已处理的数据
        for(int i = 14; i < usRxLength[test]; i++)
            chrBuffer[test][i - 14] = chrBuffer[test][i];
        usRxLength[test] -= 14;

        return true;
    }

    return false;
}

#endif // MESSAGE_EXTRACTOR_H
