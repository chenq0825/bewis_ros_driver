#include "bewis_ros_driver/message_extractor.h"
#include "bewis_ros_driver/sensor_value.h"

// 定义串口对象数组
serial::Serial ser[3];
std::mutex serial_mutex[3];

// 定义传感器数据全局变量
int sen_num1 = 0;
double sensorx1 = 0;
double sensory1 = 0;
double sensorz1 = 0;

int sen_num2 = 0;
double sensorx2 = 0;
double sensory2 = 0;
double sensorz2 = 0;

int sen_num3 = 0;
double sensorx3 = 0;
double sensory3 = 0;
double sensorz3 = 0;

// 定义ROS发布器
ros::Publisher sensor1_pub;
ros::Publisher sensor2_pub;
ros::Publisher sensor3_pub;
ros::Publisher angle_pub;

/**
 * @brief 串口0消息处理线程
 */
void msg_sentence0(int test)
{
    unsigned char buffer[1000];
    int sensor_num;
    double anglex, angley, anglez;

    ROS_INFO("Sensor 0 thread started");

    while(ros::ok())
    {
        ros::spinOnce();

        if(read_sensor_data(test, buffer, sensor_num, anglex, angley, anglez))
        {
            std::lock_guard<std::mutex> lock(serial_mutex[test]);
            sen_num1 = sensor_num;
            sensorx1 = anglex;
            sensory1 = angley;
            sensorz1 = anglez;
        }
    }
}

/**
 * @brief 串口1消息处理线程
 */
void msg_sentence1(int test)
{
    unsigned char buffer[1000];
    int sensor_num;
    double anglex, angley, anglez;

    ROS_INFO("Sensor 1 thread started");

    while(ros::ok())
    {
        ros::spinOnce();

        if(read_sensor_data(test, buffer, sensor_num, anglex, angley, anglez))
        {
            std::lock_guard<std::mutex> lock(serial_mutex[test]);
            sen_num2 = sensor_num;
            sensorx2 = anglex;
            sensory2 = angley;
            sensorz2 = anglez;
        }
    }
}

/**
 * @brief 串口2消息处理线程
 */
void msg_sentence2(int test)
{
    unsigned char buffer[1000];
    int sensor_num;
    double anglex, angley, anglez;

    ROS_INFO("Sensor 2 thread started");

    while(ros::ok())
    {
        ros::spinOnce();

        if(read_sensor_data(test, buffer, sensor_num, anglex, angley, anglez))
        {
            std::lock_guard<std::mutex> lock(serial_mutex[test]);
            sen_num3 = sensor_num;
            sensorx3 = anglex;
            sensory3 = angley;
            sensorz3 = anglez;
        }
    }
}

/**
 * @brief 消息提取器（用于调试）
 */
void rx_message_extractor(const std::string sentence)
{
    ROS_INFO("Received: %s", sentence.c_str());
}



