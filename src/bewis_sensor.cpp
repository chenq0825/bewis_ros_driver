#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <thread>
#include "bewis_ros_driver/message_extractor.h"
#include "bewis_ros_driver/angle_process.h"
#include "bewis_ros_driver/sensor_value.h"

using namespace std;

// 线程的运行函数
void read_serial_port(std::string port, int test)
{
    ROS_INFO("Attempting to connect to %s...", port.c_str());
    connect(port, test);

    if (test == 0)
    {
      msg_sentence0(test);
    }
    else if (test == 1)
    {
      msg_sentence1(test);
    }
    else if (test == 2)
    {
      msg_sentence2(test);
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS
    ros::init(argc, argv, "bewis_sensor_driver");

    // 必须先创建 NodeHandle，才能在子线程中使用 ros::Time::now() 和 ROS_INFO
    ros::NodeHandle nh;

    // 检测可用的串口设备
    ROS_INFO("=== Bewis Sensor Driver Starting ===");

    // 使用 ttyUSB0 作为唯一的传感器
    std::string port0 = "/dev/ttyUSB0";

    ROS_INFO("Configured port: %s", port0.c_str());

    int test1 = 0;
    thread th1(read_serial_port, port0, test1);  // 只使用一个传感器
    thread th2(sensor_data);

    ROS_INFO("Threads created. Waiting for sensor data...");

    th1.join();
    th2.join();

    ROS_INFO("Bewis Sensor Driver Exiting");

    return 0;
}
