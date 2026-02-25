#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iomanip>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_quaternion");
    ros::NodeHandle nh;

    // 测试数据：从实际传感器读取的欧拉角
    double roll_deg = -1.78;   // 横滚角
    double pitch_deg = -2.25;  // 俯仰角
    double yaw_deg = 301.81;   // 航向角

    std::cout << "=== ROS Quaternion Conversion Test ===" << std::endl;
    std::cout << "Input Euler Angles (degrees):" << std::endl;
    std::cout << "  Roll:  " << roll_deg << "°" << std::endl;
    std::cout << "  Pitch: " << pitch_deg << "°" << std::endl;
    std::cout << "  Yaw:   " << yaw_deg << "°" << std::endl;
    std::cout << std::endl;

    // 转换为弧度
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    // 使用ROS的tf::Quaternion进行转换
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    std::cout << "ROS tf::Quaternion Output:" << std::endl;
    std::cout << std::fixed << std::setprecision(7);
    std::cout << "  qx: " << q.x() << std::endl;
    std::cout << "  qy: " << q.y() << std::endl;
    std::cout << "  qz: " << q.z() << std::endl;
    std::cout << "  qw: " << q.w() << std::endl;
    std::cout << std::endl;

    // 反向转换：从四元数转回欧拉角
    double roll_back, pitch_back, yaw_back;
    tf::Matrix3x3(q).getRPY(roll_back, pitch_back, yaw_back);

    std::cout << "Back to Euler Angles (degrees):" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Roll:  " << roll_back * 180.0 / M_PI << "°" << std::endl;
    std::cout << "  Pitch: " << pitch_back * 180.0 / M_PI << "°" << std::endl;
    std::cout << "  Yaw:   " << yaw_back * 180.0 / M_PI << "°" << std::endl;

    return 0;
}
