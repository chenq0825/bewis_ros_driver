#ifndef ANGLE_PROCESS_H
#define ANGLE_PROCESS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include "bewis_ros_driver/sensor_value.h"

// Global sensor data - declarations
extern int sen_num1;
extern double sensorx1;
extern double sensory1;
extern double sensorz1;

extern int sen_num2;
extern double sensorx2;
extern double sensory2;
extern double sensorz2;

extern int sen_num3;
extern double sensorx3;
extern double sensory3;
extern double sensorz3;

using namespace Eigen;
using namespace std;

// Function declarations
void write_callback(const std_msgs::String::ConstPtr &msg, int test);
void sensor_data();
double cal_digger_angle(double *roll, double *pitch, int num);
void publish_sensor_tf(int sensor_id, double x, double y, double z);
void publish_imu_data(ros::Publisher &publisher, int sensor_id, double x, double y, double z);

// ROS Publishers - declarations
extern ros::Publisher sensor1_pub;
extern ros::Publisher sensor2_pub;
extern ros::Publisher sensor3_pub;
extern ros::Publisher angle_pub;

// TF Broadcaster - declaration
extern tf::TransformBroadcaster *tf_broadcaster;

#endif // ANGLE_PROCESS_H
