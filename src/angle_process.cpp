#include "bewis_ros_driver/angle_process.h"
#include "bewis_ros_driver/message_extractor.h"
#include "bewis_ros_driver/sensor_value.h"

// 定义TF广播器
tf::TransformBroadcaster *tf_broadcaster = nullptr;

/**
 * @brief 计算挖掘机角度
 * @param roll 横滚角数组
 * @param pitch 俯仰角数组
 * @param num 传感器数量
 * @return 计算得到的角度
 */
double cal_digger_angle(double *roll, double *pitch, int num)
{
    double angle = 0;

    double pitch0 = pitch[0];
    double roll0 = roll[0];
    double pitch1 = pitch[1];
    double roll1 = roll[1];

    int np = 1;
    if(pitch0 * roll0 < 0)
        np = -1;

    int np2 = 1;
    if(pitch1 * roll1 < 0)
        np2 = -1;

    return angle = sqrt(pitch0 * pitch0 + roll0 * roll0) * np -
                   sqrt(pitch1 * pitch1 + roll1 * roll1) * np2;
}

/**
 * @brief 发布传感器TF变换
 * @param sensor_id 传感器ID
 * @param x X轴角度 (Pitch)
 * @param y Y轴角度 (Roll)
 * @param z Z轴角度 (Heading/Yaw)
 */
void publish_sensor_tf(int sensor_id, double x, double y, double z)
{
    if (tf_broadcaster == nullptr)
        return;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();

    std::string frame_id = "sensor_" + std::to_string(sensor_id);
    transform.header.frame_id = "base_link";
    transform.child_frame_id = frame_id;

    // 将角度转换为弧度
    double roll = y * M_PI / 180.0;
    double pitch = x * M_PI / 180.0;
    double yaw = z * M_PI / 180.0;  // 使用航向角作为 Yaw

    // 创建四元数
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // 设置平移（可根据实际传感器位置调整）
    transform.transform.translation.x = sensor_id * 0.1;  // 每个传感器间隔10cm
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    tf_broadcaster->sendTransform(transform);
}

/**
 * @brief 发布IMU消息
 * @param publisher ROS Publisher
 * @param sensor_id 传感器ID
 * @param x X轴角度 (Pitch)
 * @param y Y轴角度 (Roll)
 * @param z Z轴角度 (Heading/Yaw，航向角)
 */
void publish_imu_data(ros::Publisher &publisher, int sensor_id, double x, double y, double z)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "sensor_" + std::to_string(sensor_id);

    // 将角度转换为弧度
    double roll = y * M_PI / 180.0;
    double pitch = x * M_PI / 180.0;
    double yaw = z * M_PI / 180.0;  // 使用传感器输出的航向角

    // 创建方向四元数
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    // 设置角速度（倾角传感器无陀螺仪，设为0）
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    // 设置线性加速度（传感器不输出加速度数据，设为0）
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    publisher.publish(imu_msg);
}

/**
 * @brief 写入串口回调函数
 */
void write_callback(const std_msgs::String::ConstPtr &msg, int test)
{
    ROS_INFO_STREAM("Writing to serial port " << test << ": " << msg->data);
    ser[test].write(msg->data);
}

/**
 * @brief 传感器数据处理线程
 * 该函数处理传感器数据并发布到ROS话题
 */
void sensor_data()
{
    // 创建节点句柄（ROS已在main中初始化）
    ros::NodeHandle nh;

    // 创建TF广播器
    tf_broadcaster = new tf::TransformBroadcaster();

    // 创建发布器（队列大小设为1，只保留最新数据，降低延迟）
    sensor1_pub = nh.advertise<sensor_msgs::Imu>("sensor1/imu", 1);
    sensor2_pub = nh.advertise<sensor_msgs::Imu>("sensor2/imu", 1);
    sensor3_pub = nh.advertise<sensor_msgs::Imu>("sensor3/imu", 1);
    angle_pub = nh.advertise<std_msgs::Float64>("digger_angle", 1);

    ros::Rate r(17);  // 17Hz ≈ 60ms间隔，匹配传感器输出频率

    ROS_INFO("Bewis Sensor Node Started");
    ROS_INFO("Publishing to: sensor1/imu, sensor2/imu, sensor3/imu");
    ROS_INFO("Publishing to: digger_angle");

    while (ros::ok())
    {
        double digger_angle;

        double pitch1 = sensorx1;
        double pitch2 = sensorx2;

        double roll1 = sensory1;
        double roll2 = sensory2;

        double roll[2] = {roll1, roll2};
        double pitch[2] = {pitch1, pitch2};

        digger_angle = cal_digger_angle(roll, pitch, 2);

        // 快速显示传感器数据（不使用清屏，避免延迟）
        // 只显示传感器1的数据，因为只有一个传感器连接
        // X=Pitch(俯仰角，前仰为正), Y=Roll(横滚角，右翻为正), Z=Heading(航向角，0-360°)
        printf("\r[S1] X:%5.1f° Y:%5.1f° Z:%5.1f°   ", sensorx1, sensory1, sensorz1);
        fflush(stdout);  // 强制立即输出

        // 发布IMU数据（包含航向角）
        publish_imu_data(sensor1_pub, sen_num1, sensorx1, sensory1, sensorz1);
        publish_imu_data(sensor2_pub, sen_num2, sensorx2, sensory2, sensorz2);
        publish_imu_data(sensor3_pub, sen_num3, sensorx3, sensory3, sensorz3);

        // 发布角度数据
        std_msgs::Float64 angle_msg;
        angle_msg.data = digger_angle;
        angle_pub.publish(angle_msg);

        // 发布TF（只发布有数据的传感器）
        if (sensorx1 != 0 || sensory1 != 0)
            publish_sensor_tf(sen_num1, sensorx1, sensory1, sensorz1);
        if (sensorx2 != 0 || sensory2 != 0)
            publish_sensor_tf(sen_num2, sensorx2, sensory2, sensorz2);
        if (sensorx3 != 0 || sensory3 != 0)
            publish_sensor_tf(sen_num3, sensorx3, sensory3, sensorz3);

        ros::spinOnce();
        r.sleep();
    }

    // 清理
    delete tf_broadcaster;
}
