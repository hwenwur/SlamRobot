#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include "robot_serial.h"

typedef struct
{
    float vx;    // x 轴线速度
    float vy;    // y 轴线速度
    float omega; // z 轴角速度
} SerialFrame;

robotserial::Serial *serial;

void twistCallback(const geometry_msgs::Twist &msg)
{
    std::cerr << "linear: " << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "\n"
              << "angular: " << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << "\n";
    if (serial->isOpen())
    {
        SerialFrame frame = {(float)msg.linear.x, (float)msg.linear.y, (float)msg.angular.z};
        serial->writeAll(&frame, sizeof(SerialFrame));
        serial->writeAll("\r\n", 2);
    }
    else
    {
        std::cerr << "serial is not open\n";
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string serialPort;
    std::string topic;
    nh_private.getParam("serial_port", serialPort);
    nh_private.getParam("cmd_vel_topic", topic);

    robotserial::Serial mSerial(serialPort, B9600);
    mSerial.open();
    serial = &mSerial;
    ros::Subscriber sub = nh.subscribe(topic, 1000, twistCallback);
    ros::spin();
    mSerial.close();
    return 0;
}
