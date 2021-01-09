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
        serial->write(&frame, sizeof(SerialFrame));
        serial->write("\r\n", 2);
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
    robotserial::Serial mSerial("/dev/pts/3", B9600);
    mSerial.open();
    serial = &mSerial;
    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1000, twistCallback);
    ros::spin();
    mSerial.close();
    return 0;
}
