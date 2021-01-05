#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include "robot_serial.h"

robotserial::Serial *serial;

void twistCallback(const geometry_msgs::Twist &msg)
{
    std::cerr << "linear: " << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "\n"
              << "angular: " << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << "\n";
    if (serial->isOpen())
    {
        // todo
        std::stringstream ss;
        ss << "linear.x: " << msg.linear.x << ", angular.z: " << msg.angular.z << ";\n";
        std::string data(ss.str());
        serial->write(data.c_str(), data.length());
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
    robotserial::Serial mSerial("/dev/pts/0", B9600);
    serial = &mSerial;
    serial->open();
    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1000, twistCallback);
    ros::spin();
    mSerial.close();
    return 0;
}
