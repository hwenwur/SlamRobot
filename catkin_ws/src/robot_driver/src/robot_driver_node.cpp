#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include "robot_serial.h"
#include "SerialFrame.h"

robotserial::Serial *serial;

void twistCallback(const geometry_msgs::Twist &msg)
{
    std::cerr << "velocity: (" << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "), ("
              << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << ")\n";
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
    unsigned int baudRate = 115200;
    nh_private.getParam("serial_port", serialPort);
    nh_private.getParam("cmd_vel_topic", topic);

    robotserial::Serial mSerial(serialPort, baudRate);
    mSerial.open();
    serial = &mSerial;
    ros::Subscriber sub = nh.subscribe(topic, 1000, twistCallback);
    ros::spin();
    mSerial.close();
    return 0;
}
