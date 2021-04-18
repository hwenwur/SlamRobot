#include "VelocityReader.h"

#include <string>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "WheelOdometry.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_driver_odom");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string odom_frame, base_frame, odom_topic;
    std::string serial_port;
    unsigned int baud_rate = 115200;
    int frequent;

    nh_private.getParam("odom_frame", odom_frame);
    nh_private.getParam("base_frame", base_frame);
    nh_private.getParam("odom_topic", odom_topic);
    nh_private.getParam("serial_port", serial_port);
    nh_private.getParam("freq", frequent);

    std::cerr << "1. odom_frame:" << std::quoted(odom_frame) << "\n"
              << "2. odom_topic:" << std::quoted(odom_topic) << "\n"
              << "3. base_frame:" << std::quoted(base_frame) << "\n"
              << "4. serial_port:" << std::quoted(serial_port) << "\n"
              << "5. frequent:" << frequent << "\n";

    robotserial::Serial mSerial(serial_port, baud_rate);
    VelocityReader reader(mSerial);
    reader.startReadLoop();

    SerialFrameTimestamped vel;

    WheelOdometry wodom(odom_topic, odom_frame, base_frame);

    ros::Rate rate(frequent);
    while (nh.ok())
    {
        if (!reader.lookupLatestFrame(&vel))
        {
            std::cerr << "Waiting for velocity data...\n";
            continue;
        }
        if (std::abs(vel.frame.vx) > 4 || std::abs(vel.frame.vy) > 4 || std::abs(vel.frame.omega) > 7)
        {
            // ignore invalid data.
            fprintf(stderr, "ignore invalid data(%.2f, %.2f, %.2f)\n", vel.frame.vx, vel.frame.vy, vel.frame.omega);
            continue;
        }
        // update pose and oritention from /odom frame, publish mav_msgs/Odometry and TF tree.
        wodom.update(vel);
        rate.sleep();
    }
    reader.stopReadLoop();
    return 0;
}
