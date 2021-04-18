#ifndef UWBREADER_H
#define UWBREADER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "robot_serial.h"

typedef struct
{
    double x;   // m
    double y;   // m
    double yaw; // radian
    int error;  //
    int si;     // signal intensity
} UWBPocket;

class UWBPublisher
{
public:
    UWBPublisher(ros::NodeHandle nh);
    ~UWBPublisher();
    void publish();

private:
    bool readPocket(UWBPocket *out);

private:
    ros::Publisher pose_pub;
    std::unique_ptr<robotserial::Serial> serial;
    std::string serial_port;
    std::string frame_id;
    std::string global_frame;
    double south_angle;
};

#endif