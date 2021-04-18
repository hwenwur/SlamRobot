#include <ros/ros.h>
#include "robot_serial.h"
#include "UWBPublisher.h"

// 2021/04/11 hwenwur
// 大疆 uwb 模块通信
// https://www.robomaster.com/zh-CN/products/components/detail/125

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_driver_node");
    ros::NodeHandle nh_private("~");

    UWBPublisher uwb(nh_private);

    ros::Rate rate(50);
    while (ros::ok())
    {
        uwb.publish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
