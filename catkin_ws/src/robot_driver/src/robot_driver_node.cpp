#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

void twistCallback(const geometry_msgs::Twist& msg)
{
    std::cout << "linear: " << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "\n"
              << "angular: " << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << "\n";
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1000, twistCallback);
    ros::spin();
    return 0;
}
