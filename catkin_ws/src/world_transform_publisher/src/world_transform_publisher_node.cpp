#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "world_transform_publisher_node");
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster pub;
    geometry_msgs::TransformStamped trans;
    double x = 0, y = 0, yaw = 0;

    nh.getParam("world_tf_dx", x);
    nh.getParam("world_tf_dy", y);
    nh.getParam("world_tf_degree", yaw);

    std::cerr << "x: " << x << ", y: " << y << ", deg: " << yaw << "\n";
    trans.header.frame_id = "world";
    trans.header.stamp = ros::Time::now();
    trans.child_frame_id = "map";
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();

    pub.sendTransform(trans);
    ros::spin();
    return 0;
}
