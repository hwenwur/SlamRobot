#include "VelocityReader.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

bool lookupLatestTrans(geometry_msgs::TransformStamped &dest, tf2_ros::Buffer &tf_buffer, const std::string &odom_frame, const std::string &base_frame)
{
    try
    {
        dest = tf_buffer.lookupTransform(odom_frame, base_frame, ros::Time(0));
        return true;
    }
    catch (const tf2::TransformException &e)
    {
        ROS_WARN_STREAM("lookupTransform Error: " << e.what() << '\n');
        return false;
    }
}

void publish_odom(ros::Publisher &odom_publisher, SerialFrameTimestamped &vel, geometry_msgs::TransformStamped &trans)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = trans.header.frame_id;
    odom.header.stamp = trans.header.stamp;

    odom.child_frame_id = trans.child_frame_id;

    geometry_msgs::Point &pos = odom.pose.pose.position;
    pos.x = trans.transform.translation.x;
    pos.y = trans.transform.translation.y;
    pos.z = trans.transform.translation.z;

    odom.pose.pose.orientation = trans.transform.rotation;

    odom.twist.twist.linear.x = vel.frame.vx;
    odom.twist.twist.linear.y = vel.frame.vy;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = vel.frame.omega;

    odom_publisher.publish(odom);
}

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
    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);

    robotserial::Serial mSerial(serial_port, baud_rate);
    VelocityReader reader(mSerial);
    reader.startReadLoop();

    SerialFrameTimestamped vel;
    geometry_msgs::TransformStamped trans;

    ros::Rate rate(frequent);
    bool r1, r2;
    while (nh.ok())
    {
        r1 = reader.lookupLatestFrame(vel);
        if (!r1)
            std::cerr << "Waiting for velocity data...\n";

        r2 = lookupLatestTrans(trans, tf_buffer, odom_frame, base_frame);
        if (!r2)
            std::cerr << "Waiting for transform data...\n";

        if (r1 && r2)
        {
            publish_odom(odom_publisher, vel, trans);
        }
        rate.sleep();
    }
    std::cerr << "Stop read loop...\n";
    reader.stopReadLoop();
    std::cerr << "Readloop stoped\n";
    return 0;
}
