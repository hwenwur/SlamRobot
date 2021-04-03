#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 发布测试数据 unadjust_map_frame -> base_link
// 每个一段时间偏移到对称位置
// 检测 location_corrector.cpp 是否能够纠正这些错误
// location_correcor.cpp 通过发布 map_frame -> unadjust_map_frame
// 变换来抵消上述误差。
double cx = -0.3321249485;
double cy = -1.075595140455;
void toSymmetry(geometry_msgs::TransformStamped &trans, double centeral_x, double centeral_y)
{

    double tmp;
    tmp = trans.transform.translation.x;
    trans.transform.translation.x = 2 * centeral_x - tmp;
    tmp = trans.transform.translation.y;
    trans.transform.translation.y = 2 * centeral_y - tmp;

    tf2::Matrix3x3 m(tf2::Quaternion(trans.transform.rotation.x,
                                     trans.transform.rotation.y,
                                     trans.transform.rotation.z,
                                     trans.transform.rotation.w));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw + M_PI);

    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "location_corrector_test");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string unadjust_map_frame, child_frame;
    double centeral_x, centeral_y;
    nh_private.getParam("unadjust_map_frame", unadjust_map_frame);
    nh_private.getParam("child_frame", child_frame);
    nh_private.getParam("centeral_x", centeral_x);
    nh_private.getParam("centeral_y", centeral_y);

    int frequency = 200;
    ros::Rate rate(frequency);
    unsigned long tick = 0;
    double x, y, yaw;
    double t;
    tf2_ros::TransformBroadcaster broadcaster;
    while (nh.ok())
    {
        t = (double)tick / frequency;
        x = 0.2 * t;
        y = 2 * sin(0.4 * M_PI * t);
        yaw = atan(2 * 0.4 * M_PI * cos(0.4 * M_PI * t));

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        geometry_msgs::TransformStamped trans;
        trans.header.frame_id = unadjust_map_frame;
        trans.header.stamp = ros::Time::now();
        trans.child_frame_id = child_frame;

        trans.transform.translation.x = x;
        trans.transform.translation.y = y;
        trans.transform.translation.z = 0;

        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        if (((int)t / 10) & 1 == 1)
        {
            toSymmetry(trans, centeral_x, centeral_y);
        }
        broadcaster.sendTransform(trans);
        tick++;
        rate.sleep();
    }
    return 0;
}