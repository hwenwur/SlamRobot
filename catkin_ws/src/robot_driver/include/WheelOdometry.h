#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include "SerialFrame.h"

#include <string>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

class WheelOdometry
{
public:
    WheelOdometry(const std::string &odom_topic,
                  const std::string &frame_id,
                  const std::string &child_frame_id);
    void update(const SerialFrameTimestamped &vel);

private:
    const tf2::Transform &updateTransform(const SerialFrameTimestamped &frame);
    void publishOdometry(const SerialFrameTimestamped &vel);

    void publishOdomtryTF(const SerialFrameTimestamped &vel);

    static tf2::Transform getTransformForMotion(double x_vel, double y_vel, double yaw_vel, double duration);

private:
    tf2::Transform trans;
    unsigned long lastUpdateTimestamp;

    const std::string &frame_id;
    const std::string &child_frame_id;
    const std::string &odom_topic;
    ros::Publisher odometryPublisher;

    tf2_ros::TransformBroadcaster odometryTFBrodcaster;

    bool publishTF;
};

#endif
