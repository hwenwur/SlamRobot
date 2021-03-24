#include "WheelOdometry.h"

WheelOdometry::WheelOdometry(const std::string &odom_topic,
                             const std::string &frame_id,
                             const std::string &child_frame_id) : odom_topic(odom_topic),
                                                                  frame_id(frame_id),
                                                                  child_frame_id(child_frame_id),
                                                                  odometryTFBrodcaster(),
                                                                  publishTF(true)

{
    trans.setIdentity();
    lastUpdateTimestamp = 0;

    ros::NodeHandle nh;
    odometryPublisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);
}

void WheelOdometry::update(const SerialFrameTimestamped &vel)
{
    updateTransform(vel);
    publishOdometry(vel);
}

const tf2::Transform &WheelOdometry::updateTransform(const SerialFrameTimestamped &frame)
{
    if (lastUpdateTimestamp < 100)
    {
        lastUpdateTimestamp = frame.timestamp;
        return this->trans;
    }
    double deltaTime = (frame.timestamp - lastUpdateTimestamp) / 1000.0;
    lastUpdateTimestamp = frame.timestamp;
    if (deltaTime > 2)
    {
        // todo
        std::cerr << "Velocity update frequently too slow.\n";
    }
    auto motion = getTransformForMotion(frame.frame.vx, frame.frame.vy, frame.frame.omega, deltaTime);
    this->trans = this->trans * motion;
    // fprintf(stderr, "Motion(%lf)(%.2f, %.2f, %.2f),%.2lf\n",
    //         lastUpdateTimestamp / 1000.0,
    //         motion.getOrigin().x(),
    //         motion.getOrigin().y(),
    //         motion.getOrigin().z(),
    //         deltaTime);
    return this->trans;
}
void WheelOdometry::publishOdometry(const SerialFrameTimestamped &vel)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = ros::Time(vel.timestamp / 1000.0);

    odom.child_frame_id = child_frame_id;

    // --- 姿态 ---
    geometry_msgs::Point &pos = odom.pose.pose.position;
    pos.x = trans.getOrigin().x();
    pos.y = trans.getOrigin().y();
    pos.z = trans.getOrigin().z();

    odom.pose.pose.orientation.x = trans.getRotation().x();
    odom.pose.pose.orientation.y = trans.getRotation().y();
    odom.pose.pose.orientation.z = trans.getRotation().z();
    odom.pose.pose.orientation.w = trans.getRotation().w();

    // --- 速度 ----
    odom.twist.twist.linear.x = vel.frame.vx;
    odom.twist.twist.linear.y = vel.frame.vy;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = vel.frame.omega;

    odometryPublisher.publish(odom);

    if (publishTF)
    {
        publishOdomtryTF(vel);
    }
}
void WheelOdometry::publishOdomtryTF(const SerialFrameTimestamped &vel)
{
    geometry_msgs::TransformStamped GeoTrans;

    GeoTrans.header.frame_id = this->frame_id;
    GeoTrans.header.stamp = ros::Time(vel.timestamp / 1000.0);

    GeoTrans.child_frame_id = this->child_frame_id;

    GeoTrans.transform.translation.x = this->trans.getOrigin().x();
    GeoTrans.transform.translation.y = this->trans.getOrigin().y();
    GeoTrans.transform.translation.z = this->trans.getOrigin().z();

    GeoTrans.transform.rotation.x = this->trans.getRotation().x();
    GeoTrans.transform.rotation.y = this->trans.getRotation().y();
    GeoTrans.transform.rotation.z = this->trans.getRotation().z();
    GeoTrans.transform.rotation.w = this->trans.getRotation().w();

    odometryTFBrodcaster.sendTransform(GeoTrans);
}
/* 
        @duration: seconds
    */
tf2::Transform WheelOdometry::getTransformForMotion(double x_vel, double y_vel, double yaw_vel, double duration)
{
    tf2::Transform ret;
    ret.setIdentity();
    if (std::abs(yaw_vel) < 0.015)
    {
        // 视为 没有角速度
        ret.setOrigin(tf2::Vector3(x_vel * duration, y_vel * duration, 0));
    }
    else
    {
        double dist_x = x_vel * duration;
        double dist_y = y_vel * duration;
        double theta = yaw_vel * duration;
        tf2::Quaternion qTheta;
        qTheta.setRPY(0, 0, theta);
        qTheta = qTheta.normalize();
        if (std::abs(dist_x) < 0.01 && std::abs(dist_y) < 0.01)
        {
            // 视为 只有角速度
            ret.setRotation(qTheta);
        }
        else
        {
            // 既有线速度，也有角速度
            // x_vel, y_vel 指的是前方和左方的速度。
            // 把前方、左方的速度分解成 x 轴和 y 轴速度，然后积分即可得到以下式子。
            // double delta_theta = yaw_vel * duration;
            double x = (x_vel * std::sin(theta) + y_vel * std::cos(theta) - y_vel) / yaw_vel;
            double y = (x_vel - x_vel * cos(theta) + y_vel * std::sin(theta)) / yaw_vel;
            ret.setOrigin(tf2::Vector3(x, y, 0));
            ret.setRotation(qTheta);
        }
    }
    return ret;
}
