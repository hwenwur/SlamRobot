#ifndef ROS_FORCE_DRIVER_H_
#define ROS_FORCE_DRIVER_H_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <memory>
#include <thread>
#include <string>
#include <mutex>

namespace gazebo
{

    class GazeboRosForceDriver : public ModelPlugin
    {
    public:
        GazeboRosForceDriver();
        ~GazeboRosForceDriver();
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

    private:
        void initParams(const sdf::ElementPtr &sdf);
        void onUpdateChild();
        void onDestoryChild();
        void publishOdometry(double detla_time);
        tf::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const;

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void rosQueueThread();

        tf::Transform getTransformForMotion(double x_vel, double y_vel, double yaw_vel, double duration);

    private:
        physics::ModelPtr parent;
        event::ConnectionPtr updateConn;
        physics::LinkPtr link;
        common::Time lastOdometryPubTime;
        ros::NodeHandlePtr rosNodeHandle;
        tf::Transform odometryTransform;
        ignition::math::Pose3d lastOdometryPose;
        std::shared_ptr<tf::TransformBroadcaster> transformBroadcaster;
        ros::CallbackQueue callbackQueue;
        std::shared_ptr<std::thread> queueThread;
        ros::Subscriber velocitySub;
        ros::Publisher odometryPub;
        bool alive;

        double xVelMax;
        double yVelMax;
        double yawVelMax; // z 轴转速

        double xForceAcclerate;
        double yForceAcclerate;
        double ztorqueAcclerate;

        std::mutex velLock;
        double xVel;
        double yVel;
        double yawVel;

        /* 
            XML 参数
         */
        std::string robotNamespace;
        std::string odometryTopic;
        double odometryRate;
        std::string odometryFrameId;
        bool publishOdometryTF;
        std::string cmdVelTopic;
        std::string baseLinkName;
    };
} // namespace gazebo

#endif