#include "ros_force_move.h"

/* 
    Gazebo ROS 驱动插件，根据 cmd_vel 对模型施加力和扭矩。

    本文件会被编译成动态库，所以没有 main 函数。

    官方示例：https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_planar_move.cpp
 */

namespace gazebo
{
    GazeboRosForceDriver::GazeboRosForceDriver()
    {
        this->xVelMax = 5;
        this->yVelMax = 5;
        this->yawVelMax = 6.28;

        this->xForceAcclerate = 15;
        this->yForceAcclerate = 15;
        this->ztorqueAcclerate = 1;
    }
    GazeboRosForceDriver::~GazeboRosForceDriver()
    {
        this->onDestoryChild();
    }
    void GazeboRosForceDriver::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        ROS_INFO("GazeboRosForceDriver loaded!");
        initParams(sdf);
        this->parent = parent;
        this->link = parent->GetLink(baseLinkName);
        this->lastOdometryPubTime = parent->GetWorld()->SimTime();
        this->lastOdometryPose = parent->WorldPose();

        this->xVel = 0;
        this->yVel = 0;
        this->yawVel = 0;
        this->alive = true;

        // 设置 x,y,z,r,p,y = 0
        this->odometryTransform.setIdentity();
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("ROS not initialized.\n");
            return;
        }
        this->rosNodeHandle.reset(new ros::NodeHandle(this->robotNamespace));
        if (this->publishOdometryTF)
        {
            this->transformBroadcaster.reset(new tf::TransformBroadcaster());
        }

        auto so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
            this->cmdVelTopic,
            1,
            std::bind(&GazeboRosForceDriver::cmdVelCallback, this, std::placeholders::_1),
            ros::VoidPtr(),
            &callbackQueue);
        this->velocitySub = rosNodeHandle->subscribe(so);
        this->odometryPub = rosNodeHandle->advertise<nav_msgs::Odometry>(this->odometryTopic, 1);

        this->queueThread.reset(new std::thread(&GazeboRosForceDriver::rosQueueThread, this));

        this->updateConn = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosForceDriver::onUpdateChild, this));
    }

    void GazeboRosForceDriver::onUpdateChild()
    {
        std::lock_guard<std::mutex> lock_guard(this->velLock);
        auto pose = this->parent->WorldPose();
        auto angular_vel = this->parent->WorldAngularVel();
        auto linear_vel = this->parent->RelativeLinearVel();

        double z_error, x_error, y_error;
        z_error = (yawVel > 0 ? (std::min(yawVel, yawVelMax)) : (std::max(yawVel, -yawVelMax))) - angular_vel.Z();
        this->link->AddTorque(ignition::math::Vector3d(0, 0, z_error * ztorqueAcclerate));

        x_error = (xVel > 0 ? (std::min(xVel, xVelMax)) : (std::max(xVel, -xVelMax))) - linear_vel.X();
        y_error = (yVel > 0 ? (std::min(yVel, yVelMax)) : (std::max(yVel, -yVelMax))) - linear_vel.Y();

        this->link->AddRelativeForce(ignition::math::Vector3d(
            x_error * xForceAcclerate,
            y_error * yForceAcclerate,
            0));

        // ROS_INFO("Current Speed: %5.2f %5.2f %5.2f\nError: %5.2f %5.2f %5.2f\n", linear_vel.X(), linear_vel.Y(), angular_vel.Z(),
        //  x_error, y_error, z_error);
        // 发布 odom
        auto current_time = this->parent->GetWorld()->SimTime();
        // 单位：秒
        double delta = (current_time - lastOdometryPubTime).Double();
        if (delta > (1 / odometryRate))
        {
            publishOdometry(delta);
            lastOdometryPubTime = current_time;
        }
    }

    void GazeboRosForceDriver::onDestoryChild()
    {
        alive = false;
        callbackQueue.clear();
        callbackQueue.disable();
        rosNodeHandle->shutdown();
        queueThread->join();
    }

    void GazeboRosForceDriver::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock_guard(velLock);
        xVel = msg->linear.x;
        yVel = msg->linear.y;
        yawVel = msg->angular.z;
    }

    void GazeboRosForceDriver::rosQueueThread()
    {
        while (this->alive && this->rosNodeHandle->ok())
        {
            this->callbackQueue.callAvailable(ros::WallDuration(0.01));
        }
    }

    /* 
        发布里程计数据
        @delta_time: 单位：秒
     */
    void GazeboRosForceDriver::publishOdometry(double detla_time)
    {
        auto linear_vel = this->parent->RelativeLinearVel();
        auto angular_vel = this->parent->RelativeAngularVel();
        auto current_time = ros::Time::now();
        nav_msgs::Odometry tmp;
        odometryTransform = odometryTransform * this->getTransformForMotion(
                                                    linear_vel.X(),
                                                    linear_vel.Y(),
                                                    angular_vel.Z(),
                                                    detla_time);
        tf::poseTFToMsg(odometryTransform, tmp.pose.pose);
        tmp.twist.twist.linear.x = linear_vel.X();
        tmp.twist.twist.linear.y = linear_vel.Y();
        tmp.twist.twist.angular.z = angular_vel.Z();

        tmp.header.stamp = current_time;
        tmp.header.frame_id = this->odometryFrameId;
        tmp.child_frame_id = this->baseLinkName;

        if (this->publishOdometryTF && this->transformBroadcaster.get())
        {
            this->transformBroadcaster->sendTransform(
                tf::StampedTransform(odometryTransform, current_time, odometryFrameId, baseLinkName));
        }
        this->odometryPub.publish(tmp);
    }

    tf::Transform GazeboRosForceDriver::getTransformForMotion(double x_vel, double y_vel, double yaw_vel, double duration)
    {
        tf::Transform ret;
        ret.setIdentity();
        if (std::abs(yaw_vel) < 0.001)
        {
            // 视为 没有角速度
            ret.setOrigin(tf::Vector3(x_vel * duration, y_vel * duration, 0));
        }
        else
        {
            double dist_x = x_vel * duration;
            double dist_y = y_vel * duration;
            double theta = yaw_vel * duration;
            if (std::abs(dist_x) < 0.01 && std::abs(dist_y) < 0.02)
            {
                // 视为 只有角速度
                ret.setRotation(tf::createQuaternionFromYaw(theta));
            }
            else
            {
                // 既有线速度，也有角速度
                // x_vel, y_vel 指的是前方和左方的速度。
                // 把前方、左方的速度分解成 x 轴和 y 轴速度，然后积分即可得到以下式子。
                // double delta_theta = yaw_vel * duration;
                double x = (x_vel * std::sin(theta) + y_vel * std::cos(theta) - y_vel) / yaw_vel;
                double y = (x_vel - x_vel * cos(theta) + y_vel * std::sin(theta)) / yaw_vel;
                ret.setOrigin(tf::Vector3(x, y, 0));
                ret.setRotation(tf::createQuaternionFromYaw(theta));
            }
        }
        return ret;
    }

    void GazeboRosForceDriver::initParams(const sdf::ElementPtr &sdf)
    {
        robotNamespace = "";
        if (sdf->HasElement("robotNamespace"))
        {
            robotNamespace = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        ROS_INFO_STREAM("robotNamespace: " << robotNamespace);

        odometryTopic = "odom";
        if (sdf->HasElement("odometryTopic"))
        {
            odometryTopic = sdf->GetElement("odometryTopic")->Get<std::string>();
        }
        ROS_INFO_STREAM("odometryTopic: " << odometryTopic);

        odometryRate = 10;
        if (sdf->HasElement("odometryRate"))
        {
            odometryRate = sdf->GetElement("odometryRate")->Get<double>();
        }
        ROS_INFO_STREAM("odometryRate: " << odometryRate);

        odometryFrameId = "odom";
        if (sdf->HasElement("odometryFrameId"))
        {
            odometryFrameId = sdf->GetElement("odometryFrameId")->Get<std::string>();
        }
        ROS_INFO_STREAM("odometryFrameId: " << odometryFrameId);

        publishOdometryTF = true;
        if (sdf->HasElement("publishOdometryTF"))
        {
            publishOdometryTF = sdf->GetElement("publishOdometryTF")->Get<bool>();
        }
        ROS_INFO_STREAM("publishOdometryTF: " << publishOdometryTF);

        cmdVelTopic = "cmd_vel";
        if (sdf->HasElement("cmdVelTopic"))
        {
            cmdVelTopic = sdf->GetElement("cmdVelTopic")->Get<std::string>();
        }
        ROS_INFO_STREAM("cmdVelTopic: " << cmdVelTopic);

        baseLinkName = "base_link";
        if (sdf->HasElement("baseLinkName"))
        {
            baseLinkName = sdf->GetElement("baseLinkName")->Get<std::string>();
        }
        ROS_INFO_STREAM("baseLinkName: " << baseLinkName);
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceDriver)
} // namespace gazebo