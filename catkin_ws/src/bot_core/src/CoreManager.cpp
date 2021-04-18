#include "CoreManager.h"

namespace botcore
{
    CoreManager::CoreManager(const ros::NodeHandle &nh) : tfBuffer(),
                                                          tfListener(tfBuffer),
                                                          movebaseClient("move_base", true)
    {
        // 连接 move_base 服务端
        while (!movebaseClient.waitForServer(ros::Duration(1)))
        {
            ROS_INFO("Waiting for move_base server come up...");
        }
        // nh.advertise<>
    }
    // Get current Location in world frame
    bool CoreManager::getCurrentPose(geometry_msgs::PoseStamped *dst)
    {
        geometry_msgs::PoseStamped v, out;
        // todo
        v.header.frame_id = "base_link";
        v.header.stamp = ros::Time::now();
        v.pose.position.x = 0;
        v.pose.position.y = 0;
        v.pose.position.z = 0;
        try
        {
            tfBuffer.transform(v, *dst, "world", ros::Duration(0.5));
            return true;
        }
        catch (const tf2::TransformException &e)
        {
            std::cerr << "Transform error: " << e.what() << "\n";
            return false;
        }
    }
    nav_msgs::OccupancyGrid &CoreManager::getOccupancyGridMap()
    {
        std::cerr << "Unimplement error!\n";
    }
    PlanStatus CoreManager::getPlanStatus()
    {
        PlanStatus ret = PlanStatus::UNDEFINED;
        if (!movebaseClient.isServerConnected())
        {
            ret = PlanStatus::NOT_RUN;
            ROS_WARN("move_base not ready...");
            return ret;
        }
        // http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html
        auto state = movebaseClient.getState();
        ROS_DEBUG_STREAM("getPlanStatus: " << state.toString() << state.getText());
        switch (state.state_)
        {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            ret = PlanStatus::REACHED;
            break;
        case actionlib::SimpleClientGoalState::PENDING:
        case actionlib::SimpleClientGoalState::ACTIVE:
            ret = PlanStatus::PLANNING;
            break;
        case actionlib::SimpleClientGoalState::PREEMPTED:
        case actionlib::SimpleClientGoalState::RECALLED:
            ret = PlanStatus::CANCELED;
            break;
        case actionlib::SimpleClientGoalState::ABORTED:
        case actionlib::SimpleClientGoalState::REJECTED:
            ret = PlanStatus::FAILED;
            break;
        case actionlib::SimpleClientGoalState::LOST:
            ret = PlanStatus::READY;
            break;
        default:
            ROS_ERROR_STREAM("Unexpected plan state: " << state.toString() << state.getText());
            ret = PlanStatus::UNDEFINED;
            break;
        }
        return ret;
    }
    bool CoreManager::getPlanTarget(Point *ret)
    {
        std::cerr << "Unimplement error!\n";
    }
    const RunningMode &CoreManager::getRunningMode()
    {
        return this->mode;
    }
    bool detectHorizonEnemy()
    {
        std::cerr << "Unimplement error!\n";
        return false;
    }

    bool CoreManager::gotoTarget(const geometry_msgs::PoseStamped &p)
    {
        if (movebaseClient.isServerConnected())
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = p;
            movebaseClient.sendGoal(goal);
            ROS_INFO("gotoTarget: %.2lf, %.2lf", p.pose.position.x, p.pose.position.y);
            return true;
        }
        else
        {
            ROS_WARN("move_base not ready...");
            return false;
        }
    }
    bool CoreManager::cancelCurrentPlan()
    {
        movebaseClient.cancelGoal();
        return true;
    }
    bool CoreManager::setRunningMode(const RunningMode &m)
    {
        this->mode = m;
        return true;
    }
};
