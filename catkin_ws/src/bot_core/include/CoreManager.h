#ifndef CORE_MANAGER_H
#define CORE_MANAGER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace botcore
{
    // 为方便运算，使用齐次坐标，第三个值一直为 1
    typedef Eigen::Vector3d Point;

    // move_base 操作接口
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    // 路径规划状态
    enum class PlanStatus
    {
        READY,    // 当前没有目标
        PLANNING, // 正在向目标前进
        CANCELED, // 目标被取消
        REACHED,  // 已经到达目标
        FAILED,   // 无法到达目标
        NOT_RUN,  // 路径规划未启用
        UNDEFINED // 其他未定义状态
    };

    // 机器运动策略
    enum class RunningMode
    {
        NOT_START, // 比赛未开始
        NORMAL,    // 正常模式，云台跟随底盘
        FREE,      // 自由模式，云台自由移动
        FOCUS,     // 自瞄模式，云台对准敌方
        GAME_OVER, // 比赛结束
    };

    class CoreManager
    {
    public:
        CoreManager(const ros::NodeHandle &nh);
        // -------------------------------
        // 获取相关信息 函数
        // -------------------------------
        // 获取当前位置
        bool getCurrentPose(geometry_msgs::PoseStamped *dst);
        // 获取场地地图
        nav_msgs::OccupancyGrid &getOccupancyGridMap();
        // 获取当前路径规划状态
        PlanStatus getPlanStatus();
        // 获取当前路径规划目标
        bool getPlanTarget(Point *ret);
        // 获取机器运动策略
        const RunningMode &getRunningMode();
        // 检测视野中是否有敌方
        bool detectHorizonEnemy();

        // -------------------------------
        // 设置运动状态 函数
        // -------------------------------
        // 设置路径规划目标
        bool gotoTarget(const geometry_msgs::PoseStamped &p);
        // 取消正在执行的路径规划
        bool cancelCurrentPlan();
        // 设置运动策略
        bool setRunningMode(const RunningMode &m);

    private:
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        MoveBaseClient movebaseClient;
        RunningMode mode;
    };

} // namespace botcore
#endif // CORE_MANAGER_H
