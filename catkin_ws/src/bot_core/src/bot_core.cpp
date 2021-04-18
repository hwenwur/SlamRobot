#include <iostream>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "CoreManager.h"

typedef struct
{
    double x, y;
    double qx, qy, qz, qw;
} Pose_t;

Pose_t pose_list[] = {
    {2.39540958405, 1.25604581833, 0, 0, -0.00577522351257, 0.999983323258},
    {5.32127761841, 1.08342242241, 0, 0, -0.432226251843, 0.901765195169},
    {11.0847682953, -2.61184549332, 0, 0, 0.661418400029, 0.750017133207},
    {10.5371780396, 0.993265330791, 0, 0, 0.903450086404, 0.428693295233},
    {12.8716602325, 3.06878900528, 0, 0, 0.691209208698, 0.722654709949},
    {14.7353687286, 0.946556091309, 0, 0, 0.908298669326, 0.41832227684},
    {12.2740592957, -0.74718016386, 0, 0, 0.943019257509, 0.332738155263},
};

void fillPose(geometry_msgs::PoseStamped *out, Pose_t &in)
{
    geometry_msgs::Pose &pose = out->pose;
    geometry_msgs::Point &p = pose.position;
    geometry_msgs::Quaternion &q = pose.orientation;
    out->header.frame_id = "map";
    out->header.stamp = ros::Time::now();
    p.x = in.x;
    p.y = in.y;
    p.z = 0;
    q.x = in.qx;
    q.y = in.qy;
    q.z = in.qz;
    q.w = in.qw;
}

void feedBack(const move_base_msgs::MoveBaseFeedbackConstPtr &p)
{
    const geometry_msgs::Pose &pose = p->base_position.pose;
    ROS_INFO("Current progress(%s): x: %.2lf, y:%.2lf",
             p->base_position.header.frame_id.c_str(),
             pose.position.x,
             pose.position.y);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bot_core_node");
    ros::NodeHandle nh;

    botcore::CoreManager cm(nh);
    ros::Rate rate(1);
    geometry_msgs::PoseStamped pose;

    botcore::MoveBaseClient client("move_base", true);

    while (!client.waitForServer(ros::Duration(0.5)))
    {
        ROS_INFO("waiting for move_base actionlib server to come up");
    }

    int goalIndex = 0;
    while (nh.ok())
    {
        auto state = cm.getPlanStatus();
        if (state == botcore::PlanStatus::PLANNING)
        {
            std::cerr << "PLANNING...";
            if (cm.getCurrentPose(&pose))
            {
                std::cerr << "Current Pose: " << pose.pose.position.x << ", " << pose.pose.position.y << "\n";
            }
            else
            {
                std::cerr << "Failed to get current pose\n";
            }
        }
        else
        {
            std::cerr << "Goto goal: " << goalIndex << "\n";
            geometry_msgs::PoseStamped goal;
            fillPose(&goal, pose_list[goalIndex]);
            cm.gotoTarget(goal);
            goalIndex++;
        }
        if (goalIndex == 6)
            goalIndex = 3;
        rate.sleep();
    }

    return 0;
}
