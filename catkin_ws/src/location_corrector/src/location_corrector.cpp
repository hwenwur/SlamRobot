#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// 由于赛场是中心对称的，基于点云的定位算法有一定概率飘到对称点去。
// 本程序用于校正上述错误。
// hwenwur 2021/03/29

class LocationCorrector
{
public:
    LocationCorrector(const std::string &map_frame,
                      const std::string &unadjust_map_frame,
                      const std::string &child_frame,
                      const double &centeral_x,
                      const double &centeral_y) : map_frame(map_frame),
                                                  unadjust_map_frame(unadjust_map_frame),
                                                  child_frame(child_frame),
                                                  centeral_x(centeral_x),
                                                  centeral_y(centeral_y),
                                                  adjust(false)
    {
    }
    void spin(const geometry_msgs::TransformStamped &trans)
    {
        current_x = trans.transform.translation.x;
        current_y = trans.transform.translation.y;
        delta_x = std::abs(current_x - last_x);
        delta_y = std::abs(current_y - last_y);
        if (delta_y + delta_x > 0.3)
        {
            // 位置坐标发生突变
            // 可能是正常定位调整，也可能是飘到对称点去了
            // 先检测突变前后的坐标是不是关于中心点对称
            error = std::abs(current_x + last_x - 2 * centeral_x) +
                    std::abs(current_y + last_y - 2 * centeral_y);
            if (error < (delta_x + delta_y))
            {
                ROS_INFO("Adjust: (%.2lf, %.2lf) <- (%.2f, %.2f), Error: %.2f\n", current_x, current_y, last_x, last_y, error);
                adjust = !adjust;
            }
        }
        publishTransform(trans);
        last_x = current_x;
        last_y = current_y;
    }
    void publishTransform(const geometry_msgs::TransformStamped &trans)
    {
        // 发布 map -> unadjust_map_frame
        // adjust = false; transform 不作用
        // adjust = true; transform 为原来中心对称位置
        geometry_msgs::TransformStamped newTrans;
        newTrans.header.frame_id = map_frame;
        newTrans.header.stamp = trans.header.stamp;

        newTrans.child_frame_id = unadjust_map_frame;

        tf2::Quaternion q;
        if (adjust)
        {
            q.setRPY(0, 0, M_PI);
            newTrans.transform.translation.x = 2 * centeral_x;
            newTrans.transform.translation.y = 2 * centeral_y;
        }
        else
        {
            q.setRPY(0, 0, 0);
            newTrans.transform.translation.x = 0;
            newTrans.transform.translation.y = 0;
        }

        newTrans.transform.translation.z = 0;

        newTrans.transform.rotation.x = q.x();
        newTrans.transform.rotation.y = q.y();
        newTrans.transform.rotation.z = q.z();
        newTrans.transform.rotation.w = q.w();
        // map_frame --> unadjust_map_frame
        broadcaster.sendTransform(newTrans);
    }

private:
    const std::string &map_frame;
    const std::string &unadjust_map_frame;
    const std::string &child_frame;
    const double &centeral_x;
    const double &centeral_y;

    tf2_ros::TransformBroadcaster broadcaster;

    bool adjust;

    // spin 函数调用频率很高，为减少资源消耗，部分局部变量改为全局变量
    double last_x;
    double last_y;
    double current_x;
    double current_y;
    double delta_x;
    double delta_y;
    double error;
};

bool getTransform(geometry_msgs::TransformStamped *trans,
                  const tf2_ros::Buffer &tfBuffer,
                  const std::string &map_frame,
                  const std::string &child_frame)
{
    try
    {
        *trans = tfBuffer.lookupTransform(map_frame, child_frame, ros::Time(0));
        return true;
    }
    catch (tf2::TransformException &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "location_corrector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string unadjust_map_frame, child_frame;
    std::string map_frame;
    double centeral_x = 0, centeral_y = 0;

    nh_private.getParam("unadjust_map_frame", unadjust_map_frame);
    nh_private.getParam("child_frame", child_frame);
    nh_private.getParam("map_frame", map_frame);
    nh_private.getParam("centeral_x", centeral_x);
    nh_private.getParam("centeral_y", centeral_y);

    LocationCorrector corrector(map_frame, unadjust_map_frame, child_frame, centeral_x, centeral_y);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    geometry_msgs::TransformStamped trans;

    ros::Rate rate(200);
    ROS_INFO("Start correcting...");
    while (nh.ok())
    {
        if (getTransform(&trans, tfBuffer, unadjust_map_frame, child_frame))
        {
            corrector.spin(trans);
        }
        rate.sleep();
    }
    return 0;
}
