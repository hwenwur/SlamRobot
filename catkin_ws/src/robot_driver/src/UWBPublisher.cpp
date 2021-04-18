#include "UWBPublisher.h"

UWBPublisher::UWBPublisher(ros::NodeHandle nh)
{
    nh.param<std::string>("serial_port", serial_port, "/dev/uwb");
    nh.param<std::string>("frame_id", frame_id, "uwb");
    nh.param<std::string>("global_frame", global_frame, "world");
    nh.param<double>("south_angle", south_angle, 0);

    ROS_INFO_STREAM("params:\n\n"
                    << "serial_port: " << serial_port
                    << "\nframe_id: " << frame_id
                    << "\nglobal_frame: " << global_frame
                    << "\nsouth_angle: " << south_angle);

    // todo
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/uwb_pose", 100);

    serial = std::make_unique<robotserial::Serial>(serial_port, 115200);

    if (serial->open() != 0)
    {
        ROS_ERROR("can not open serial(%d): %s", (int)serial->getStatus(), serial_port.c_str());
    }
}

UWBPublisher::~UWBPublisher()
{
    if (serial->isOpen())
        serial->close();
}

bool UWBPublisher::readPocket(UWBPocket *out)
{
    static unsigned short buff[11];
    if (serial->availableBytes() < sizeof(buff))
    {
        return false;
    }
    if (serial->read(&buff, sizeof(buff)) >= sizeof(buff))
    {
        short x = (signed short)buff[0];
        short y = (signed short)buff[1];
        unsigned short yaw = buff[2];
        unsigned short error = buff[9] & (0b1111);
        unsigned short si = buff[9] >> 14;

        // convert to metric
        out->x = x / 100.0;
        out->y = y / 100.0;
        // convert to radian, range: [0 - 2pi]
        out->yaw = yaw / 3600.0 * 2 * M_PI;
        out->error = errno;
        out->si = si;
        return true;
    }
    else
    {
        // todo
        return false;
    }
}

void UWBPublisher::publish()
{
    UWBPocket p;
    if (readPocket(&p))
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame;
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, p.yaw + south_angle);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose_pub.publish(pose);
    }
    else
    {
        ROS_WARN("Failed to read UWB message");
    }
}