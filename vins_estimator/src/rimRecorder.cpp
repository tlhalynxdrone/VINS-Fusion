#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// Function to read a single key press without waiting for Enter
char getKey()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcgetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

class RimRecorder
{
public:
    RimRecorder(ros::NodeHandle &nh)
    {
        odom_sub_ = nh.subscribe("/vins_estimator/odometry", 1000, &RimRecorder::odomCallback, this);
        recording_active_ = true;
        ROS_INFO("RimRecorder started. Press Enter to record a point, Spacebar to finish.");
    }

    void spin()
    {
        while (recording_active_ && ros::ok())
        {
            ros::spinOnce();
            char key = getKey();
            if (key == '\n' || key == '\r')  // Enter key
            {
                if (last_position_set_)
                {
                    flag_points_.push_back(last_position_);
                    ROS_INFO_STREAM("Recorded point: [" << last_position_.transpose() << "]");
                }
                else
                {
                    ROS_WARN("No odometry received yet!");
                }
            }
            else if (key == ' ')  // Spacebar
            {
                recording_active_ = false;
                ROS_INFO("Finishing recording...");
                printResults();
            }
        }
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        last_position_ = Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
        last_position_set_ = true;
    }

    void printResults()
    {
        if (flag_points_.empty())
        {
            ROS_WARN("No points recorded.");
            return;
        }

        ROS_INFO("Flagged points and distances:");

        double total_distance = 0.0;
        for (size_t i = 0; i < flag_points_.size(); i++)
        {
            ROS_INFO_STREAM("Point " << i << ": [" << flag_points_[i].transpose() << "]");
            if (i > 0)
            {
                double dist = (flag_points_[i] - flag_points_[i - 1]).norm();
                total_distance += dist;
                ROS_INFO_STREAM("Distance from previous: " << dist << " m");
            }
        }
        ROS_INFO_STREAM("Total distance traveled between flags: " << total_distance << " m");
    }

    ros::Subscriber odom_sub_;
    Vector3d last_position_;
    bool last_position_set_ = false;
    bool recording_active_;

    vector<Vector3d> flag_points_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_flag_recorder");
    ros::NodeHandle nh;

    RimRecorder recorder(nh);
    recorder.spin();

    return 0;
}
