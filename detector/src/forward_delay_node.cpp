#include <ros/ros.h>
#include <iostream>
#include <random>

#include "aruco_msgs/Detector_vel.h"
#include "uav_msgs/feedback_delay.h"

using namespace std;

std::default_random_engine generator;
 std::normal_distribution<double> distribution(0.04, 0.002);
//std::normal_distribution<double> distribution(0.04, 0.0);

aruco_msgs::Detector_vel forward_msg;
uav_msgs::feedback_delay feedback_msg;

double limit(double value, double lower_bound, double upper_bound)
{
    if (value < lower_bound)
        value = lower_bound;

    else if (value > upper_bound)
        value = upper_bound;

    return value;
}

double TimeDelay()
{
    double delay = distribution(generator);
    // return 0;
    return delay;
    // return limit(delay, 0.1, 0.2);
}

void receive_forward_msg_cb(const aruco_msgs::Detector_vel::ConstPtr &msg)
{
    forward_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_delay_node");
    ros::NodeHandle nh_;

    ros::Subscriber forward_msg_from_ground = nh_.subscribe<aruco_msgs::Detector_vel>("/forward_channel", 50, receive_forward_msg_cb);
    ros::Publisher forward_msg_to_uav = nh_.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 50);

    while (ros::ok())
    {
        // ros::spinOnce();
        ros::spinOnce();
        double forward_delay = TimeDelay();
        forward_delay = limit(forward_delay, 0.038, 0.042);
        cout << "forward_delay: \033[1m\033[32m" << forward_delay << "\033[0m" << endl;

        bool forward_delay_flag = false;
        double timer_start_time = ros::Time().now().toSec();

        while (!forward_delay_flag)
        {
            if (!forward_delay_flag && (ros::Time().now().toSec() - forward_msg.timestamp) >= forward_delay)
            {
                forward_msg_to_uav.publish(forward_msg);
                forward_delay_flag = true;

                cout << "forward pub time: " << ros::Time().now().toSec() << ", forward delay: " << ros::Time().now().toSec() - timer_start_time << endl;
            }
        }
        cout << endl;
    }

    // ros::spin();
    return 0;
}