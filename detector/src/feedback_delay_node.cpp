#include <ros/ros.h>
#include <iostream>
#include <random>

#include "aruco_msgs/Detector_vel.h"
#include "uav_msgs/feedback_delay.h"

using namespace std;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.04, 0.002);
// std::normal_distribution<double> distribution(0.04, 0.0);

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

void receive_feedback_msg_cb(const uav_msgs::feedback_delay::ConstPtr &msg)
{
    feedback_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feedback_delay_node");
    ros::NodeHandle nh_;

    ros::Subscriber feedback_msg_from_uav = nh_.subscribe<uav_msgs::feedback_delay>("/uav_feedback/delay", 50, receive_feedback_msg_cb);
    ros::Publisher feedback_msg_to_ground = nh_.advertise<uav_msgs::feedback_delay>("/feedback_channel", 50);

    while (ros::ok())
    {
        // ros::spinOnce();
        ros::spinOnce();

        double feedback_delay = TimeDelay();
        feedback_delay = limit(feedback_delay, 0.038, 0.042);
        cout << "feedback_delay: \033[1m\033[32m" << feedback_delay << "\033[0m" << endl;

        bool feedback_delay_flag = false;
        double timer_start_time = ros::Time().now().toSec();

        while (!feedback_delay_flag)
        {
            // cout << "feedback_msg.timestamp: " << feedback_msg.timestamp << endl;
            if (!feedback_delay_flag && (ros::Time().now().toSec() - feedback_msg.timestamp) >= feedback_delay)
            {
                feedback_msg_to_ground.publish(feedback_msg);
                feedback_delay_flag = true;

                cout << "feedback pub time: " << ros::Time().now().toSec() << ", feedback delay: " << ros::Time().now().toSec() - timer_start_time << endl;
            }
        }
        cout << endl;
    }

    // ros::spin();
    return 0;
}