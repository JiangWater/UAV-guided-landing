#include <ros/ros.h>
#include <iostream>
#include <random>

#include "aruco_msgs/Detector_vel.h"
#include "uav_msgs/feedback_delay.h"

using namespace std;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.1, 0.02);

aruco_msgs::Detector_vel forward_msg;
uav_msgs::feedback_delay feedback_msg;

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

void receive_feedback_msg_cb(const uav_msgs::feedback_delay::ConstPtr &msg)
{
    feedback_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delay_node");
    ros::NodeHandle nh_;

    ros::Subscriber forward_msg_from_ground = nh_.subscribe<aruco_msgs::Detector_vel>("/forward_channel", 50, receive_forward_msg_cb);
    ros::Publisher forward_msg_to_uav = nh_.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 50);
    ros::Subscriber feedback_msg_from_uav = nh_.subscribe<uav_msgs::feedback_delay>("/uav_feedback/delay", 50, receive_feedback_msg_cb);
    ros::Publisher feedback_msg_to_ground = nh_.advertise<uav_msgs::feedback_delay>("/feedback_channel", 50);

    while (ros::ok())
    {
        // ros::spinOnce();
        ros::spinOnce();
        double forward_delay = TimeDelay();
        double feedback_delay = TimeDelay();
        cout << "forward_delay: \033[1m\033[32m" << forward_delay << "\033[0m" << endl;
        cout << "feedback_delay: \033[1m\033[32m" << feedback_delay << "\033[0m" << endl;

        bool forward_delay_flag = false, feedback_delay_flag = false;
        double timer_start_time = ros::Time().now().toSec();

        while (!forward_delay_flag)
        {
            if (!forward_delay_flag && (ros::Time().now().toSec() - forward_msg.timestamp) >= forward_delay)
            {
                forward_msg_to_uav.publish(forward_msg);
                forward_delay_flag = true;

                cout << "forward pub time: " << ros::Time().now().toSec() << ", forward delay: " << ros::Time().now().toSec() - timer_start_time << endl;
            }

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