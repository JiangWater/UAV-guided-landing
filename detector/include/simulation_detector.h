#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

#include "aruco_msgs/Detector.h"
#include "aruco_msgs/Detector_vel.h"
#include "mpc.h"
#include "math_utils.h"
#include "uav_msgs/feedback_delay.h"

class detectorNode
{
public:
    detectorNode();
    ~detectorNode()
    {
        // spinner->stop();
    }

private:
    ros::NodeHandle nh;

    ros::Publisher detector_cmd_pub;
    ros::Subscriber detector_sub;
    ros::Subscriber position_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber platform_odom_sub;
    ros::Subscriber uav_feedback_sub;

    ros::Publisher pub_cmd_to_delayNode;
    ros::Subscriber sub_cmd_from_delayNode;

    ros::Timer control_loop_timer;
    geometry_msgs::Vector3 rpy;
    aruco_msgs::Detector detect;
    aruco_msgs::Detector_vel detector_uav_cmd;
    uav_msgs::feedback_delay uav_feedback_msg;

    // MPC mpc;
    unique_ptr<MPC> mpc;

    map<string, double> mpc_params;
    double w_P_xy, w_P_z, w_P_xy_sum, w_P_z_sum, w_V_xy, w_V_z, w_q;
    double w_P_xy_n, w_P_z_n, w_P_xy_sum_n, w_P_z_sum_n, w_V_xy_n, w_V_z_n, w_q_n;
    double w_T, w_omega_x, w_omega_y, w_omega_z;
    double w_U_x, w_U_y, w_U_z;

    // 回调函数
    void control_loop_timer_cb(const ros::TimerEvent &event);
    void detector_cb(const aruco_msgs::Detector::ConstPtr &msg);
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void platform_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void uav_feedback_cb(const uav_msgs::feedback_delay::ConstPtr &msg);

    void NMPC_control();
    void LMPC_control();
    void PrintInfo();
};

//
void landing_strategy(int &flag, int &status, Eigen::Vector3d Pos_err, int &land_num1, int &land_num2, int land_time1, int land_time2);
double limit(double value, double lower_bound, double upper_bound);
double TimeDelay();

/*-------------------------------------
data_record = 0  不记录数据
data_record = 1  记录数据
---------------------------------------*/
int data_record = 0;
ofstream oFile;
int file_created = 0;

/*-------------------------------------
control_method = 0   Nonlinear Model Predictive Control
control_method = 1   Linear Model Predictive Control
---------------------------------------*/
int control_method;

bool marker_lost = false;
int land_flag = 1; // 降落标志位
int flag = 0, id = -1, num_all = 0, land = 0, status = 0, last_id = -1;
int land_num1 = 0, land_num2 = 0;
int land_time1, land_time2;

// UAV物理参数
double mass;             // 质量
double arm_length;       // 电机轴-飞控距离
double hover_throttle;   // 悬停油门
double max_tilt;         // 最大倾斜角
double max_accel_xy = 3; // 最大水平加速度
double max_accel_z = 1;  // 最大垂直加速度

// 时间
double current_time = 0.0;
double last_time = 0.0;
double start_time = 0.0;
double dt = 0.0;
double solve_time;

// 延迟相关
int delay_flag;
double est_delay = 0;
double last_est_delay = 0;
double feedback_delay;
double forward_delay;
double receive_timestamp;

// Arocu_ros 输出识别位置
Eigen::Vector3d Relative_pos(0.0, 0.0, 0.0);    // 当前相对位置
Eigen::Vector3d Relative_pos_ex(0.0, 0.0, 0.0); // 前一帧相对位置
Eigen::Vector3d Relative_pos_sum(0.0, 0.0, 0.0);
Eigen::Vector3d Relative_dpos(0.0, 0.0, 0.0);

Eigen::Vector3d Pos_target(0.0, 0.0, 0.0); // 目标位置
Eigen::Vector3d Pos_err(0.0, 0.0, 0.0);    // 位置误差

//====================模型预测控制Model Predictive Control=======================
int last_flag = 0;
int MPC_flag = 0;

int Np = 20;
int Nc = 20;

Eigen::Matrix<double, 13, 1> Nmpc_x0;
Eigen::Matrix<double, 13, 1> Nmpc_xRef;
Eigen::Matrix<double, 13, 1> Nmpc_predicted_x0;

Eigen::Matrix<double, 6, 1> Lmpc_x0;
Eigen::Matrix<double, 6, 1> Lmpc_xRef;
Eigen::Matrix<double, 6, 1> Lmpc_predicted_x0;

// Eigen::Vector3d vel_sp;
// Eigen::Vector3d force_sp;
Eigen::Vector3d accel_sp;
Eigen::Vector3d attitude_sp;
Eigen::Vector3d rate_sp;
Eigen::Vector3d last_rate_sp(0.0, 0.0, 0.0);
Eigen::Quaterniond q_sp;
Eigen::Vector3d Relative_accel_sp;
Eigen::Vector3d last_Relative_accel_sp(0.0, 0.0, 0.0);

double thrust_sp;
double last_thrust_sp = 9.8;
double thrust_sp_normalize;

// Eigen::Vector3d Relative_accel;
Eigen::Vector4d q_detect;
Eigen::Vector4d q_detect_ex;
tf::Quaternion quat_w;

Eigen::Vector4d q_ugv;
Eigen::Vector3d vel_ugv;
Eigen::Vector3d vel_ugv_ex;
Eigen::Vector3d accel_ugv;
Eigen::Vector3d accel_ugv_w;
Eigen::Vector3d omega_ugv;
Eigen::Vector3d omega_ugv_ex;
Eigen::Vector3d alpha_ugv;
Eigen::Vector3d Euler_ugv;

//=============================================================================

//---------------------------------------t265定位相关------------------------------------------
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d Euler_t265;
geometry_msgs::TransformStamped t265;
geometry_msgs::TransformStamped t265_last;

//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;
Eigen::Vector3d linear_vel_drone_fcu;
Eigen::Vector3d angular_vel_drone_fcu;

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu; // 无人机当前欧拉角(来自fcu)

//=============================================================================

double limit(double value, double lower_bound, double upper_bound)
{
    if (value < lower_bound)
        value = lower_bound;

    else if (value > upper_bound)
        value = upper_bound;

    return value;
}

template <typename T>
bool in_range(T value, T lower_bound, T upper_bound)
{
    if (value >= lower_bound && value <= upper_bound)
        return true;
    return false;
}

void detectorNode::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    q_fcu = q_fcu_enu;

    // Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}

void detectorNode::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Eigen::Vector3d linear_vel_drone_fcu_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    Eigen::Vector3d angular_vel_drone_fcu_enu(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    linear_vel_drone_fcu = linear_vel_drone_fcu_enu;
    angular_vel_drone_fcu = angular_vel_drone_fcu_enu;
}

//  x0(0, 1, 2)     x0(3, 4, 5)      x0(6, 7, 8)    x0(9, 10, 11, 12)
//   P(x, y, z)    P_sum(x, y, z)     V(x, y, z)    q(w, x, y, z)
Eigen::Matrix<double, 13, 1> Nmpc_one_step_predictor(const Eigen::Matrix<double, 13, 1> &x0, const double &dt, const Eigen::Vector4d &q_ugv, const Eigen::Vector3d &omega_ugv, const Eigen::Vector3d &alpha_ugv, const Eigen::Vector3d &accel_ugv_w, const double &T, const Eigen::Vector3d &body_rate)
{
    Eigen::Matrix<double, 13, 1> Nmpc_predicted_x0;
    Nmpc_predicted_x0 << x0(0) + x0(6) * dt,
        x0(1) + x0(7) * dt,
        x0(2) + x0(8) * dt,
        x0(3) + x0(0) * dt,
        x0(4) + x0(1) * dt,
        x0(5) + x0(2) * dt,
        x0(6) + ((omega_ugv(2) * omega_ugv(2) + omega_ugv(1) * omega_ugv(1)) * x0(0) + (-omega_ugv(0) * omega_ugv(1) + alpha_ugv(2)) * x0(1) + (-omega_ugv(0) * omega_ugv(2) - alpha_ugv(1)) * x0(2) + 2 * (omega_ugv(2) * x0(7) - omega_ugv(1) * x0(8)) + 2 * (x0(10) * x0(12) + x0(9) * x0(11)) * T - 2 * (q_ugv(1) * q_ugv(3) - q_ugv(0) * q_ugv(2)) * 9.8 - accel_ugv_w(0)) * dt,
        x0(7) + ((-omega_ugv(0) * omega_ugv(1) - alpha_ugv(2)) * x0(0) + (omega_ugv(0) * omega_ugv(0) + omega_ugv(2) * omega_ugv(2)) * x0(1) + (-omega_ugv(1) * omega_ugv(2) + alpha_ugv(0)) * x0(2) + 2 * (-omega_ugv(2) * x0(6) + omega_ugv(0) * x0(8)) + 2 * (x0(11) * x0(12) - x0(9) * x0(10)) * T - 2 * (q_ugv(2) * q_ugv(3) + q_ugv(0) * q_ugv(1)) * 9.8 - accel_ugv_w(1)) * dt,
        x0(8) + ((-omega_ugv(0) * omega_ugv(2) + alpha_ugv(1)) * x0(0) + (-omega_ugv(1) * omega_ugv(2) - alpha_ugv(0)) * x0(1) + (omega_ugv(0) * omega_ugv(0) + omega_ugv(1) * omega_ugv(1)) * x0(2) + 2 * (omega_ugv(1) * x0(6) - omega_ugv(0) * x0(7)) + (1 - 2 * x0(10) * x0(10) - 2 * x0(11) * x0(11)) * T - (1 - 2 * q_ugv(1) * q_ugv(1) - 2 * q_ugv(2) * q_ugv(2)) * 9.8 - accel_ugv_w(2)) * dt,
        x0(9) + (0.5 * (-body_rate(0) * x0(10) - body_rate(1) * x0(11) - body_rate(2) * x0(12) + omega_ugv(0) * x0(10) + omega_ugv(1) * x0(11) + omega_ugv(2) * x0(12))) * dt,
        x0(10) + (0.5 * (body_rate(0) * x0(9) + body_rate(2) * x0(11) - body_rate(1) * x0(12) - omega_ugv(0) * x0(9) + omega_ugv(2) * x0(11) - omega_ugv(1) * x0(12))) * dt,
        x0(11) + (0.5 * (body_rate(1) * x0(9) - body_rate(2) * x0(10) + body_rate(0) * x0(12) - omega_ugv(1) * x0(9) - omega_ugv(2) * x0(10) + omega_ugv(0) * x0(12))) * dt,
        x0(12) + (0.5 * (body_rate(2) * x0(9) + body_rate(1) * x0(10) - body_rate(0) * x0(11) - omega_ugv(2) * x0(9) + omega_ugv(1) * x0(10) - omega_ugv(0) * x0(11))) * dt;

    return Nmpc_predicted_x0;
}

Eigen::Matrix<double, 6, 1> Lmpc_one_step_predictor(const Eigen::Matrix<double, 6, 1> &x0, const double &dt, const Eigen::Vector3d &u)
{
    Eigen::Matrix<double, 6, 1> Lmpc_predicted_x0;
    Lmpc_predicted_x0 << x0(0) + x0(3) * dt,
        x0(1) + x0(4) * dt,
        x0(2) + x0(5) * dt,
        x0(3) + u(0) * dt,
        x0(4) + u(1) * dt,
        x0(5) + u(2) * dt;

    return Lmpc_predicted_x0;
}