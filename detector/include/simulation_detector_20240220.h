#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <queue>

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
#include "aruco_msgs/Control_sequence.h"
#include "mpc.h"
#include "Fuzzy_PID.h"
#include "math_utils.h"

class detectorNode
{
public:
    detectorNode();

private:
    ros::NodeHandle nh;

    ros::Publisher detector_vel_pub;
    ros::Publisher control_squence_pub;
    ros::Subscriber detector_sub;
    ros::Subscriber position_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber platform_odom_sub;

    // MPC mpc;
    unique_ptr<MPC> mpc;

    map<string, double> mpc_params;
    double w_P_xy, w_P_z, w_V_xy, w_V_z, w_q, w_T, w_omega_x, w_omega_y, w_omega_z;
    double w_P_xy_n, w_P_z_n, w_V_xy_n, w_V_z_n, w_q_n;

    double w_U_x, w_U_y, w_U_z;

    // 回调函数
    void control_loop_timer_cb(const ros::TimerEvent &event);
    void detector_cb(const aruco_msgs::Detector::ConstPtr &msg);
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void platform_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

    void MPC_control();
    void FuzzyPID_control();
    void PID_control();
    void PrintInfo();
};

ros::Time c_time;
ros::Time l_time;
ros::Timer control_loop_timer;
aruco_msgs::Detector detect;
aruco_msgs::Detector_vel detector_uav_cmd;
aruco_msgs::Detector_vel last_cmd;
aruco_msgs::Control_sequence sequence_msg;
geometry_msgs::Vector3 rpy;

//
void landing_strategy(int &flag, int &status, Eigen::Vector3d Pos_err, int &land_num1, int &land_num2, int land_time1, int land_time2);
double limit(double value, double lower_bound, double upper_bound);
void rotation_yaw(float yaw_angle, float body_frame[2], float enu_frame[2]);
Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);
Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
double TimeDelay();

template <typename T>
class ControlCommand
{
public:
    ControlCommand(T msg, double time) : msg(msg), time(time){};
    T msg;
    double time;
};

std::queue<ControlCommand<aruco_msgs::Detector_vel>> Control_Queue;

/*-------------------------------------
data_record = 0  不记录数据
data_record = 1  记录数据
---------------------------------------*/
int data_record = 0;
ofstream oFile;
int file_created = 0;

/*-------------------------------------
control_method = 0   PID
control_method = 1   模糊PID
control_method = 2   模型预测控制MPC
---------------------------------------*/
int control_method;

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
std::default_random_engine generator;
std::normal_distribution<double> distribution(0.1, 0.02);
double est_delay = 0;

// Arocu_ros 输出识别位置
Eigen::Vector3d Relative_pos(0.0, 0.0, 0.0);    // 当前相对位置
Eigen::Vector3d Relative_pos_ex(0.0, 0.0, 0.0); // 前一帧相对位置
Eigen::Vector3d Relative_dpos(0.0, 0.0, 0.0);

Eigen::Vector3d Pos_target(0.0, 0.0, 0.0);  // 目标位置
Eigen::Vector3d Pos_err(0.0, 0.0, 0.0);     // 位置误差
Eigen::Vector3d Pos_err_ex(0.0, 0.0, 0.0);  // 上一次位置误差
Eigen::Vector3d Pos_err_sum(0.0, 0.0, 0.0); // 位置误差积分
Eigen::Vector3d Pos_derr(0.0, 0.0, 0.0);    // 位置误差微分

// 位置式PID
Eigen::Vector3d Pos_x_pid(0.36, 0.001, 0.04);
Eigen::Vector3d Pos_y_pid(0.36, 0.002, 0.04);
Eigen::Vector3d Pos_z_pid(0.36, 0.001, 0.01);

double yaw_rate_sp = 0.0;
double yaw_err = 0.0, yaw_err_ex = 0.0; // 姿态角误差/上一次误差
double yaw_err_sum = 0.0;               // 姿态角误差积分项
double yaw_derr = 0.0;                  // 姿态角误差微分项
Eigen::Vector3d Yaw_pid(0.5, 0.0, 0.0); // 姿态角PID

Eigen::Vector3d Output_vel_scale(6.0, 6.3, 3.6); // 输出速度调整系数

//==================================模糊PID====================================
int fuzzy_flag = 0;

Eigen::Vector3d Fpid_err_scale;  // 误差的量化因子
Eigen::Vector3d Fpid_derr_scale; // 误差微分的量化因子

Eigen::Vector3d Fpid_x_scale; // delta (kp ki kd)的量化因子
Eigen::Vector3d Fpid_y_scale;
Eigen::Vector3d Fpid_z_scale;

Eigen::Vector3d Fpid_output;       // 模糊PID的输出(增量)
Eigen::Vector3d Fpid_output_scale; // 模糊PID输出的量化因子

Incremental_PID Fpid_icm_x;
Incremental_PID Fpid_icm_y;
Incremental_PID Fpid_icm_z;

Fuzzy_PID Fpid_x;
Fuzzy_PID Fpid_y;
Fuzzy_PID Fpid_z;

double *Fpid_x_pid;
double *Fpid_y_pid;
double *Fpid_z_pid;
//=============================================================================

//====================模型预测控制Model Predictive Control=======================
int last_flag = 0;
int MPC_flag = 0;
// int NumStateVariable = 10;
// int NumInputVariable = 4;
int Np = 25;
int Nc = 25;

// vector<double> mpc_result;
//  Eigen::Matrix<double, 6, 6> A;
//  Eigen::Matrix<double, 6, 3> B;

// Eigen::VectorXd xMax, xMin;
// Eigen::VectorXd uMax, uMin;

// Eigen::DiagonalMatrix<double, 6> Q, Q_n;
// Eigen::DiagonalMatrix<double, 3> R;

Eigen::Matrix<double, 6, 1> x0;
Eigen::Matrix<double, 6, 1> xRef;

// Eigen::SparseMatrix<double> hessian;
// Eigen::VectorXd gradient;
// Eigen::SparseMatrix<double> linearMatrix;
// Eigen::SparseMatrix<double> constraintMatrix;
// Eigen::VectorXd lowerBound, upperBound;

// OsqpEigen::Solver solver;
// Eigen::Vector3d ctr;
// Eigen::VectorXd QPSolution;

// Eigen::Vector3d vel_sp;
// Eigen::Vector3d force_sp;
Eigen::Vector3d accel_sp;
Eigen::Vector3d angular_vel_sp;
Eigen::Vector3d attitude_sp;
Eigen::Vector3d rate_sp;
Eigen::Quaterniond q_sp;
Eigen::Vector3d Relative_accel_sp;

double thrust_sp = mass * g;
double thrust_sp_normalize;

// Eigen::Vector3d Relative_accel;
Eigen::Vector4d q_detect;

Eigen::Vector4d q_ugv;
Eigen::Vector3d vel_ugv;
Eigen::Vector3d vel_ugv_ex;
Eigen::Vector3d accel_ugv;
Eigen::Vector3d omega_ugv;
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
