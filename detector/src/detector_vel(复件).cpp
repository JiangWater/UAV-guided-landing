#include <iostream>
#include <ros/ros.h>
#include <math_utils.h>
#include <aruco_msgs/Detector.h>
#include <aruco_msgs/Detector_vel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "mpc.h"
#include "Fuzzy_PID.h"
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

using namespace std;
using namespace Eigen;

ros::Time c_time;
ros::Time l_time;
aruco_msgs::Detector detect;
aruco_msgs::Detector_vel detector_uav_vel_cmd;
geometry_msgs::Vector3 rpy;

// Arocu_ros 输出识别位置
double Xbody = 0.0, Ybody = 0.0, Zbody = 0.0;
double Xbody_ex = 0.0, Ybody_ex = 0.0, Zbody_ex = 0.0; // 前一帧位置

int flag = 0, id = -1, num_all = 0, num_right = 0, land = 0, status = 0, last_id = -1;
int land_num1 = 0, land_num2 = 0;
int land_time1, land_time2;

double last_err_x = 0.0, last_err_y = 0.0, last_err_z = 0.0;
double current_time = 0.0;
double last_time = 0.0;
double start_time = 0.0;

// 目标位置
double target_x;
double target_y;
double target_z;

// 位置误差
double err_x = 0.0, err_x0 = 0.0;
double err_y = 0.0, err_y0 = 0.0;
double err_z = 0.0, err_z0 = 0.0;

// 位置误差积分项
double err_sum_x = 0.0;
double err_sum_y = 0.0;
double err_sum_z = 0.0;

// PI
double P_x = 0.8, P_y = 0.8, P_z = 0.4;
double I_x = 0.007, I_y = 0.007;

// 位置式PID 增量式PID 模糊PID
double x_kp = 0.36, x_ki = 0.001, x_kd = 0.04;
double y_kp = 0.36, y_ki = 0.002, y_kd = 0.04;
double z_kp = 0.36, z_ki = 0.001, z_kd = 0.01;

// 输出速度调整系数
double k_vel_x = 6.0;
double k_vel_y = 6.3;
double k_vel_z = 3.6;

// 增量式PID delta_vel
double x_icm_vel;
double y_icm_vel;
double z_icm_vel;

Incremental_PID x_icm_pid;
Incremental_PID y_icm_pid;
Incremental_PID z_icm_pid;

// 模糊PID
int fuzzy_flag = 0;
double dt = 0.0;
double derr_x = 0.0, derr_y = 0.0, derr_z = 0.0; // 误差的微分
double x_scale_err, y_scale_err, z_scale_err;	 // 误差的量化因子
double x_scale_derr, y_scale_derr, z_scale_derr; // 误差微分的量化因子
double x_scale_kp, x_scale_ki, x_scale_kd;		 // delta (kp ki kd)的量化因子
double y_scale_kp, y_scale_ki, y_scale_kd;
double z_scale_kp, z_scale_ki, z_scale_kd;
double x_fpid_output, y_fpid_output, z_fpid_output;	   // 模糊PID的输出(增量)
double x_scale_output, y_scale_output, z_scale_output; // 模糊PID输出的量化因子

Incremental_PID x_fuzzy_icm_pid;
Incremental_PID y_fuzzy_icm_pid;
Incremental_PID z_fuzzy_icm_pid;

Incremental_PID x_fzy_pid;
Incremental_PID y_fzy_pid;
Incremental_PID z_fzy_pid;

Fuzzy_PID x_fuzzy_pid;
Fuzzy_PID y_fuzzy_pid;
Fuzzy_PID z_fuzzy_pid;

double *x_fzy_fpid;
double *y_fzy_fpid;
double *z_fzy_fpid;

// MPC
#define mass 2.0
int last_flag = 0;
int MPC_flag = 0;
int NumStateVariable = 6;
int NumInputVariable = 3;
int Np = 25;
int Nc = 25;

double last_Xbody = 0.0;
double last_Ybody = 0.0;
double last_Zbody = 0.0;
double vXbody = 0.0;
double vYbody = 0.0;
double vZbody = 0.0;

Eigen::Matrix<double, 6, 6> A;
Eigen::Matrix<double, 6, 3> B;
Eigen::MatrixXd F;
Eigen::MatrixXd Phi;

Eigen::Matrix<double, 6, 1> xMax;
Eigen::Matrix<double, 6, 1> xMin;
Eigen::Matrix<double, 3, 1> uMax;
Eigen::Matrix<double, 3, 1> uMin;

Eigen::DiagonalMatrix<double, 6> Q;
Eigen::DiagonalMatrix<double, 3> R;
Eigen::MatrixXd Q_bar;
Eigen::MatrixXd R_bar;

Eigen::MatrixXd H0;
Eigen::MatrixXd C;
Eigen::MatrixXd G;

Eigen::Matrix<double, 6, 1> x0;
Eigen::Matrix<double, 6, 1> xRef;

Eigen::SparseMatrix<double> hessian;
Eigen::VectorXd gradient;
Eigen::SparseMatrix<double> linearMatrix;
Eigen::VectorXd lowerBound;
Eigen::VectorXd upperBound;

OsqpEigen::Solver solver;
Eigen::Vector3d ctr;
Eigen::VectorXd QPSolution;

//---------------------------------------t265定位相关------------------------------------------
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d Euler_t265;
geometry_msgs::TransformStamped t265;
geometry_msgs::TransformStamped t265_last;

//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;
Eigen::Vector3d vel_drone_fcu;

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu; // 无人机当前欧拉角(来自fcu)

/*-------------------------------------
data_record = 0  不记录数据
data_record = 1  记录数据
---------------------------------------*/
int data_record = 0;
ofstream oFile;
int file_created = 0;

/*-------------------------------------
control_method = 0   程序原始的PI控制
control_method = 1   PID
control_method = 2   增量 PID
control_method = 3   模糊 PID
control_method = 4   模型预测控制MPC
---------------------------------------*/
int control_method;

// 坐标转换
void detector_cb(const aruco_msgs::Detector::ConstPtr &msg)
{
	detect = *msg;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(detect.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	rpy.x = roll;
	rpy.y = pitch;
	rpy.z = yaw;

	Xbody_ex = Xbody;
	Ybody_ex = Ybody;
	Zbody_ex = Zbody;

	Xbody = -detect.pose.position.y;
	Ybody = detect.pose.position.x;
	Zbody = -detect.pose.position.z;

	// 识别丢失情况
	if (Ybody == -1.0)
	{
		Xbody = Xbody_ex;
		Ybody = Ybody_ex;
		Zbody = Zbody_ex;
	}
	last_id = id;
	id = detect.id;
	num_all++;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
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

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	// Read the Drone Velocity from the Mavros Package [Frame: ENU]
	Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

	vel_drone_fcu = vel_drone_fcu_enu;
}

void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	if (msg->header.frame_id == "camera_odom_frame")
	{
		pos_drone_t265[0] = msg->pose.pose.position.x;
		pos_drone_t265[1] = msg->pose.pose.position.y;
		// pos_drone_t265[2] = msg->pose.pose.position.z;
		pos_drone_t265[2] = pos_drone_fcu[2];
		Eigen::Quaterniond q_t265_enu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
		q_t265 = q_t265_enu;
		Euler_t265 = quaternion_to_euler(q_t265);
	}
}

int main(int argc, char **argv)
{
	// ROS订阅与发布
	ros::init(argc, argv, "detector_vel_node");
	ros::NodeHandle nh;
	nh.param<int>("BasicParam/control_method", control_method, 4);
	// nh.param<int>("BasicParam/data_record", data_record, 1);
	nh.param<double>("ControlParam/target_z", target_z, -0.6);
	nh.param<int>("ControlParam/land_time1", land_time1, 160);
	nh.param<int>("ControlParam/land_time2", land_time2, 30);
	ros::Subscriber detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 25, detector_cb);
	ros::Publisher detector_vel_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
	ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);
	ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1000, t265_cb);
	ros::Rate rate(25.0);

	/***************************************************************************************************************************
	 * 数据记录
	 *   1        2          3          4        5 6 7     8     9     10     11 12 13      14 15 16
	 * method | 时间戳 | 捕获/丢失flag | flag | 定位x y z | Xbody Ybody Zbody | 速度x y z | 速度sp x y z|
	 ***************************************************************************************************************************/
	if (file_created == 0)
	{
		char filename[200] = {0};
		time_t time_now = time(NULL);
		tm *p = localtime(&time_now);
		sprintf(filename, "/home/itr/aruco_ws/src/detector/data_record/Data %d-%d-%d %d:%02d.csv", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min);
		oFile.open(filename, ios::out | ios::app);
		file_created = 1;
		oFile << "Control method"
			  << ","
			  << "time"
			  << ","
			  << "id"
			  << ","
			  << "flag"
			  << ","
			  << "Pos_x"
			  << ","
			  << "Pos_y"
			  << ","
			  << "Pos_z"
			  << ","
			  << "Xbody"
			  << ","
			  << "Ybody"
			  << ","
			  << "Zbody"
			  << ","
			  << "Vel_x"
			  << ","
			  << "Vel_y"
			  << ","
			  << "Vel_z"
			  << ","
			  << "Vel_sp_x"
			  << ","
			  << "Vel_sp_y"
			  << ","
			  << "Vel_sp_z"
			  << endl;
	}

	c_time = ros::Time::now();
	start_time = c_time.toSec();

	while (ros::ok())
	{

		if (data_record != 1 && last_id == -1 && id != -1)
		{
			data_record = 1;
		}

		switch (control_method)
		{
		// PI
		case 0:
			// 位置识别限幅
			if (Xbody > 1.0)
			{
				Xbody = 0.5;
			}
			if (Xbody < -1.0)
			{
				Xbody = -0.5;
			}
			if (Ybody > 1.0)
			{
				Ybody = 0.5;
			}
			if (Ybody < -1.0)
			{
				Ybody = -0.5;
			}

			// 位置误差
			err_x = target_x - Xbody;
			err_y = target_y - Ybody;
			err_z = target_z - Zbody;

			// 位置误差积分
			err_sum_x += err_x;
			err_sum_y += err_y;
			err_sum_z += err_z;

			// 位置误差微分限幅
			if (err_sum_x > 5)
			{
				err_sum_x = 5;
			}
			if (err_sum_x < -5)
			{
				err_sum_x = -5;
			}
			if (err_sum_y > 2)
			{
				err_sum_y = 2;
			}
			if (err_sum_y < -2)
			{
				err_sum_y = -2;
			}
			if (err_sum_z > 5)
			{
				err_sum_z = 5;
			}
			if (err_sum_z < -5)
			{
				err_sum_z = -5;
			}

			// 满足marker在摄像头内条件，进入降落第一阶段
			if (id != -1)
				num_right++;
			if (num_all >= 30 && flag == 0 && num_right >= 25)
			{
				flag = 1;
				num_all = 0;
				num_right = 0;
			}
			else if (num_all >= 30)
			{
				num_all = 0;
				num_right = 0;
			}

			// 顺利降落到相机坐标系下（0,0,0.3）左右，进入降落第二阶段
			if (flag == 1 && (-0.3) < err_x && err_x < 0.3 && (-0.3) < err_y && err_y < 0.3 && (-0.25) > Zbody && Zbody > (-1.1) && (id != -1))
			{
				flag = 2;
				target_z = 0;
				err_sum_z = 0;
				ROS_INFO("case2");
			}

			ROS_INFO("case3      err_x:%f,err_y:%f,Zbody:%f,id:%d ", err_x, err_x, Zbody, id);

			// 顺利降落到相机坐标系下（0,0,0.1）左右，进入降落第三阶段
			if (flag == 2 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && Zbody > (-0.2) && (id != -1))
			{
				flag = 3;
				ROS_INFO("case3");
			}

			if (id != -1)
			{
				detector_uav_vel_cmd.vel.linear.x = P_x * err_x + I_x * err_sum_x;
				detector_uav_vel_cmd.vel.linear.y = P_y * err_y + I_y * err_sum_y;

				if (Zbody < (-0.3))
				{
					detector_uav_vel_cmd.vel.linear.z = P_z * err_z;
				}
				else
				{
					detector_uav_vel_cmd.vel.linear.z = 0.5 * err_z;
				}

				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			else
			{
				detector_uav_vel_cmd.vel.linear.x = 0;
				detector_uav_vel_cmd.vel.linear.y = 0;
				detector_uav_vel_cmd.vel.linear.z = 0;
				detector_uav_vel_cmd.flag = 0;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			break;

		//-----------------------------------------------------------------------------------------
		// PID
		case 1:

			if (id != -1 && land == 0)
			{
				flag = 1;
			}

			// 时间微分
			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{
				// 误差e
				err_x = target_x - Xbody;
				err_y = target_y - Ybody;
				err_z = target_z - Zbody;

				err_sum_x += err_x;
				err_sum_y += err_y;
				err_sum_z += err_z;

				if (err_sum_y > 2)
				{
					err_sum_y = 2;
				}
				if (err_sum_y < -2)
				{
					err_sum_y = -2;
				}

				if (err_sum_x > 2)
				{
					err_sum_x = 2;
				}
				if (err_sum_x < -2)
				{
					err_sum_x = -2;
				}

				if (err_sum_z > 1)
				{
					err_sum_z = 1;
				}
				if (err_sum_z < -0.5)
				{
					err_sum_z = -0.5;
				}

				// 误差的微分ec
				derr_x = (err_x - last_err_x) / dt;
				derr_y = (err_y - last_err_y) / dt;
				derr_z = (err_z - last_err_z) / dt;
				detector_uav_vel_cmd.vel.linear.x = x_kp * err_x + x_ki * err_sum_x + x_kd * derr_x;
				detector_uav_vel_cmd.vel.linear.y = y_kp * err_y + y_ki * err_sum_y + y_kd * derr_y;
				detector_uav_vel_cmd.vel.linear.z = z_kp * err_z + z_ki * err_sum_z + z_kd * derr_z;
				detector_uav_vel_cmd.vel.linear.x = k_vel_x * detector_uav_vel_cmd.vel.linear.x;
				detector_uav_vel_cmd.vel.linear.y = k_vel_y * detector_uav_vel_cmd.vel.linear.y;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				if (detector_uav_vel_cmd.vel.linear.x > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.x < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = -0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = -0.5;
				}
				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && (-0.4) > Zbody && Zbody > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= land_time1)
					{
						flag = 1;
						target_z = -0.2;
						status = 1;
					}
				}

				if (status == 1 && Zbody > (-0.3))
				{
					land_num2++;
					if (land_num2 >= land_time2)
					{
						flag = 3;
					}
				}

				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}
				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			last_err_x = err_x;
			last_err_y = err_y;
			last_err_z = err_z;
			last_time = current_time;
			break;

		//-----------------------------------------------------------------------------------------
		// 增量式PID
		case 2:

			if (id != -1)
			{
				flag = 1;
			}

			// 时间微分
			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{
				x_icm_pid.init(0.0, x_kp, x_ki, x_kd);
				y_icm_pid.init(0.0, y_kp, y_ki, y_kd);

				x_icm_vel = x_icm_pid.Incremental_PID_output(Xbody);
				y_icm_vel = y_icm_pid.Incremental_PID_output(Ybody);

				detector_uav_vel_cmd.vel.linear.x += 4.6 * x_icm_vel;
				detector_uav_vel_cmd.vel.linear.y += 4.6 * y_icm_vel;

				err_z = target_z - Zbody;
				err_sum_z += err_z;
				derr_z = (err_z - last_err_z) / dt;

				if (err_sum_z > 1)
				{
					err_sum_z = 1;
				}
				if (err_sum_z < -0.5)
				{
					err_sum_z = -0.5;
				}

				detector_uav_vel_cmd.vel.linear.z = z_kp * err_z + z_ki * err_sum_z + z_kd * derr_z;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				if (detector_uav_vel_cmd.vel.linear.x > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.x < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = -0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = -0.5;
				}

				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && (-0.4) > Zbody && Zbody > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= land_time1)
					{
						flag = 1;
						target_z = -0.2;
						status = 1;
					}
				}

				if (status == 1)
				{
					land_num2++;
					if (land_num2 >= land_time2)
					{
						flag = 3;
					}
				}
				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}

				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			last_time = current_time;
			break;

		//-----------------------------------------------------------------------------------------
		//  模糊PID
		case 3:

			if (id != -1)
			{
				flag = 1;
			}

			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{

				// 误差e(输入1)
				err_x = 0.0 - Xbody;
				err_y = 0.0 - Ybody;

				// 误差的微分ec(输入2)
				derr_x = (err_x - last_err_x) / dt;
				derr_y = (err_y - last_err_y) / dt;

				err_sum_x += err_x;
				err_sum_y += err_y;

				if (err_sum_y > 2)
				{
					err_sum_y = 2;
				}
				if (err_sum_y < -2)
				{
					err_sum_y = -2;
				}

				if (err_sum_x > 2)
				{
					err_sum_x = 2;
				}
				if (err_sum_x < -2)
				{
					err_sum_x = -2;
				}

				// 赋PID初值
				if (fuzzy_flag == 0)
				{
					x_fuzzy_pid.init(0.0, x_kp, x_ki, x_kd);
					y_fuzzy_pid.init(0.0, y_kp, y_ki, y_kd);
					fuzzy_flag = 1;
				}

				// delta(P I D)和两个输入的量化因子(需要调整)
				x_scale_kp = 0.036 / 3, x_scale_ki = 0.0002 / 3, x_scale_kd = 0.005 / 3, x_scale_err = 15, x_scale_derr = 15 * dt;
				y_scale_kp = 0.036 / 3, y_scale_ki = 0.0002 / 3, y_scale_kd = 0.005 / 3, y_scale_err = 15, y_scale_derr = 15 * dt;

				x_fzy_fpid = x_fuzzy_pid.Fuzzy_PID_output(Xbody, err_x, derr_x, x_scale_kp, x_scale_ki, x_scale_kd, x_scale_err, x_scale_derr, x_fuzzy_icm_pid);
				y_fzy_fpid = y_fuzzy_pid.Fuzzy_PID_output(Ybody, err_y, derr_y, y_scale_kp, y_scale_ki, y_scale_kd, y_scale_err, y_scale_derr, y_fuzzy_icm_pid);

				x_fpid_output = x_fzy_fpid[0] * err_x + x_fzy_fpid[1] * err_sum_x + x_fzy_fpid[2] * derr_x;
				y_fpid_output = y_fzy_fpid[0] * err_y + y_fzy_fpid[1] * err_sum_y + y_fzy_fpid[2] * derr_y;

				// 输出的量化因子(需要调整)
				x_scale_output = 6.4;
				y_scale_output = 6.4;

				detector_uav_vel_cmd.vel.linear.x = x_fpid_output * x_scale_output;
				detector_uav_vel_cmd.vel.linear.y = y_fpid_output * y_scale_output;

				err_z = target_z - Zbody;
				err_sum_z += err_z;
				derr_z = (err_z - last_err_z) / dt;

				if (err_sum_z > 1)
				{
					err_sum_z = 1;
				}
				if (err_sum_z < -0.5)
				{
					err_sum_z = -0.5;
				}

				detector_uav_vel_cmd.vel.linear.z = z_kp * err_z + z_ki * err_sum_z + z_kd * derr_z;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && (-0.4) > Zbody && Zbody > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= land_time1)
					{
						flag = 1;
						target_z = -0.25;
						status = 1;
					}
				}

				if (status == 1)
				{
					land_num2++;
					if (land_num2 >= land_time2)
					{
						flag = 3;
					}
				}

				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}

				if (detector_uav_vel_cmd.vel.linear.x > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.x < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.x = -0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y > 0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = 0.5;
				}

				if (detector_uav_vel_cmd.vel.linear.y < -0.5)
				{
					detector_uav_vel_cmd.vel.linear.y = -0.5;
				}

				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}

			last_time = current_time;
			last_err_x = err_x;
			last_err_y = err_y;
			last_err_z = err_z;

			break;

		//-----------------------------------------------------------------------------------------
		//  MPC
		case 4:

			Np = 25;
			Nc = 25;

			if (id != -1)
			{
				flag = 1;
			}

			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (last_flag == 1)
			{
				vXbody = (Xbody - last_Xbody) / dt;
				vYbody = (Ybody - last_Ybody) / dt;
				vZbody = (Zbody - last_Zbody) / dt;
				x0 << Xbody, vXbody, Ybody, vYbody, Zbody, vZbody;
			}

			if (dt > 0.001 && MPC_flag == 1 && last_flag == 1)
			{
				GradientMatrices(G, C, x0, xRef, gradient);

				solver.updateGradient(gradient);

				if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
					return 1;

				QPSolution = solver.getSolution();
				ctr = QPSolution.head(NumInputVariable);
				detector_uav_vel_cmd.vel.linear.x = detector_uav_vel_cmd.vel.linear.x + (ctr[0] * dt);
				detector_uav_vel_cmd.vel.linear.y = detector_uav_vel_cmd.vel.linear.y + (ctr[1] * dt);
				detector_uav_vel_cmd.vel.linear.z = detector_uav_vel_cmd.vel.linear.z + (ctr[2] * dt);
			}

			if (dt > 0.001 && MPC_flag == 0 && last_flag == 1)
			{
				xRef << 0, 0, 0, 0, -0.5, 0;
				setDynamicsMatrices(A, B);
				setInequalityConstraints(xMax, xMin, uMax, uMin);
				setWeightMatrices(Q, R);

				PredictMatrices(F, Phi, A, B, Np, Nc);
				// MatrixXd Q_bar = MatrixXd::Zero(Np * NumStateVariable, Np * NumStateVariable);
				// MatrixXd R_bar = MatrixXd::Zero(Np * NumInputVariable, Np * NumInputVariable);
				WeightMatrices(Q_bar, R_bar, Q, R, Np, NumStateVariable, NumInputVariable);
				QuadProgMatrices(H0, C, Q_bar, R_bar, F, Phi, Np, NumStateVariable, NumInputVariable, hessian);

				ConstraintMatrices(NumStateVariable, NumInputVariable, Np, linearMatrix);
				ConstraintVectors(uMax, uMin, NumInputVariable, Np, lowerBound, upperBound);

				GradientMatrices(G, C, x0, xRef, gradient);

				solver.settings()->setWarmStart(true);

				solver.data()->setNumberOfVariables(Np * NumInputVariable);
				solver.data()->setNumberOfConstraints(Np * NumInputVariable);

				if (!solver.data()->setHessianMatrix(hessian))
					return 1;
				if (!solver.data()->setGradient(gradient))
					return 1;
				if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
					return 1;
				if (!solver.data()->setLowerBound(lowerBound))
					return 1;
				if (!solver.data()->setUpperBound(upperBound))
					return 1;

				if (!solver.initSolver())
					return 1;
				if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
					return 1;

				QPSolution = solver.getSolution();

				MPC_flag = 1;

				ctr = QPSolution.head(NumInputVariable);
				detector_uav_vel_cmd.vel.linear.x = vel_drone_fcu[0] + (ctr[0] * dt);
				detector_uav_vel_cmd.vel.linear.y = vel_drone_fcu[1] + (ctr[1] * dt);
				detector_uav_vel_cmd.vel.linear.z = vel_drone_fcu[2] + (ctr[2] * dt);
			}

			if (detector_uav_vel_cmd.vel.linear.x > 0.1)
			{
				detector_uav_vel_cmd.vel.linear.x = 0.1;
			}
			else if (detector_uav_vel_cmd.vel.linear.x < -0.1)
			{
				detector_uav_vel_cmd.vel.linear.x = -0.1;
			}

			if (detector_uav_vel_cmd.vel.linear.y > 0.1)
			{
				detector_uav_vel_cmd.vel.linear.y = 0.1;
			}
			else if (detector_uav_vel_cmd.vel.linear.y < -0.1)
			{
				detector_uav_vel_cmd.vel.linear.y = -0.1;
			}

			// 降落
			if (flag == 1 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && (-0.4) > Zbody && Zbody > (-1.0) && (id != -1) && status == 0)
			{
				land_num1++;
				if (land_num1 >= land_time1)
				{
					flag = 1;
					target_z = -0.2;
					status = 1;
				}
			}

			if (status == 1 && Zbody > (-0.3))
			{
				land_num2++;
				if (land_num2 >= land_time2)
				{
					flag = 3;
				}
			}

			if (flag == 0)
			{
				detector_uav_vel_cmd.vel.linear.x = 0;
				detector_uav_vel_cmd.vel.linear.y = 0;
				detector_uav_vel_cmd.vel.linear.z = 0;
			}

			detector_uav_vel_cmd.flag = flag;
			detector_vel_pub.publish(detector_uav_vel_cmd);

			last_flag = flag;
			last_time = current_time;
			last_Xbody = Xbody;
			last_Ybody = Ybody;
			last_Zbody = Zbody;

			break;
		default:
			break;
		}

		ROS_INFO("********************Param********************");
		ROS_INFO("control_method:%d", control_method);
		ROS_INFO("data_record:%d", data_record);
		ROS_INFO("land_time1:%d", land_time1);
		ROS_INFO("land_time2:%d", land_time2);
		ROS_INFO("flag:%d", detector_uav_vel_cmd.flag);
		// ROS_INFO("status:%d", status);
		ROS_INFO("****************Absolute Pos****************");
		ROS_INFO("Pos_x:%f", pos_drone_t265[0]);
		ROS_INFO("Pos_y:%f", pos_drone_t265[1]);
		ROS_INFO("Pos_z:%f", pos_drone_t265[2]);
		ROS_INFO("**************** Relative Pos****************");
		ROS_INFO("Xbody:%f", Xbody);
		ROS_INFO("Ybody:%f", Ybody);
		ROS_INFO("Zbody:%f", Zbody);
		ROS_INFO("********************Speed********************");
		ROS_INFO("vel_x:%f", vel_drone_fcu[0]);
		ROS_INFO("vel_y:%f", vel_drone_fcu[1]);
		ROS_INFO("vel_z:%f", vel_drone_fcu[2]);
		ROS_INFO("******************Speed Sp*******************");
		ROS_INFO("vel_sp_x:%f", detector_uav_vel_cmd.vel.linear.x);
		ROS_INFO("vel_sp_y:%f", detector_uav_vel_cmd.vel.linear.y);
		ROS_INFO("vel_sp_z:%f", detector_uav_vel_cmd.vel.linear.z);
		// ROS_INFO("ctr[0] * dt:%f", ctr[0] * dt);
		// ROS_INFO("ctr[1] * dt:%f", ctr[1] * dt);
		// ROS_INFO("ctr[2] * dt:%f", ctr[2] * dt);
		// ROS_INFO("*********************************************");
		/*
				if (data_record == 1 && id != -1)
				{
					oFile << err_x << "," << err_y << "," << Zbody << "," << detector_uav_vel_cmd.vel.linear.x << "," << detector_uav_vel_cmd.vel.linear.y << "," << detector_uav_vel_cmd.vel.linear.z << "," << err_sum_x << "," << err_sum_y << "," << derr_x << "," << derr_y << "," << current_time << endl; // endl 换行
				}
		*/

		if (file_created == 1 && data_record == 1)
		{
			oFile << control_method << "," << (current_time - start_time) << "," << id << "," << flag << "," << pos_drone_t265[0] << "," << pos_drone_t265[1] << "," << pos_drone_t265[2] << "," << Xbody << "," << Ybody << "," << Zbody << "," << vel_drone_fcu[0] << "," << vel_drone_fcu[1] << "," << vel_drone_fcu[2] << "," << detector_uav_vel_cmd.vel.linear.x << "," << detector_uav_vel_cmd.vel.linear.y << "," << detector_uav_vel_cmd.vel.linear.z << endl; // endl 换行
		}

		ros::spinOnce();
		rate.sleep();
	}
	oFile.close();
	return 0;
}
