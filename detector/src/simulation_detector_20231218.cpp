#include "simulation_detector.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
	// ROS订阅与发布
	ros::init(argc, argv, "detector_vel_node");
	ros::NodeHandle nh;
	nh.param<int>("BasicParam/control_method", control_method, 0);
	nh.param<int>("BasicParam/data_record", data_record, 1);
	nh.param<int>("BasicParam/land_flag", land_flag, 1);
	nh.param<double>("ControlParam/target_z", Pos_target(2), -0.8);
	nh.param<int>("ControlParam/land_time1", land_time1, 160);
	nh.param<int>("ControlParam/land_time2", land_time2, 30);
	nh.param<double>("ControlParam/yaw_kp", Yaw_pid(0), 0.5);
	nh.param<double>("ControlParam/yaw_ki", Yaw_pid(1), 0.0);
	nh.param<double>("ControlParam/yaw_kd", Yaw_pid(2), 0.0);
	nh.param<double>("UAVParam/hover_throttle", hover_throttle, 0.43);
	nh.param<double>("UAVParam/mass", mass, 1.52);
	nh.param<double>("UAVParam/arm_length", arm_length, 0.225);
	nh.param<double>("UAVParam/max_tilt", max_tilt, pi / 9);
	nh.param<int>("MpcParam/Np", Np, 20);
	nh.param<int>("MpcParam/Nc", Nc, 20);
	control_loop_timer = nh.createTimer(ros::Duration(0.02), control_loop_timer_cb);

	detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 25, detector_cb);

	detector_vel_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);
	control_squence_pub = nh.advertise<aruco_msgs::Control_sequence>("/aruco_single/control_squence", 25);

	position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
	velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);
	platform_odom_sub = nh.subscribe<nav_msgs::Odometry>("/moving_platform/wheelcontrol/odom", 10, platform_odom_cb);
	// ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1000, t265_cb);
	// ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 25);
	ros::spin();
	return 0;
}

void control_loop_timer_cb(const ros::TimerEvent &event)
{
	/***************************************************************************************************************************
	 * 数据记录
	 *   1        2          3          4        5 6 7     8     9     10     11 12 13      14 15 16
	 * method | 时间戳 | 捕获/丢失flag | flag | 定位x y z | Relative_pos(0) Relative_pos(1) Relative_pos(2) | 速度x y z | 速度sp x y z|
	 ***************************************************************************************************************************/

	if (data_record == 1 && file_created == 0)
	{
		char filename[200] = {0};
		time_t time_now = time(NULL);
		tm *p = localtime(&time_now);
		sprintf(filename, "/home/yx/experiment_record/data_record/Data %d-%d-%d %d:%02d.csv", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min);
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
			  << "Relative_pos(0)"
			  << ","
			  << "Relative_pos(1)"
			  << ","
			  << "Relative_pos(2)"
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
			  << ","
			  << "detected_yaw"
			  << ","
			  << "yaw_rate_sp"
			  << endl;
	}

	c_time = ros::Time::now();
	start_time = c_time.toSec();

	switch (control_method)
	{
	// PID
	case 0:
		PID_control();
		break;

	//  模糊PID
	case 1:
		// FuzzyPID_control();
		break;

	//  MPC
	case 2:
		MPC_control();
		break;

	default:
		break;
	}

	PrintInfo();

	if (file_created == 1 && data_record == 1)
	{
		oFile << control_method << "," << (current_time - start_time) << "," << id << "," << flag << "," << Relative_pos(0) << "," << Relative_pos(1) << "," << Relative_pos(2) << "," << linear_vel_drone_fcu(0) << "," << linear_vel_drone_fcu(1) << "," << linear_vel_drone_fcu(2) << "," << detector_uav_cmd.vel.linear.x << "," << detector_uav_cmd.vel.linear.y << "," << detector_uav_cmd.vel.linear.z << "," << rpy.z << "," << yaw_rate_sp << endl; // endl 换行
	}
}

void MPC_control()
{
	// Np = 20;
	// Nc = 20;

	if (id != -1 && land == 0)
	{
		flag = 1;
	}

	c_time = ros::Time::now();
	current_time = c_time.toSec();
	dt = current_time - last_time;

	if (last_flag == 1)
	{
		// ROS_INFO("-----> breakpoint:%d <-----", breakpoint++);
		Relative_dpos(0) = (Relative_pos(0) - Relative_pos_ex(0)) / dt;
		Relative_dpos(1) = (Relative_pos(1) - Relative_pos_ex(1)) / dt;
		Relative_dpos(2) = (Relative_pos(2) - Relative_pos_ex(2)) / dt;

		accel_ugv(0) = (vel_ugv(0) - vel_ugv_ex(0)) / dt;
		accel_ugv(1) = (vel_ugv(1) - vel_ugv_ex(1)) / dt;
		accel_ugv(2) = (vel_ugv(2) - vel_ugv_ex(2)) / dt;

		x0 << -Relative_pos(0), -Relative_dpos(0), -Relative_pos(1), -Relative_dpos(1), -Relative_pos(2), -Relative_dpos(2);

		xRef << Pos_target(0), 0, Pos_target(1), 0, -Pos_target(2), 0;

		Pos_err(0) = xRef(0) - x0(0);
		Pos_err(1) = xRef(2) - x0(2);
		Pos_err(2) = xRef(4) - x0(4);
	}

	if (dt > 0.001 && MPC_flag == 1 && last_flag == 1)
	{
		updateConstraintVectors(x0, lowerBound, upperBound);

		solver.updateBounds(lowerBound, upperBound);
		solver.solveProblem();

		QPSolution = solver.getSolution();
		ctr = QPSolution.block(NumStateVariable * (Np + 1), 0, NumInputVariable, 1);
	}

	if (dt > 0.001 && MPC_flag == 0 && last_flag == 1)
	{
		setDynamicsMatrices(A, B, dt);
		setInequalityConstraints(xMax, xMin, uMax, uMin);
		setWeightMatrices(Q, R, Q_n);

		setHessianMatrices(Q, Q_n, R, Np, Nc, NumStateVariable, NumInputVariable, hessian);
		setGradientMatrices(Q, Q_n, xRef, Np, Nc, NumStateVariable, NumInputVariable, gradient);
		setConstraintMatrix(A, B, Np, Nc, NumStateVariable, NumInputVariable, constraintMatrix);
		setConstraintVectors(xMax, xMin, uMax, uMin, x0, Np, Nc, NumStateVariable, NumInputVariable, lowerBound, upperBound);

		solver.settings()->setWarmStart(true);

		solver.data()->setNumberOfVariables(NumStateVariable * (Np + 1) + NumInputVariable * Np);
		solver.data()->setNumberOfConstraints(2 * NumStateVariable * (Np + 1) + NumInputVariable * Np);

		solver.data()->setHessianMatrix(hessian);
		solver.data()->setGradient(gradient);
		solver.data()->setLinearConstraintsMatrix(constraintMatrix);
		solver.data()->setLowerBound(lowerBound);
		solver.data()->setUpperBound(upperBound);
		solver.initSolver();
		solver.solveProblem();

		QPSolution = solver.getSolution();

		MPC_flag = 1;

		ctr = QPSolution.block(NumStateVariable * (Np + 1), 0, NumInputVariable, 1);
	}

	thrust_sp_normalize = hover_throttle;

	if (id != -1 && MPC_flag == 1)
	{
		// accel_sp(0) = ctr(0);
		// accel_sp(1) = ctr(1);
		accel_sp(2) = g + ctr(2);

		// !!!!!!!!!!!!!!!!  Euler_fcu来自飞控 在反馈回路引入了延迟
		accel_sp(0) = ctr(0) * std::cos(Euler_fcu(2)) - ctr(1) * std::sin(Euler_fcu(2));
		accel_sp(1) = ctr(0) * std::sin(Euler_fcu(2)) + ctr(1) * std::cos(Euler_fcu(2));

		thrust_sp = accel_sp(2) / (std::cos(Euler_fcu(0)) * std::cos(Euler_fcu(1)));

		thrust_sp_normalize = thrust_sp * hover_throttle / g;
		thrust_sp_normalize = limit(thrust_sp_normalize, 0.25, 0.6);

		// attitude_sp(2) = attitude_sp(2) + 0.5 * (0 - rpy.z);
		attitude_sp(2) = Euler_fcu(2) - 3.5 * rpy.z;
		attitude_sp(0) = std::asin((accel_sp(0) * std::sin(Euler_fcu(2)) - accel_sp(1) * std::cos(Euler_fcu(2))) / thrust_sp);
		attitude_sp(1) = std::atan((accel_sp(0) * std::cos(Euler_fcu(2)) + accel_sp(1) * std::sin(Euler_fcu(2))) / accel_sp(2));

		q_sp = Eigen::AngleAxisd(attitude_sp(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(attitude_sp(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(attitude_sp(0), Eigen::Vector3d::UnitX());
	}

	if (land_flag == 1)
		landing_strategy(flag, status, Pos_err, land_num1, land_num2, land_time1, land_time2);

	detector_uav_cmd.attitude_control_flag = true;
	detector_uav_cmd.attitude_control_type_mask = 0b00000111;
	// detector_uav_cmd.attitude_control_type_mask = 0b10000000;
	detector_uav_cmd.thrust = thrust_sp_normalize;

	detector_uav_cmd.quat.w = q_sp.w();
	detector_uav_cmd.quat.x = q_sp.x();
	detector_uav_cmd.quat.y = q_sp.y();
	detector_uav_cmd.quat.z = q_sp.z();

	// detector_uav_cmd.vel.linear.x = vel_sp(0);
	// detector_uav_cmd.vel.linear.y = vel_sp(1);
	// detector_uav_cmd.vel.linear.z = vel_sp(2);

	detector_uav_cmd.flag = flag;
	// detector_vel_pub.publish(detector_uav_cmd);
	Control_Queue.push(ControlCommand<aruco_msgs::Detector_vel>(detector_uav_cmd, current_time));

	if (!Control_Queue.empty() && ros::Time::now().toSec() - Control_Queue.front().time >= TimeDelay())
	{
		detector_vel_pub.publish(Control_Queue.front().msg);
		last_cmd = Control_Queue.front().msg;
		Control_Queue.pop();
	}
	else
		detector_vel_pub.publish(last_cmd);

	last_flag = flag;
	last_time = current_time;

	Relative_pos_ex(0) = Relative_pos(0);
	Relative_pos_ex(1) = Relative_pos(1);
	Relative_pos_ex(2) = Relative_pos(2);

	vel_ugv_ex(0) = vel_ugv(0);
	vel_ugv_ex(1) = vel_ugv(1);
	vel_ugv_ex(2) = vel_ugv(2);
}

/*
void FuzzyPID_control()
{
	if (id != -1 && land == 0)
	{
		flag = 1;
	}

	c_time = ros::Time::now();
	current_time = c_time.toSec();
	dt = current_time - last_time;

	if (dt > 0.001)
	{

		// 误差e(输入1)
		Pos_err(0) = 0.0 - Relative_pos(0);
		Pos_err(1) = 0.0 - Relative_pos(1);

		// 误差的微分ec(输入2)
		Pos_derr(0) = (Pos_err(0) - Pos_err_ex(0)) / dt;
		Pos_derr(1) = (Pos_err(1) - Pos_err_ex(1)) / dt;

		Pos_err_sum(0) += Pos_err(0);
		Pos_err_sum(1) += Pos_err(1);

		if (Pos_err_sum(1) > 2)
		{
			Pos_err_sum(1) = 2;
		}
		if (Pos_err_sum(1) < -2)
		{
			Pos_err_sum(1) = -2;
		}

		if (Pos_err_sum(0) > 2)
		{
			Pos_err_sum(0) = 2;
		}
		if (Pos_err_sum(0) < -2)
		{
			Pos_err_sum(0) = -2;
		}

		// 赋PID初值
		if (fuzzy_flag == 0)
		{
			Fpid_x.init(0.0, Pos_x_pid(0), Pos_x_pid(1), Pos_x_pid(2));
			Fpid_y.init(0.0, Pos_y_pid(0), Pos_y_pid(1), Pos_y_pid(2));
			fuzzy_flag = 1;
		}

		// delta(P I D)和两个输入的量化因子(需要调整)
		Fpid_x_scale(0) = 0.036 / 3, Fpid_x_scale(1) = 0.0002 / 3, Fpid_x_scale(2) = 0.005 / 3, Fpid_err_scale(0) = 15, Fpid_derr_scale(0) = 15 * dt;
		Fpid_y_scale(0) = 0.036 / 3, Fpid_y_scale(1) = 0.0002 / 3, Fpid_y_scale(2) = 0.005 / 3, Fpid_err_scale(1) = 15, Fpid_derr_scale(1) = 15 * dt;

		Fpid_x_pid = Fpid_x.Fuzzy_PID_output(Relative_pos(0), Pos_err(0), Pos_derr(0), Fpid_x_scale(0), Fpid_x_scale(1), Fpid_x_scale(2), Fpid_err_scale(0), Fpid_derr_scale(0), Fpid_icm_x);
		Fpid_y_pid = Fpid_y.Fuzzy_PID_output(Relative_pos(1), Pos_err(1), Pos_derr(1), Fpid_y_scale(0), Fpid_y_scale(1), Fpid_y_scale(2), Fpid_err_scale(1), Fpid_derr_scale(1), Fpid_icm_y);

		Fpid_output(0) = Fpid_x_pid(0) * Pos_err(0) + Fpid_x_pid(1) * Pos_err_sum(0) + Fpid_x_pid(2) * Pos_derr(0);
		Fpid_output(1) = Fpid_y_pid(0) * Pos_err(1) + Fpid_y_pid(1) * Pos_err_sum(1) + Fpid_y_pid(2) * Pos_derr(1);

		// 输出的量化因子(需要调整)
		Fpid_output_scale(0) = 6.4;
		Fpid_output_scale(1) = 6.4;

		detector_uav_cmd.vel.linear.x = Fpid_output(0) * Fpid_output_scale(0);
		detector_uav_cmd.vel.linear.y = Fpid_output(1) * Fpid_output_scale(1);

		Pos_err(2) = Pos_target(2) - Relative_pos(2);
		Pos_err_sum(2) += Pos_err(2);
		Pos_derr(2) = (Pos_err(2) - Pos_err_ex(2)) / dt;

		if (Pos_err_sum(2) > 1)
		{
			Pos_err_sum(2) = 1;
		}
		if (Pos_err_sum(2) < -0.5)
		{
			Pos_err_sum(2) = -0.5;
		}

		detector_uav_cmd.vel.linear.z = Pos_z_pid(0) * Pos_err(2) + Pos_z_pid(1) * Pos_err_sum(2) + Pos_z_pid(2) * Pos_derr(2);
		detector_uav_cmd.vel.linear.z = Output_vel_scale(2) * detector_uav_cmd.vel.linear.z;

		// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
		if (flag == 1 && (-0.1) < Pos_err(0) && Pos_err(0) < 0.1 && (-0.1) < Pos_err(1) && Pos_err(1) < 0.1 && (-0.4) > Relative_pos(2) && Relative_pos(2) > (-1.0) && (id != -1) && status == 0)
		{
			land_num1++;
			if (land_num1 >= land_time1)
			{
				flag = 1;
				Pos_target(2) = -0.25;
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
			detector_uav_cmd.vel.linear.x = 0;
			detector_uav_cmd.vel.linear.y = 0;
			detector_uav_cmd.vel.linear.z = 0;
		}

		if (detector_uav_cmd.vel.linear.x > 0.5)
		{
			detector_uav_cmd.vel.linear.x = 0.5;
		}

		if (detector_uav_cmd.vel.linear.x < -0.5)
		{
			detector_uav_cmd.vel.linear.x = -0.5;
		}

		if (detector_uav_cmd.vel.linear.y > 0.5)
		{
			detector_uav_cmd.vel.linear.y = 0.5;
		}

		if (detector_uav_cmd.vel.linear.y < -0.5)
		{
			detector_uav_cmd.vel.linear.y = -0.5;
		}

		detector_uav_cmd.flag = flag;
		detector_vel_pub.publish(detector_uav_cmd);
	}

	last_time = current_time;
	Pos_err_ex(0) = Pos_err(0);
	Pos_err_ex(1) = Pos_err(1);
	Pos_err_ex(2) = Pos_err(2);
}
*/

void PID_control()
{
	if (id != -1 && land == 0)
		flag = 1;

	// 时间微分
	c_time = ros::Time::now();
	current_time = c_time.toSec();
	dt = current_time - last_time;

	if (dt > 0.001)
	{
		// 误差e
		// Pos_err = Pos_target - Relative_pos;
		Pos_err(0) = Pos_target(0) - Relative_pos(0);
		Pos_err(1) = Pos_target(1) - Relative_pos(1);
		Pos_err(2) = Pos_target(2) - Relative_pos(2);

		Pos_err_sum(0) += Pos_err(0);
		Pos_err_sum(1) += Pos_err(1);
		Pos_err_sum(2) += Pos_err(2);

		Pos_err_sum(0) = limit(Pos_err_sum(0), -20, 20);
		Pos_err_sum(1) = limit(Pos_err_sum(1), -20, 20);
		Pos_err_sum(2) = limit(Pos_err_sum(2), -0.5, 0.5);

		// 误差的微分ec
		Pos_derr(0) = (Pos_err(0) - Pos_err_ex(0)) / dt;
		Pos_derr(1) = (Pos_err(1) - Pos_err_ex(1)) / dt;
		Pos_derr(2) = (Pos_err(2) - Pos_err_ex(2)) / dt;

		vel_sp(0) = Output_vel_scale(0) * (Pos_x_pid(0) * Pos_err(0) + Pos_x_pid(1) * Pos_err_sum(0) + Pos_x_pid(2) * Pos_derr(0));
		vel_sp(1) = Output_vel_scale(1) * (Pos_y_pid(0) * Pos_err(1) + Pos_y_pid(1) * Pos_err_sum(1) + Pos_y_pid(2) * Pos_derr(1));
		vel_sp(2) = Output_vel_scale(2) * (Pos_z_pid(0) * Pos_err(2) + Pos_z_pid(1) * Pos_err_sum(2) + Pos_z_pid(2) * Pos_derr(2));

		vel_sp(0) = limit(vel_sp(0), -1.0, 1.0);
		vel_sp(1) = limit(vel_sp(1), -1.0, 1.0);
		vel_sp(2) = limit(vel_sp(2), -1.0, 1.0);

		// yaw控制 控制量为yaw_rate
		if (id != -1)
		{
			yaw_err = 0 - rpy.z;

			if (yaw_err < 0.5)
				yaw_err_sum += yaw_err;

			yaw_err_sum = limit(yaw_err_sum, -5, 5);
			yaw_derr = (yaw_err - yaw_err_ex) / dt;
			yaw_rate_sp = Yaw_pid(0) * yaw_err + Yaw_pid(1) * yaw_err_sum + Yaw_pid(2) * yaw_derr;
			yaw_rate_sp = limit(yaw_rate_sp, -0.5, 0.5);
		}
		else
			yaw_rate_sp = 0.0;

		if (land_flag == 1)
			landing_strategy(flag, status, Pos_err, land_num1, land_num2, land_time1, land_time2);

		if (flag == 0)
		{
			vel_sp(0) = 0.0;
			vel_sp(1) = 0.0;
			vel_sp(2) = 0.0;
		}

		detector_uav_cmd.vel.linear.x = -(vel_sp(0) * std::cos(Euler_fcu(2)) - vel_sp(1) * std::sin(Euler_fcu(2)));
		detector_uav_cmd.vel.linear.y = -(vel_sp(0) * std::sin(Euler_fcu(2)) + vel_sp(1) * std::cos(Euler_fcu(2)));
		detector_uav_cmd.vel.linear.z = -vel_sp(2);
		detector_uav_cmd.yaw_rate_ref = yaw_rate_sp;
		detector_uav_cmd.flag = flag;
		detector_uav_cmd.attitude_control_flag = false;
		detector_vel_pub.publish(detector_uav_cmd);
	}
	Pos_err_ex(0) = Pos_err(0);
	Pos_err_ex(1) = Pos_err(1);
	Pos_err_ex(2) = Pos_err(2);
	yaw_err_ex = yaw_err;
	last_time = current_time;
}

void landing_strategy(int &flag, int &status, Eigen::Vector3d Pos_err, int &land_num1, int &land_num2, int land_time1, int land_time2)
{
	if (flag == 1 && (-0.1) < Pos_err(0) && Pos_err(0) < 0.1 && (-0.1) < Pos_err(1) && Pos_err(1) < 0.1 && (-0.4) > Relative_pos(2) && Relative_pos(2) > (-1.0) && (id != -1) && status == 0)
	{
		land_num1++;
		if (land_num1 >= land_time1)
		{
			flag = 1;
			Pos_target(2) = -0.3;
			status = 1;
		}
	}

	if (status == 1 && Relative_pos(2) > (-0.3))
	{
		land_num2++;
		if (land_num2 >= land_time2)
		{
			flag = 3;
		}
	}
}

void PrintInfo()
{
	// ROS_INFO("******************Param******************");
	ROS_INFO("control_method:%d", control_method);
	// ROS_INFO("data_record:%d", data_record);
	// ROS_INFO("land_time1:%d", land_time1);
	// ROS_INFO("land_time2:%d", land_time2);
	ROS_INFO("flag:%d", detector_uav_cmd.flag);
	// ROS_INFO("status:%d", status);
	ROS_INFO("dt:%f", dt);
	// ROS_INFO("**************Absolute Pos**************");
	// ROS_INFO("Pos_x:%f", pos_drone_t265(0));
	// ROS_INFO("Pos_y:%f", pos_drone_t265(1));
	// ROS_INFO("Pos_z:%f", pos_drone_t265(2));
	// ROS_INFO("pos_drone_fcu(0):%f", pos_drone_fcu(0));
	// ROS_INFO("pos_drone_fcu(1):%f", pos_drone_fcu(1));
	// ROS_INFO("pos_drone_fcu(2):%f", pos_drone_fcu(2));
	ROS_INFO("************** Relative Pos**************");
	ROS_INFO("Relative_pos(0):%f", Relative_pos(0));
	ROS_INFO("Relative_pos(1):%f", Relative_pos(1));
	ROS_INFO("Relative_pos(2):%f", Relative_pos(2));
	ROS_INFO("Relative_dpos(0):%f", Relative_dpos(0));
	ROS_INFO("Relative_dpos(1):%f", Relative_dpos(1));
	ROS_INFO("Relative_dpos(2):%f", Relative_dpos(2));
	// ROS_INFO("******************Speed******************");
	// ROS_INFO("linear_vel_drone_fcu(0):%f", linear_vel_drone_fcu(0));
	// ROS_INFO("linear_vel_drone_fcu(1):%f", linear_vel_drone_fcu(1));
	// ROS_INFO("linear_vel_drone_fcu(2):%f", linear_vel_drone_fcu(2));
	// ROS_INFO("****************Speed Sp*****************");
	// ROS_INFO("vel_sp(0):%f", vel_sp(0));
	// ROS_INFO("vel_sp(1):%f", vel_sp(1));
	// ROS_INFO("vel_sp(2):%f", vel_sp(2));
	// ROS_INFO("************Detected Attitude************");
	//  ROS_INFO("detected roll:%f", rpy.x);
	//  ROS_INFO("sin roll:%f", sin(rpy.x));
	//  ROS_INFO("cos roll:%f", cos(rpy.x));
	//  ROS_INFO("detected pitch:%f", rpy.y);
	// ROS_INFO("detected yaw:%f", rpy.z);
	//  ROS_INFO("Eurler_fcu(0):%f",Euler_fcu(0));
	//  ROS_INFO("Eurler_fcu(1):%f",Euler_fcu(1));
	//  ROS_INFO("Eurler_fcu(2):%f",Euler_fcu(2));
	//   ROS_INFO("************Attitude Rate Sp*************");
	//   ROS_INFO("yaw_rate_sp:%f", yaw_rate_sp);
	ROS_INFO("**************** MPC ********************");
	ROS_INFO("ctr(0):%f", ctr(0));
	ROS_INFO("ctr(1):%f", ctr(1));
	ROS_INFO("ctr(2):%f", ctr(2));
	// ROS_INFO("dt:%f", dt);
	ROS_INFO("*************Attitude Control************");
	ROS_INFO("thrust_sp_normalize:%f", thrust_sp_normalize);
	ROS_INFO("roll_sp:%f", attitude_sp(0));
	ROS_INFO("pitch_sp:%f", attitude_sp(1));
	ROS_INFO("yaw_sp:%f", attitude_sp(2));
	ROS_INFO("*****************************************");
}

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
	// return limit(delay, 0.1, 0.2);
	return 0.0;
}

void detector_cb(const aruco_msgs::Detector::ConstPtr &msg)
{
	detect = *msg;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(detect.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	// rpy.x = roll;
	// rpy.y = pitch;
	// rpy.z = yaw;

	rpy.x = pitch;
	if (roll < 0)
	{
		rpy.y = roll + pi;
	}
	else if (roll > 0)
	{
		rpy.y = roll - pi;
	}
	else
	{
		rpy.y = roll;
	}
	rpy.y = -rpy.y;
	rpy.z = yaw;

	geometry_msgs::Quaternion quat_cam = tf::createQuaternionMsgFromRollPitchYaw(rpy.x, rpy.y, rpy.z);

	// ROS_INFO("======================================");
	// ROS_INFO("quat_cam.w:%f",quat_cam.w);
	// ROS_INFO("quat_cam.x:%f",quat_cam.x);
	// ROS_INFO("quat_cam.y:%f",quat_cam.y);
	// ROS_INFO("quat_cam.z:%f",quat_cam.z);
	// ROS_INFO("======================================");

	q_detect(0) = quat_cam.w;
	q_detect(1) = quat_cam.x;
	q_detect(2) = quat_cam.y;
	q_detect(3) = quat_cam.z;

	Relative_pos_ex(0) = Relative_pos(0);
	Relative_pos_ex(1) = Relative_pos(1);
	Relative_pos_ex(2) = Relative_pos(2);

	Relative_pos(0) = -detect.pose.position.y;
	Relative_pos(1) = detect.pose.position.x;
	Relative_pos(2) = -detect.pose.position.z;

	// 识别丢失情况
	if (Relative_pos(1) == -1.0)
	{
		Relative_pos(0) = Relative_pos_ex(0);
		Relative_pos(1) = Relative_pos_ex(1);
		Relative_pos(2) = Relative_pos_ex(2);
	}
	last_id = id;
	id = detect.id;
	num_all++;

	if (id == -1)
	{
		rpy.x = 0;
		rpy.y = 0;
		rpy.z = 0;
	}
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
	Eigen::Vector3d linear_vel_drone_fcu_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
	Eigen::Vector3d angular_vel_drone_fcu_enu(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

	linear_vel_drone_fcu = linear_vel_drone_fcu_enu;
	angular_vel_drone_fcu = angular_vel_drone_fcu_enu;
}
/*
void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	if (msg->header.frame_id == "camera_odom_frame")
	{
		pos_drone_t265(0) = msg->pose.pose.position.x;
		pos_drone_t265(1) = msg->pose.pose.position.y;
		// pos_drone_t265(2) = msg->pose.pose.position.z;
		pos_drone_t265(2) = pos_drone_fcu(2);
		Eigen::Quaterniond q_t265_enu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
		q_t265 = q_t265_enu;
		Euler_t265 = quaternion_to_euler(q_t265);
	}
}
*/

void platform_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	q_ugv(0) = msg->pose.pose.orientation.w;
	q_ugv(1) = msg->pose.pose.orientation.x;
	q_ugv(2) = msg->pose.pose.orientation.y;
	q_ugv(3) = msg->pose.pose.orientation.z;

	vel_ugv(0) = msg->twist.twist.linear.x;
	vel_ugv(1) = msg->twist.twist.linear.y;
	vel_ugv(2) = msg->twist.twist.linear.z;

	omega_ugv(0) = msg->twist.twist.angular.x;
	omega_ugv(1) = msg->twist.twist.angular.y;
	omega_ugv(2) = msg->twist.twist.angular.z;
}
