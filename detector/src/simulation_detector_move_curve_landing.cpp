#include "simulation_detector_move_curve_landing.h"

using namespace std;

detectorNode::detectorNode()
{
	nh.param<int>("BasicParam/control_method", control_method, 0);
	nh.param<int>("BasicParam/data_record", data_record, 1);
	nh.param<int>("BasicParam/land_flag", land_flag, 0);
	nh.param<int>("BasicParam/delay_flag", delay_flag, 0);
	nh.param<double>("ControlParam/target_z", Pos_target(2), 0.8);
	nh.param<int>("ControlParam/land_time1", land_time1, 160);
	nh.param<int>("ControlParam/land_time2", land_time2, 30);
	nh.param<double>("UAVParam/hover_throttle", hover_throttle, 0.45);
	nh.param<double>("UAVParam/mass", mass, 1.5);

	if (control_method == 0)
	{
		nh.param<int>("NmpcParam/Np", Np, 20);
		nh.param<int>("NmpcParam/Nc", Nc, 20);
		nh.param<double>("NmpcParam/w_P_xy", w_P_xy, 0);
		nh.param<double>("NmpcParam/w_P_z", w_P_z, 0);
		nh.param<double>("NmpcParam/w_P_xy_sum", w_P_xy_sum, 0);
		nh.param<double>("NmpcParam/w_P_z_sum", w_P_z_sum, 0);
		nh.param<double>("NmpcParam/w_V_xy", w_V_xy, 0);
		nh.param<double>("NmpcParam/w_V_z", w_V_z, 0);
		nh.param<double>("NmpcParam/w_q", w_q, 0);
		nh.param<double>("NmpcParam/w_P_xy_n", w_P_xy_n, 0);
		nh.param<double>("NmpcParam/w_P_z_n", w_P_z_n, 0);
		nh.param<double>("NmpcParam/w_P_xy_sum_n", w_P_xy_sum_n, 0);
		nh.param<double>("NmpcParam/w_P_z_sum_n", w_P_z_sum_n, 0);
		nh.param<double>("NmpcParam/w_V_xy_n", w_V_xy_n, 0);
		nh.param<double>("NmpcParam/w_V_z_n", w_V_z_n, 0);
		nh.param<double>("NmpcParam/w_q_n", w_q_n, 0);
		nh.param<double>("NmpcParam/w_T", w_T, 0);
		nh.param<double>("NmpcParam/w_omega_x", w_omega_x, 0);
		nh.param<double>("NmpcParam/w_omega_y", w_omega_y, 0);
		nh.param<double>("NmpcParam/w_omega_z", w_omega_z, 0);
	}
	else if (control_method == 1)
	{
		nh.param<int>("LmpcParam/Np", Np, 20);
		nh.param<int>("LmpcParam/Nc", Nc, 20);
		nh.param<double>("LmpcParam/w_P_xy", w_P_xy, 0);
		nh.param<double>("LmpcParam/w_P_z", w_P_z, 0);
		nh.param<double>("LmpcParam/w_P_xy_sum", w_P_xy_sum, 0);
		nh.param<double>("LmpcParam/w_P_z_sum", w_P_z_sum, 0);
		nh.param<double>("LmpcParam/w_V_xy", w_V_xy, 0);
		nh.param<double>("LmpcParam/w_V_z", w_V_z, 0);
		nh.param<double>("LmpcParam/w_P_xy_n", w_P_xy_n, 0);
		nh.param<double>("LmpcParam/w_P_z_n", w_P_z_n, 0);
		nh.param<double>("LmpcParam/w_P_xy_sum_n", w_P_xy_sum_n, 0);
		nh.param<double>("LmpcParam/w_P_z_sum_n", w_P_z_sum_n, 0);
		nh.param<double>("LmpcParam/w_V_xy_n", w_V_xy_n, 0);
		nh.param<double>("LmpcParam/w_V_z_n", w_V_z_n, 0);
		nh.param<double>("LmpcParam/w_U_x", w_U_x, 0);
		nh.param<double>("LmpcParam/w_U_y", w_U_y, 0);
		nh.param<double>("LmpcParam/w_U_z", w_U_z, 0);
	}

	control_loop_timer = nh.createTimer(ros::Duration(0.02), &detectorNode::control_loop_timer_cb, this);

	detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 50, &detectorNode::detector_cb, this, ros::TransportHints().tcpNoDelay(true));

	detector_cmd_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);

	position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &detectorNode::pos_cb, this);
	velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &detectorNode::vel_cb, this);
	platform_odom_sub = nh.subscribe<nav_msgs::Odometry>("/moving_platform/wheelcontrol/odom", 10, &detectorNode::platform_odom_cb, this);

	pub_cmd_to_delayNode = nh.advertise<aruco_msgs::Detector_vel>("/forward_channel", 50);
	sub_cmd_from_delayNode = nh.subscribe<uav_msgs::feedback_delay>("/feedback_channel", 50, &detectorNode::uav_feedback_cb, this);
	//  ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1000, t265_cb);
	//  ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 25);

	if (control_method == 0)
	{
		mpc_params["mass"] = mass;
		mpc_params["Np"] = Np;

		mpc_params["w_P_xy"] = w_P_xy;
		mpc_params["w_P_z"] = w_P_z;
		mpc_params["w_P_xy_sum"] = w_P_xy_sum;
		mpc_params["w_P_z_sum"] = w_P_z_sum;
		mpc_params["w_V_xy"] = w_V_xy;
		mpc_params["w_V_z"] = w_V_z;
		mpc_params["w_q"] = w_q;

		mpc_params["w_P_xy_n"] = w_P_xy_n;
		mpc_params["w_P_z_n"] = w_P_z_n;
		mpc_params["w_P_xy_sum_n"] = w_P_xy_sum_n;
		mpc_params["w_P_z_sum_n"] = w_P_z_sum_n;
		mpc_params["w_V_xy_n"] = w_V_xy_n;
		mpc_params["w_V_z_n"] = w_V_z_n;
		mpc_params["w_q_n"] = w_q_n;

		mpc_params["w_T"] = w_T;
		mpc_params["w_omega_x"] = w_omega_x;
		mpc_params["w_omega_y"] = w_omega_y;
		mpc_params["w_omega_z"] = w_omega_z;

		mpc.reset(new MPC("NMPC"));
		mpc->LoadParams(mpc_params);
		mpc->setNmpcSolver();
		cout << "\033[1m\033[44;37m ----------> NMPC solver set! <---------- \033[0m" << endl;
	}
	else if (control_method == 1)
	{
		mpc_params["mass"] = mass;
		mpc_params["Np"] = Np;

		mpc_params["w_P_xy"] = w_P_xy;
		mpc_params["w_P_z"] = w_P_z;
		mpc_params["w_P_xy_sum"] = w_P_xy_sum;
		mpc_params["w_P_z_sum"] = w_P_z_sum;
		mpc_params["w_V_xy"] = w_V_xy;
		mpc_params["w_V_z"] = w_V_z;
		mpc_params["w_P_xy_n"] = w_P_xy_n;
		mpc_params["w_P_z_n"] = w_P_z_n;
		mpc_params["w_P_xy_sum_n"] = w_P_xy_sum_n;
		mpc_params["w_P_z_sum_n"] = w_P_z_sum_n;
		mpc_params["w_V_xy_n"] = w_V_xy_n;
		mpc_params["w_V_z_n"] = w_V_z_n;

		mpc_params["w_U_x"] = w_U_x;
		mpc_params["w_U_y"] = w_U_y;
		mpc_params["w_U_z"] = w_U_z;

		mpc.reset(new MPC("LMPC"));
		mpc->LoadParams(mpc_params);
		mpc->setLmpcSolver();
		cout << "\033[1m\033[44;37m ----------> LMPC solver set! <---------- \033[0m" << endl;
	}
}

void detectorNode::control_loop_timer_cb(const ros::TimerEvent &event)
{
	if (data_record == 1 && file_created == 0)
	{
		char filename[100] = {0};
		time_t time_now = time(NULL);
		tm *p = localtime(&time_now);
		sprintf(filename, "/home/yx/experiment_record/data_record/Data %d-%d-%d %d:%02d.csv", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min);
		oFile.open(filename, ios::out | ios::app);
		file_created = 1;
		if (control_method == 0)
		{
			oFile << "time"
				  << ","
				  << "id"
				  << ","
				  << "flag"
				  << ","
				  << "status"
				  << ","
				  << "P_x"
				  << ","
				  << "P_y"
				  << ","
				  << "P_z"
				  << ","
				  << "error_z"
				  << ","
				  << "V_x"
				  << ","
				  << "V_y"
				  << ","
				  << "V_z"
				  << ","
				  << "roll"
				  << ","
				  << "pitch"
				  << ","
				  << "yaw"
				  << ","
				  << "T"
				  << ","
				  << "omega_x"
				  << ","
				  << "omega_y"
				  << ","
				  << "omega_z"
				  << ","
				  << "solve time"
				  << endl;
		}
		else if (control_method == 1)
		{
			oFile << "time"
				  << ","
				  << "id"
				  << ","
				  << "flag"
				  << ","
				  << "status"
				  << ","
				  << "P_x"
				  << ","
				  << "P_y"
				  << ","
				  << "P_z"
				  << ","
				  << "error_z"
				  << ","
				  << "V_x"
				  << ","
				  << "V_y"
				  << ","
				  << "V_z"
				  << ","
				  << "roll"
				  << ","
				  << "pitch"
				  << ","
				  << "yaw"
				  << ","
				  << "U_x"
				  << ","
				  << "U_y"
				  << ","
				  << "U_z"
				  << ","
				  << "solve time"
				  << endl;
		}
	}

	switch (control_method)
	{
		// NMPC
	case 0:
		NMPC_control();
		break;

	// LMPC
	case 1:
		LMPC_control();
		break;
	default:
		break;
	}
	// PrintInfo();
}

void detectorNode::NMPC_control()
{

	if (id != -1 && land == 0)
	{
		flag = 1;
	}

	current_time = ros::Time::now().toSec();
	dt = current_time - last_time;

	if (last_flag == 1)
	{
		Relative_dpos(0) = (Relative_pos(0) - Relative_pos_ex(0)) / dt;
		Relative_dpos(1) = (Relative_pos(1) - Relative_pos_ex(1)) / dt;
		Relative_dpos(2) = (Relative_pos(2) - Relative_pos_ex(2)) / dt;

		Relative_pos_sum(0) += Relative_pos(0);
		Relative_pos_sum(1) += Relative_pos(1);
		Relative_pos_sum(2) += Relative_pos(2);

		Relative_pos_sum(0) = limit(Relative_pos_sum(0), -5.0, 5.0);
		Relative_pos_sum(1) = limit(Relative_pos_sum(1), -5.0, 5.0);
		Relative_pos_sum(2) = limit(Relative_pos_sum(2), -5.0, 5.0);

		accel_ugv(0) = (vel_ugv(0) - vel_ugv_ex(0)) / dt;
		accel_ugv(1) = (vel_ugv(1) - vel_ugv_ex(1)) / dt;
		accel_ugv(2) = (vel_ugv(2) - vel_ugv_ex(2)) / dt;

		accel_ugv_w = quat_VectorMultiplication(q_ugv, accel_ugv);

		alpha_ugv(0) = (omega_ugv(0) - omega_ugv_ex(0)) / dt;
		alpha_ugv(1) = (omega_ugv(1) - omega_ugv_ex(1)) / dt;
		alpha_ugv(2) = (omega_ugv(2) - omega_ugv_ex(2)) / dt;

		if (q_detect(0) < 0.0)
		{
			q_detect(0) = -q_detect(0);
			q_detect(1) = -q_detect(1);
			q_detect(2) = -q_detect(2);
			q_detect(3) = -q_detect(3);
		}

		Nmpc_x0 << Relative_pos(0), Relative_pos(1), Relative_pos(2), Relative_pos_sum(0), Relative_pos_sum(1), Relative_pos_sum(2), Relative_dpos(0), Relative_dpos(1), Relative_dpos(2), q_detect(0), q_detect(1), q_detect(2), q_detect(3);

		Nmpc_xRef << Pos_target(0), Pos_target(1), Pos_target(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		// cout << "Relative_pos_sum: " << Relative_pos_sum.transpose() << endl;
		cout << "x0: " << Nmpc_x0.transpose() << endl;
		cout << endl;

		Pos_err(0) = Nmpc_xRef(0) - Nmpc_x0(0);
		Pos_err(1) = Nmpc_xRef(1) - Nmpc_x0(1);
		Pos_err(2) = Nmpc_xRef(2) - Nmpc_x0(2);

		//    高亮 蓝色背景 白色字  关闭所有
		// \033[1m\033[44;37m  \033[0m

		if (land_flag)
		{
			landing_strategy(flag, status, Pos_err, land_num1, land_num2, land_time1, land_time2);
		}
	}

	thrust_sp_normalize = hover_throttle;

	if (last_flag == 1)
	{
		auto solve_start = ros::Time::now();
		if (delay_flag)
		{
			// Nmpc_predicted_x0 = Nmpc_one_step_predictor(Nmpc_x0, est_delay, q_ugv, omega_ugv, alpha_ugv, accel_ugv_w, last_thrust_sp, last_rate_sp);
			Nmpc_predicted_x0 = Nmpc_rk4_predictor(Nmpc_x0, est_delay, q_ugv, omega_ugv, alpha_ugv, accel_ugv_w, last_thrust_sp, last_rate_sp);
			cout << "\033[1m\033[44;37m RK4! \033[0m" << endl;

			mpc->Nmpc_solve(Nmpc_predicted_x0, Nmpc_xRef, q_ugv, omega_ugv, alpha_ugv, accel_ugv_w);

			// mpc->Nmpc_solve(Nmpc_x0, Nmpc_xRef, q_ugv, omega_ugv, alpha_ugv, accel_ugv_w);
			// cout << "\033[1m\033[44;37m Uncompensated! \033[0m" << endl;
		}
		else
		{
			mpc->Nmpc_solve(Nmpc_x0, Nmpc_xRef, q_ugv, omega_ugv, alpha_ugv, accel_ugv_w);
		}
		auto solve_end = ros::Time::now();
		solve_time = (solve_end - solve_start).toSec();

		auto mpc_result = mpc->Nmpc_getFirstU();

		thrust_sp = mpc_result[0];
		thrust_sp_normalize = thrust_sp * hover_throttle / 9.8;
		thrust_sp_normalize = limit(thrust_sp_normalize, 0.25, 0.6);

		rate_sp(0) = mpc_result[1];
		rate_sp(1) = mpc_result[2];
		rate_sp(2) = mpc_result[3];
	}

	detector_uav_cmd.timestamp = ros::Time::now().toSec();
	detector_uav_cmd.flag = flag;
	detector_uav_cmd.attitude_control_flag = true;
	detector_uav_cmd.attitude_control_type_mask = 0b10000000; // 忽略ATTITUDE

	detector_uav_cmd.thrust = thrust_sp_normalize;
	detector_uav_cmd.body_rate.x = rate_sp(0);
	detector_uav_cmd.body_rate.y = rate_sp(1);
	detector_uav_cmd.body_rate.z = rate_sp(2);

	if (!delay_flag)
	{
		detector_cmd_pub.publish(detector_uav_cmd);
	}
	else
	{
		pub_cmd_to_delayNode.publish(detector_uav_cmd);
	}

	last_flag = flag;
	last_time = current_time;

	Relative_pos_ex(0) = Relative_pos(0);
	Relative_pos_ex(1) = Relative_pos(1);
	Relative_pos_ex(2) = Relative_pos(2);

	q_detect_ex(0) = q_detect(0);
	q_detect_ex(1) = q_detect(1);
	q_detect_ex(2) = q_detect(2);
	q_detect_ex(3) = q_detect(3);

	vel_ugv_ex(0) = vel_ugv(0);
	vel_ugv_ex(1) = vel_ugv(1);
	vel_ugv_ex(2) = vel_ugv(2);

	omega_ugv_ex(0) = omega_ugv(0);
	omega_ugv_ex(1) = omega_ugv(1);
	omega_ugv_ex(2) = omega_ugv(2);

	last_thrust_sp = thrust_sp;
	last_rate_sp(0) = rate_sp(0);
	last_rate_sp(1) = rate_sp(1);
	last_rate_sp(2) = rate_sp(2);

	if (file_created == 1 && data_record == 1)
	{
		oFile << current_time << "," << id << "," << flag << "," << status << "," << Relative_pos(0) << "," << Relative_pos(1) << "," << Relative_pos(2) << "," << Pos_target(2) - Relative_pos(2) << "," << Relative_dpos(0) << "," << Relative_dpos(1) << "," << Relative_dpos(2) << "," << rpy.x << "," << rpy.y << "," << rpy.z << "," << thrust_sp / mass << "," << rate_sp(0) << "," << rate_sp(1) << "," << rate_sp(2) << "," << solve_time << endl;
	}
}

void detectorNode::LMPC_control()
{
	if (id != -1 && land == 0)
	{
		flag = 1;
	}

	current_time = ros::Time::now().toSec();
	dt = current_time - last_time;

	if (last_flag == 1)
	{
		Relative_dpos(0) = (Relative_pos(0) - Relative_pos_ex(0)) / dt;
		Relative_dpos(1) = (Relative_pos(1) - Relative_pos_ex(1)) / dt;
		Relative_dpos(2) = (Relative_pos(2) - Relative_pos_ex(2)) / dt;

		Relative_pos_sum(0) += Relative_pos(0);
		Relative_pos_sum(1) += Relative_pos(1);
		Relative_pos_sum(2) += Relative_pos(2);

		Relative_pos_sum(0) = limit(Relative_pos_sum(0), -5.0, 5.0);
		Relative_pos_sum(1) = limit(Relative_pos_sum(1), -5.0, 5.0);
		Relative_pos_sum(2) = limit(Relative_pos_sum(2), -5.0, 5.0);

		accel_ugv(0) = (vel_ugv(0) - vel_ugv_ex(0)) / dt;
		accel_ugv(1) = (vel_ugv(1) - vel_ugv_ex(1)) / dt;
		accel_ugv(2) = (vel_ugv(2) - vel_ugv_ex(2)) / dt;

		Lmpc_x0 << Relative_pos(0), Relative_pos(1), Relative_pos(2), Relative_pos_sum(0), Relative_pos_sum(1), Relative_pos_sum(2), Relative_dpos(0), Relative_dpos(1), Relative_dpos(2);
		Lmpc_xRef << Pos_target(0), Pos_target(1), Pos_target(2), 0, 0, 0, 0, 0, 0;

		cout << "x0: " << Lmpc_x0.transpose() << endl;
		cout << endl;

		Pos_err(0) = Lmpc_xRef(0) - Lmpc_x0(0);
		Pos_err(1) = Lmpc_xRef(1) - Lmpc_x0(1);
		Pos_err(2) = Lmpc_xRef(2) - Lmpc_x0(2);

		//    高亮 蓝色背景 白色字  关闭所有
		// \033[1m\033[44;37m  \033[0m
		if (land_flag)
		{
			landing_strategy(flag, status, Pos_err, land_num1, land_num2, land_time1, land_time2);
		}
	}

	thrust_sp_normalize = hover_throttle;

	if (last_flag == 1)
	{
		auto solve_start = ros::Time::now();
		if (delay_flag)
		{
			// Lmpc_predicted_x0 = Lmpc_one_step_predictor(Lmpc_x0, est_delay, last_Relative_accel_sp);
			Lmpc_predicted_x0 = Lmpc_rk4_predictor(Lmpc_x0, est_delay, last_Relative_accel_sp);
			cout << "\033[1m\033[44;37m RK4! \033[0m" << endl;
			mpc->Lmpc_solve(Lmpc_predicted_x0, Lmpc_xRef);

			// mpc->Lmpc_solve(Lmpc_x0, Lmpc_xRef);
			// cout << "\033[1m\033[44;37m Uncompensated! \033[0m" << endl;
		}
		else
		{
			mpc->Lmpc_solve(Lmpc_x0, Lmpc_xRef);
		}
		auto solve_end = ros::Time::now();
		solve_time = (solve_end - solve_start).toSec();
		// cout << "solve time: " << solve_time << endl;

		auto mpc_result = mpc->Lmpc_getFirstU();
		// cout << "mpc_result: " << mpc_result << endl;

		Relative_accel_sp(0) = mpc_result[0];
		Relative_accel_sp(1) = mpc_result[1];
		Relative_accel_sp(2) = mpc_result[2];

		// accel_sp(0) = ctr(0);
		// accel_sp(1) = ctr(1);
		accel_sp(2) = 9.8 + mpc_result[2];

		// 用ugv的姿态和检测的相对姿态来代替 Euler_fcu
		/*
		cout << "Euler_fcu(0): " << Euler_fcu(0) << endl;
		cout << "Euler_fcu(1): " << Euler_fcu(1) << endl;
		cout << "Euler_fcu(2): " << Euler_fcu(2) << endl;
		cout << endl;
		cout << "Euler_ugv(0) + rpy.x: " << Euler_ugv(0) + rpy.x << endl;
		cout << "Euler_ugv(1) + rpy.y: " << Euler_ugv(1) + rpy.y << endl;
		cout << "Euler_ugv(2) + rpy.z: " << Euler_ugv(2) + rpy.z << endl;
		cout << endl;
				*/

		accel_sp(0) = mpc_result[0] * std::cos(Euler_ugv(2) + rpy.z) - mpc_result[1] * std::sin(Euler_ugv(2) + rpy.z);
		accel_sp(1) = mpc_result[0] * std::sin(Euler_ugv(2) + rpy.z) + mpc_result[1] * std::cos(Euler_ugv(2) + rpy.z);

		thrust_sp = accel_sp(2) / (std::cos(Euler_ugv(0) + rpy.y) * std::cos(Euler_ugv(1) + rpy.x));

		thrust_sp_normalize = thrust_sp * hover_throttle / 9.8;
		thrust_sp_normalize = limit(thrust_sp_normalize, 0.25, 0.6);

		// attitude_sp(2) = attitude_sp(2) + 0.5 * (0 - rpy.z);
		attitude_sp(2) = Euler_ugv(2) + rpy.z - 2 * rpy.z;
		attitude_sp(0) = std::asin((accel_sp(0) * std::sin(Euler_ugv(2) + rpy.z) - accel_sp(1) * std::cos(Euler_ugv(2) + rpy.z)) / thrust_sp);
		attitude_sp(1) = std::atan((accel_sp(0) * std::cos(Euler_ugv(2) + rpy.z) + accel_sp(1) * std::sin(Euler_ugv(2) + rpy.z)) / accel_sp(2));

		q_sp = Eigen::AngleAxisd(attitude_sp(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(attitude_sp(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(attitude_sp(0), Eigen::Vector3d::UnitX());
	}

	detector_uav_cmd.timestamp = ros::Time::now().toSec();
	detector_uav_cmd.flag = flag;
	detector_uav_cmd.attitude_control_flag = true;
	detector_uav_cmd.attitude_control_type_mask = 0b00000111;

	detector_uav_cmd.thrust = thrust_sp_normalize;
	detector_uav_cmd.quat.w = q_sp.w();
	detector_uav_cmd.quat.x = q_sp.x();
	detector_uav_cmd.quat.y = q_sp.y();
	detector_uav_cmd.quat.z = q_sp.z();

	if (!delay_flag)
	{
		detector_cmd_pub.publish(detector_uav_cmd);
	}
	else
	{
		pub_cmd_to_delayNode.publish(detector_uav_cmd);
	}

	last_flag = flag;
	last_time = current_time;

	Relative_pos_ex(0) = Relative_pos(0);
	Relative_pos_ex(1) = Relative_pos(1);
	Relative_pos_ex(2) = Relative_pos(2);

	vel_ugv_ex(0) = vel_ugv(0);
	vel_ugv_ex(1) = vel_ugv(1);
	vel_ugv_ex(2) = vel_ugv(2);

	last_Relative_accel_sp(0) = Relative_accel_sp(0);
	last_Relative_accel_sp(1) = Relative_accel_sp(1);
	last_Relative_accel_sp(2) = Relative_accel_sp(2);

	if (file_created == 1 && data_record == 1)
	{
		oFile << current_time << "," << id << "," << flag << "," << status << "," << Relative_pos(0) << "," << Relative_pos(1) << "," << Relative_pos(2) << "," << Pos_target(2) - Relative_pos(2) << "," << Relative_dpos(0) << "," << Relative_dpos(1) << "," << Relative_dpos(2) << "," << rpy.x << "," << rpy.y << "," << rpy.z << "," << Relative_accel_sp(0) << "," << Relative_accel_sp(1) << "," << Relative_accel_sp(2) << "," << solve_time << endl;
	}
}

void landing_strategy(int &flag, int &status, Eigen::Vector3d Pos_err, int &land_num1, int &land_num2, int land_time1, int land_time2)
{
	if (flag == 1 && in_range(Pos_err(0), -0.1, 0.1) && in_range(Pos_err(1), -0.1, 0.1) && in_range(Relative_pos(2), 0.4, 1.0) && (id != -1) && status == 0)
	{
		land_num1++;
		if (land_num1 >= land_time1)
		{
			flag = 1;
			Pos_target(2) = 0.4;
			status = 1;
		}
	}

	if (status == 1 && Relative_pos(2) <= 0.5)
	{
		land_num2++;
		if (land_num2 >= land_time2)
		{
			flag = 3;
			status = 2;
		}
	}
}

void detectorNode::PrintInfo()
{
	// ROS_INFO("******************Param******************");
	/// ROS_INFO("control_method:%d", control_method);
	// ROS_INFO("data_record:%d", data_record);
	// ROS_INFO("land_time1:%d", land_time1);
	// ROS_INFO("land_time2:%d", land_time2);
	/// ROS_INFO("flag:%d", detector_uav_cmd.flag);
	// ROS_INFO("status:%d", status);
	/// ROS_INFO("dt:%f", dt);
	// ROS_INFO("**************Absolute Pos**************");
	// ROS_INFO("Pos_x:%f", pos_drone_t265(0));
	// ROS_INFO("Pos_y:%f", pos_drone_t265(1));
	// ROS_INFO("Pos_z:%f", pos_drone_t265(2));
	// ROS_INFO("pos_drone_fcu(0):%f", pos_drone_fcu(0));
	// ROS_INFO("pos_drone_fcu(1):%f", pos_drone_fcu(1));
	// ROS_INFO("pos_drone_fcu(2):%f", pos_drone_fcu(2));
	ROS_INFO("************ Relative Pos/vel ************");
	ROS_INFO("Relative_pos(0):%f", Relative_pos(0));
	ROS_INFO("Relative_pos(1):%f", Relative_pos(1));
	ROS_INFO("Relative_pos(2):%f", Relative_pos(2));
	ROS_INFO("Relative_dpos(0):%f", Relative_dpos(0));
	ROS_INFO("Relative_dpos(1):%f", Relative_dpos(1));
	ROS_INFO("Relative_dpos(2):%f", Relative_dpos(2));
	//  ROS_INFO("Euler_fcu(0):%f",Euler_fcu(0));
	//  ROS_INFO("Euler_fcu(1):%f",Euler_fcu(1));
	//  ROS_INFO("Euler_fcu(2):%f",Euler_fcu(2));
	//   ROS_INFO("************Attitude Rate Sp*************");
	//   ROS_INFO("yaw_rate_sp:%f", yaw_rate_sp);
	ROS_INFO("**************** MPC ********************");
	ROS_INFO("body_rate_x:%f(rad/s)", rate_sp(0));
	ROS_INFO("body_rate_y:%f(rad/s)", rate_sp(1));
	ROS_INFO("body_rate_z:%f(rad/s)", rate_sp(2));
	//   ROS_INFO("dt:%f", dt);
	/// ROS_INFO("*************Attitude Control************");
	ROS_INFO("thrust_sp_normalize:%f", thrust_sp_normalize);
	ROS_INFO("*****************************************");
}

void detectorNode::detector_cb(const aruco_msgs::Detector::ConstPtr &msg)
{
	detect = *msg;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(detect.pose.orientation, quat);
	double roll, pitch, yaw;

	// tf::Quaternion quat_b2d(-1.0, 0.0, 0.0, 0.0);
	// tf::Quaternion quat_d2w(0.0, 0.0, -0.7071068, 0.7071068);
	// quat_w = quat_d2w * quat * quat_b2d * quat_d2w.inverse();

	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	rpy.x = pitch;
	rpy.y = pi - roll;
	rpy.z = yaw;

	// 未识别
	if (last_id == -1 && id == -1)
	{
		rpy.x = 0;
		rpy.y = 0;
		rpy.z = 0;
	}

	geometry_msgs::Quaternion quat_cam = tf::createQuaternionMsgFromRollPitchYaw(rpy.x, rpy.y, rpy.z);

	q_detect(0) = quat_cam.w;
	q_detect(1) = quat_cam.x;
	q_detect(2) = quat_cam.y;
	q_detect(3) = quat_cam.z;

	Relative_pos(0) = detect.pose.position.y;  //- => +
	Relative_pos(1) = -detect.pose.position.x; //+ => -
	Relative_pos(2) = detect.pose.position.z;

	// 识别丢失情况
	if (last_id != -1 && id == -1)
	{
		marker_lost = true;
	}

	if (marker_lost && last_id == -1 && id != -1)
	{
		marker_lost = false;
	}

	if (marker_lost)
	{
		Relative_pos(0) = Relative_pos_ex(0);
		Relative_pos(1) = Relative_pos_ex(1);
		Relative_pos(2) = Relative_pos_ex(2);

		q_detect(0) = q_detect_ex(0);
		q_detect(1) = q_detect_ex(1);
		q_detect(2) = q_detect_ex(2);
		q_detect(3) = q_detect_ex(3);
	}

	/*
	if (Relative_pos(1) == -1.0)
	{
		Relative_pos(0) = Relative_pos_ex(0);
		Relative_pos(1) = Relative_pos_ex(1);
		Relative_pos(2) = Relative_pos_ex(2);
	}
	*/

	last_id = id;
	id = detect.id;
	num_all++;
}

void detectorNode::uav_feedback_cb(const uav_msgs::feedback_delay::ConstPtr &msg)
{
	receive_timestamp = ros::Time::now().toSec();
	uav_feedback_msg = *msg;
	forward_delay = msg->forward_delay;
	feedback_delay = receive_timestamp - msg->timestamp;
	est_delay = 0.5 * (forward_delay + feedback_delay);

	cout << "forward_delay: " << forward_delay << " ,feedback_delay: " << feedback_delay << endl;
	cout << "est_delay: " << est_delay << endl;
	cout << endl;
}

void detectorNode::platform_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	q_ugv(0) = msg->pose.pose.orientation.w;
	q_ugv(1) = msg->pose.pose.orientation.x;
	q_ugv(2) = msg->pose.pose.orientation.y;
	q_ugv(3) = msg->pose.pose.orientation.z;
	Euler_ugv = quaternion_to_euler(q_ugv);

	vel_ugv(0) = msg->twist.twist.linear.x;
	vel_ugv(1) = msg->twist.twist.linear.y;
	vel_ugv(2) = msg->twist.twist.linear.z;

	omega_ugv(0) = msg->twist.twist.angular.x;
	omega_ugv(1) = msg->twist.twist.angular.y;
	omega_ugv(2) = msg->twist.twist.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulation_detector_node");

	detectorNode *node = new detectorNode();

	ros::spin();
	return 0;
}