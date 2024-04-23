#include "simulation_detector.h"

using namespace std;

detectorNode::detectorNode()
{
	nh.param<int>("BasicParam/control_method", control_method, 0);
	nh.param<int>("BasicParam/data_record", data_record, 1);
	nh.param<int>("BasicParam/land_flag", land_flag, 0);
	nh.param<double>("ControlParam/target_z", Pos_target(2), 0.8);
	nh.param<int>("ControlParam/land_time1", land_time1, 160);
	nh.param<int>("ControlParam/land_time2", land_time2, 30);
	nh.param<double>("ControlParam/yaw_kp", Yaw_pid(0), 0.5);
	nh.param<double>("ControlParam/yaw_ki", Yaw_pid(1), 0.0);
	nh.param<double>("ControlParam/yaw_kd", Yaw_pid(2), 0.0);
	nh.param<double>("UAVParam/hover_throttle", hover_throttle, 0.45);
	nh.param<double>("UAVParam/mass", mass, 1.5);
	nh.param<double>("UAVParam/arm_length", arm_length, 0.225);
	nh.param<double>("UAVParam/max_tilt", max_tilt, pi / 9);

	nh.param<int>("MpcParam/Np", Np, 20);
	nh.param<int>("MpcParam/Nc", Nc, 20);

	nh.param<double>("MpcParam/w_P_xy", w_P_xy, 0);
	nh.param<double>("MpcParam/w_P_z", w_P_z, 0);
	nh.param<double>("MpcParam/w_V_xy", w_V_xy, 0);
	nh.param<double>("MpcParam/w_V_z", w_V_z, 0);
	nh.param<double>("MpcParam/w_q", w_q, 0);
	nh.param<double>("MpcParam/w_P_xy_n", w_P_xy_n, 0);
	nh.param<double>("MpcParam/w_P_z_n", w_P_z_n, 0);
	nh.param<double>("MpcParam/w_V_xy_n", w_V_xy_n, 0);
	nh.param<double>("MpcParam/w_V_z_n", w_V_z_n, 0);
	nh.param<double>("MpcParam/w_q_n", w_q_n, 0);
	nh.param<double>("MpcParam/w_T", w_T, 0);
	nh.param<double>("MpcParam/w_omega_x", w_omega_x, 0);
	nh.param<double>("MpcParam/w_omega_y", w_omega_y, 0);
	nh.param<double>("MpcParam/w_omega_z", w_omega_z, 0);
	nh.param<double>("MpcParam/w_U_x", w_U_x, 0);
	nh.param<double>("MpcParam/w_U_y", w_U_y, 0);
	nh.param<double>("MpcParam/w_U_z", w_U_z, 0);

	control_loop_timer = nh.createTimer(ros::Duration(0.02), &detectorNode::control_loop_timer_cb, this);

	detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 50, &detectorNode::detector_cb, this, ros::TransportHints().tcpNoDelay(true));

	detector_vel_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);
	control_squence_pub = nh.advertise<aruco_msgs::Control_sequence>("/aruco_single/control_squence", 25);

	position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &detectorNode::pos_cb, this);
	velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &detectorNode::vel_cb, this);
	platform_odom_sub = nh.subscribe<nav_msgs::Odometry>("/moving_platform/wheelcontrol/odom", 10, &detectorNode::platform_odom_cb, this);
	// ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1000, t265_cb);
	// ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 25);

	mpc_params["mass"] = mass;
	mpc_params["Np"] = Np;
	mpc_params["w_P_xy"] = w_P_xy;
	mpc_params["w_P_z"] = w_P_z;
	mpc_params["w_V_xy"] = w_V_xy;
	mpc_params["w_V_z"] = w_V_z;
	mpc_params["w_q"] = w_q;
	mpc_params["w_P_xy_n"] = w_P_xy_n;
	mpc_params["w_P_z_n"] = w_P_z_n;
	mpc_params["w_V_xy_n"] = w_V_xy_n;
	mpc_params["w_V_z_n"] = w_V_z_n;
	mpc_params["w_q_n"] = w_q_n;
	mpc_params["w_T"] = w_T;
	mpc_params["w_omega_x"] = w_omega_x;
	mpc_params["w_omega_y"] = w_omega_y;
	mpc_params["w_omega_z"] = w_omega_z;
	mpc_params["w_U_x"] = w_U_x;
	mpc_params["w_U_y"] = w_U_y;
	mpc_params["w_U_z"] = w_U_z;

	mpc.reset(new MPC());
	mpc->LoadParams(mpc_params);
	mpc->setSolver();
	// mpc->test_setSolver();

	spinner.reset(new ros::AsyncSpinner(4));
	spinner->start();
}

void detectorNode::control_loop_timer_cb(const ros::TimerEvent &event)
{
	/***************************************************************************************************************************
	 * 数据记录
	 *   1        2          3          4        5 6 7     8     9     10     11 12 13      14 15 16
	 * method | 时间戳 | 捕获/丢失flag | flag | 定位x y z | Relative_pos(0) Relative_pos(1) Relative_pos(2) | 速度x y z | 速度sp x y z|
	 ***************************************************************************************************************************/

	if (data_record == 1 && file_created == 0)
	{
		char filename[100] = {0};
		time_t time_now = time(NULL);
		tm *p = localtime(&time_now);
		sprintf(filename, "/home/yx/experiment_record/data_record/Data %d-%d-%d %d:%02d.csv", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min);
		oFile.open(filename, ios::out | ios::app);
		file_created = 1;
		oFile << "time"
			  << ","
			  << "id"
			  << ","
			  << "flag"
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

	// c_time = ros::Time::now();
	// start_time = ros::Time::now().toSec();

	switch (control_method)
	{
	// PID
	case 0:
		// PID_control();
		break;

	//  模糊PID
	case 1:
		// FuzzyPID_control();
		break;

	//  NMPC
	case 2:
		NMPC_control();
		break;

	default:
		break;
	}

	/// PrintInfo();

	/*
	if (file_created == 1 && data_record == 1)
	{
		oFile << current_time << "," << id << "," << flag << "," << Relative_pos(0) << "," << Relative_pos(1) << "," << Relative_pos(2) << "," << Relative_dpos(0) << "," << Relative_dpos(1) << "," << Relative_dpos(2) << "," << solve_time << "," << thrust_sp << "," << rate_sp(0) << "," << rate_sp(1) << "," << rate_sp(2) << endl;
	}
	*/
}

void detectorNode::NMPC_control()
{

	if (id != -1 && land == 0)
	{
		flag = 1;
	}

	// c_time = ros::Time::now();
	current_time = ros::Time::now().toSec();
	dt = current_time - last_time;

	/*
	cout << "current_time: " << current_time << endl;
	if (current_time > 25)
	{
		last_flag = 1;
	}
	else
		last_flag = 0;
	*/

	if (last_flag == 1)
	{
		Relative_dpos(0) = (Relative_pos(0) - Relative_pos_ex(0)) / dt;
		Relative_dpos(1) = (Relative_pos(1) - Relative_pos_ex(1)) / dt;
		Relative_dpos(2) = (Relative_pos(2) - Relative_pos_ex(2)) / dt;

		accel_ugv(0) = (vel_ugv(0) - vel_ugv_ex(0)) / dt;
		accel_ugv(1) = (vel_ugv(1) - vel_ugv_ex(1)) / dt;
		accel_ugv(2) = (vel_ugv(2) - vel_ugv_ex(2)) / dt;

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

		x0 << Relative_pos(0), Relative_pos(1), Relative_pos(2), Relative_dpos(0), Relative_dpos(1), Relative_dpos(2), q_detect(0), q_detect(1), q_detect(2), q_detect(3);

		xRef << Pos_target(0), Pos_target(1), Pos_target(2), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		cout << "x0: " << x0.transpose() << endl;

		/*
		if (q_fcu.w() < 0.0)
		{
			q_fcu.w() = -q_fcu.w();
			q_fcu.x() = -q_fcu.x();
			q_fcu.y() = -q_fcu.y();
			q_fcu.z() = -q_fcu.z();
		}

		x0 << pos_drone_fcu(0), pos_drone_fcu(1), pos_drone_fcu(2), linear_vel_drone_fcu(0), linear_vel_drone_fcu(1), linear_vel_drone_fcu(2), q_fcu.w(), q_fcu.x(), q_fcu.y(), q_fcu.z();

		xRef << 3.0, 0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		cout << "x0: " << x0.transpose() << endl;
		cout << "xRef: " << xRef.transpose() << endl;
		*/

		Pos_err(0) = xRef(0) - x0(0);
		Pos_err(1) = xRef(1) - x0(1);
		Pos_err(2) = xRef(2) - x0(2);

		//    高亮 蓝色背景 白色字  关闭所有
		// \033[1m\033[44;37m  \033[0m

		// cout << "x1-x6(p, v): \033[1m\033[44;37m" << x0(0) << ", " << x0(1) << ", " << x0(2) << ", " << x0(3) << ", " << x0(4) << ", " << x0(5) << "\033[0m" << endl;
		// cout << "x7-x10(q): " << x0(6) << ", " << x0(7) << ", " << x0(8) << ", " << x0(9) << endl;
		// cout << "quat_w: " << quat_w.w() << ", " << quat_w.x() << ", " << quat_w.y() << ", " << quat_w.z() << endl;
		// cout << "-q_fcu: " << -q_fcu.w() << ", " << -q_fcu.x() << ", " << -q_fcu.y() << ", " << -q_fcu.z() << endl;
		// cout << endl;

		landing_strategy(land_flag, status, Pos_err, land_num1, land_num2, land_time1, land_time2);
	}

	thrust_sp_normalize = hover_throttle;

	if (last_flag == 1)
	{
		auto solve_start = ros::Time::now();
		mpc->solve(x0, xRef, q_ugv, omega_ugv, alpha_ugv, accel_ugv);
		auto solve_end = ros::Time::now();
		solve_time = (solve_end - solve_start).toSec();
		// cout << "solve time: " << solve_time << endl;

		auto mpc_result = mpc->getFirstU();
		// cout << "mpc_result: " << mpc_result << endl;

		thrust_sp = mpc_result[0];
		thrust_sp_normalize = thrust_sp * hover_throttle / 9.8;
		thrust_sp_normalize = limit(thrust_sp_normalize, 0.25, 0.6);

		rate_sp(0) = mpc_result[1];
		rate_sp(1) = mpc_result[2];
		rate_sp(2) = mpc_result[3];

		// thrust_sp_normalize = hover_throttle;
		// rate_sp(0) = 0.0;
		// rate_sp(1) = 0.0;
		// rate_sp(2) = 0.0;
	}

	// detector_uav_cmd.attitude_control_flag = false;
	detector_uav_cmd.attitude_control_flag = true;
	// detector_uav_cmd.attitude_control_type_mask = 0b00000111; // 忽略ROLL_RATE, PITCH_RATE, YAW_RATE
	detector_uav_cmd.attitude_control_type_mask = 0b10000000; // 忽略ATTITUDE
	detector_uav_cmd.thrust = thrust_sp_normalize;

	// detector_uav_cmd.quat.w = q_sp.w();
	// detector_uav_cmd.quat.x = q_sp.x();
	// detector_uav_cmd.quat.y = q_sp.y();
	// detector_uav_cmd.quat.z = q_sp.z();

	detector_uav_cmd.body_rate.x = rate_sp(0);
	detector_uav_cmd.body_rate.y = rate_sp(1);
	detector_uav_cmd.body_rate.z = rate_sp(2);

	detector_uav_cmd.flag = flag;
	detector_vel_pub.publish(detector_uav_cmd);

	/*
	Control_Queue.push(ControlCommand<aruco_msgs::Detector_vel>(detector_uav_cmd, current_time));

	if (!Control_Queue.empty() && ros::Time::now().toSec() - Control_Queue.front().time >= TimeDelay())
	{
		detector_vel_pub.publish(Control_Queue.front().msg);
		last_cmd = Control_Queue.front().msg;
		Control_Queue.pop();
	}
	else
		detector_vel_pub.publish(last_cmd);
	*/

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

	if (file_created == 1 && data_record == 1)
	{
		oFile << current_time << "," << id << "," << flag << "," << Relative_pos(0) << "," << Relative_pos(1) << "," << Relative_pos(2) << "," << Pos_target(2) - Relative_pos(2) << "," << Relative_dpos(0) << "," << Relative_dpos(1) << "," << Relative_dpos(2) << "," << thrust_sp / mass << "," << rate_sp(0) << "," << rate_sp(1) << "," << rate_sp(2) << "," << solve_time << endl;
	}
}

/*
void detectorNode::FuzzyPID_control()
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

/*
void detectorNode::PID_control()
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
*/

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
	// ROS_INFO("****************Speed Sp*****************");
	// ROS_INFO("vel_sp(0):%f", vel_sp(0));
	// ROS_INFO("vel_sp(1):%f", vel_sp(1));
	// ROS_INFO("vel_sp(2):%f", vel_sp(2));
	// ROS_INFO("************Detected Attitude************");
	//  ROS_INFO("detected roll:%f", rpy.x);
	//  ROS_INFO("detected pitch:%f", rpy.y);
	// ROS_INFO("detected yaw:%f", rpy.z);
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
/*
void detectorNode::t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
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

	ros::waitForShutdown();
	return 0;
}