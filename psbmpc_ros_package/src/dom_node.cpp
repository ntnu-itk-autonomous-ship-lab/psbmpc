/****************************************************************************************
 *
 *  File name : dom_node.cpp
 *
 *  Function  : Class functions for the PSBMPC Dynamic Obstacle Manager ROS node
 *
 *	           ---------------------
 *
 *  Version 1.0
 *
 *  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim.
 *  All rights reserved.
 *
 *  Author    : Trym Tengesdal
 *
 *  Modified  :
 *
 *****************************************************************************************/

#include "dom_node.hpp"

#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ctime>
#include <exception>
#include <limits>
#include <iostream>

/****************************************************************************************
 *  Name     : DOM_Node
 *  Function : Class constructor, initializes parameters, variables and objects
 *  Method   :
 *  Author   :
 *****************************************************************************************/
DOM_Node::DOM_Node(
	ros::NodeHandle &nh // In: Handle to the DOM node, initialized in the main loop
	) : nh(nh), synchronizer(ownship_pose_subscriber, ownship_velocity_subscriber, 1)
{
	//=========================================================================
	// Fetch parameters from the server, initialize relevant objects
	//=========================================================================
	psbmpc_mode = nh.param<std::string>("initial_psbmpc_mode", "psbmpc");
	first_update = false;

	do_data_topic_name = nh.param<std::string>("do_data_topic_name", "");
	do_estimate_topic_name = nh.param<std::string>("do_estimate_topic_name", "");
	polygons_topic_name = nh.param<std::string>("polygons_topic_name", "");
	ownship_pose_topic_name = nh.param<std::string>("ownship_pose_topic_name", "");
	ownship_velocity_topic_name = nh.param<std::string>("ownship_velocity_topic_name", "");
	get_polygons_service_name = nh.param<std::string>("get_polygons_service_name", "");
	psbmpc_mode_topic_name = nh.param<std::string>("psbmpc_mode_topic_name", "");

	do_data_publish_rate = nh.param<double>("do_data_publish_rate", 0.0001);

	ownship_length = nh.param<double>("/psbmpc/ownship/l", 6);
	ownship_state.resize(0);

	t_prev = ros::Time::now().sec, t_now = t_prev;

	std::string u_offsets, chi_offsets;
	std::vector<double> dpars;
	std::vector<int> ipars;
	int cpe_method(0), prediction_method(0), guidance_method(0);
	u_offsets = nh.param<std::string>("/psbmpc/sbmpc_parameters/u_offsets", u_offsets);
	chi_offsets = nh.param<std::string>("/psbmpc/sbmpc_parameters/chi_offsets", chi_offsets);
	prediction_method = nh.param<int>("/psbmpc/sbmpc_parameters/prediction_method", 100);
	guidance_method = nh.param<int>("/psbmpc/sbmpc_parameters/guidance_method", 100);
	ipars = nh.param<std::vector<int>>("/psbmpc/sbmpc_parameters/ipars", ipars);
	dpars = nh.param<std::vector<double>>("/psbmpc/sbmpc_parameters/dpars", dpars);

	sbmpc_pars = PSBMPC_LIB::SBMPC_Parameters(
		PSBMPC_LIB::CPU::parse_VVD(u_offsets),
		PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
		static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
		static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method),
		ipars,
		dpars);

	u_offsets = nh.param<std::string>("/psbmpc/psbmpc_parameters/u_offsets", u_offsets);
	chi_offsets = nh.param<std::string>("/psbmpc/psbmpc_parameters/chi_offsets", chi_offsets);
	cpe_method = nh.param<int>("/psbmpc/psbmpc_parameters/cpe_method", 1);
	prediction_method = nh.param<int>("/psbmpc/psbmpc_parameters/prediction_method", 1);
	guidance_method = nh.param<int>("/psbmpc/psbmpc_parameters/guidance_method", 1);
	ipars = nh.param<std::vector<int>>("/psbmpc/psbmpc_parameters/ipars", ipars);
	dpars = nh.param<std::vector<double>>("/psbmpc/psbmpc_parameters/dpars", dpars);

	psbmpc_pars = PSBMPC_LIB::PSBMPC_Parameters(
		PSBMPC_LIB::CPU::parse_VVD(u_offsets),
		PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
		static_cast<PSBMPC_LIB::CPE_Method>(cpe_method),
		static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
		static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method),
		ipars,
		dpars);

	double T_lost_limit(15.0), T_tracked_limit(15.0);
	bool obstacle_filter_on(false);
	T_lost_limit = nh.param<double>("manager/T_lost_limit", T_lost_limit);
	T_tracked_limit = nh.param<double>("manager/T_tracked_limit", T_tracked_limit);
	obstacle_filter_on = nh.param<bool>("manager/obstacle_filter_on", obstacle_filter_on);
	psbmpc_do_manager = PSBMPC_LIB::Obstacle_Manager(T_lost_limit, T_tracked_limit, obstacle_filter_on);
	sbmpc_do_manager = PSBMPC_LIB::Obstacle_Manager(T_lost_limit, T_tracked_limit, obstacle_filter_on);

	double r_ct(10.0), sigma_x(0.01), sigma_xy(0.0), sigma_y(0.01), gamma_x(0.1), gamma_y(0.1);
	r_ct = nh.param<double>("predictor/r_ct", r_ct);
	sigma_x = nh.param<double>("predictor/sigma_x", sigma_x);
	sigma_xy = nh.param<double>("predictor/sigma_xy", sigma_xy);
	sigma_y = nh.param<double>("predictor/sigma_y", sigma_y);
	gamma_x = nh.param<double>("predictor/gamma_x", gamma_x);
	gamma_y = nh.param<double>("predictor/gamma_y", gamma_y);

	sbmpc_do_predictor = PSBMPC_LIB::Obstacle_Predictor(
		r_ct,
		sigma_x,
		sigma_xy,
		sigma_y,
		gamma_x,
		gamma_y,
		sbmpc_pars);

	psbmpc_do_predictor = PSBMPC_LIB::Obstacle_Predictor(
		r_ct,
		sigma_x,
		sigma_xy,
		sigma_y,
		gamma_x,
		gamma_y,
		psbmpc_pars);

	polygons_ned.resize(0);
	relevant_polygons_ned.resize(0);

	//=========================================================================
	// Initialize clients, services, publishers and subscribers
	//=========================================================================
	do_data_publisher = nh.advertise<ros_af_msgs::DynamicObstaclesData>(do_data_topic_name, 1);
	do_data_publish_timer = nh.createTimer(
		ros::Duration(1.0 / do_data_publish_rate),
		std::bind(&DOM_Node::publish_do_data, this));

	do_estimate_subscriber = nh.subscribe(do_estimate_topic_name, 1, &DOM_Node::do_estimate_callback, this);

	polygons_subscriber = nh.subscribe(polygons_topic_name, 1, &DOM_Node::polygons_callback, this);

	ownship_pose_subscriber.subscribe(nh, ownship_pose_topic_name, 1);
	ownship_velocity_subscriber.subscribe(nh, ownship_velocity_topic_name, 1);
	synchronizer.registerCallback(boost::bind(&DOM_Node::ownship_state_callback, this, _1, _2));

	psbmpc_mode_subscriber = nh.subscribe(psbmpc_mode_topic_name, 1, &DOM_Node::psbmpc_mode_callback, this);

	get_polygons_client = nh.serviceClient<psbmpc::GetPolygons>(get_polygons_service_name);
}

/****************************************************************************************
 *  Name     : publish_do_data
 *  Function : Publishes dynamic obstacle data (individually)
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void DOM_Node::publish_do_data()
{
	std::scoped_lock dom_data_lock(
		ownship_data_mutex,
		do_estimate_data_mutex,
		track_data_mutex,
		polygons_data_mutex);

	bool initialized = ownship_state.size() >= 4 && obstacle_states.cols() > 0 && obstacle_covariances.cols() > 0;
	if (!initialized)
	{
		return;
	}

	// Only update when obstacle states has proper values
	for (int i = 0; i < obstacle_states.cols(); i++)
	{
		if (obstacle_states.col(i).isZero())
		{
			return;
		}
	}

	t_now = ros::Time::now().sec;
	if (!first_update)
	{
		first_update = true;
		t_prev = ros::Time::now().sec;
		t_now = t_prev;
	}

	PSBMPC_LIB::Dynamic_Obstacles obstacles;
	double dt_p(0.0);
	if (psbmpc_mode == "sbmpc")
	{
		sbmpc_do_manager(obstacle_states, obstacle_covariances, sbmpc_pars, t_now - t_prev);

		sbmpc_do_predictor(sbmpc_do_manager.get_data(), ownship_state, sbmpc_pars, true);

		obstacles = sbmpc_do_manager.get_data();
		dt_p = sbmpc_pars.get_dpar(i_dpar_dt_SBMPC);
	}
	else if (psbmpc_mode == "psbmpc")
	{
		psbmpc_do_manager(obstacle_states, obstacle_covariances, psbmpc_pars, t_now - t_prev);

		psbmpc_do_predictor.operator()(psbmpc_do_manager.get_data(), ownship_state, psbmpc_pars);

		obstacles = psbmpc_do_manager.get_data();
		dt_p = psbmpc_pars.get_dpar(i_dpar_dt);
	}

	int n_do(obstacles.size());
	ros_af_msgs::DynamicObstaclesData do_data_msg;
	do_data_msg.do_data.resize(n_do);
	do_data_msg.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
	do_data_msg.header.stamp = ros::Time::now();

	ros_af_msgs::DynamicObstacleTrajectory trajectory;
	ros_af_msgs::KinematicEstimate estimate;

	std::vector<Eigen::MatrixXd> xs_i_p;
	Eigen::MatrixXd P_p;
	Eigen::Matrix4d P_k;
	double l(0.0), w(0.0), d_0i(0.0);
	int n_ps_i(0), n_samples(0);
	for (int i = 0; i < n_do; i++)
	{
		do_data_msg.do_data[i].header.frame_id = nh.param<std::string>("/frames/local_NED", "");
		do_data_msg.do_data[i].header.stamp.sec = do_time_stamps(i);
		do_data_msg.do_data[i].ID = obstacles[i].get_ID();

		l = obstacles[i].get_length();
		w = obstacles[i].get_width();
		do_data_msg.do_data[i].dimensions.x = l / 2.0;
		do_data_msg.do_data[i].dimensions.y = l / 2.0;
		do_data_msg.do_data[i].dimensions.z = w / 2.0;
		do_data_msg.do_data[i].dimensions.w = w / 2.0;

		xs_i_p = obstacles[i].get_trajectories();
		P_p = obstacles[i].get_trajectory_covariance();
		n_ps_i = xs_i_p.size();

		d_0i = (xs_i_p[0].block<2, 1>(0, 0) - ownship_state.block<2, 1>(0, 0)).norm() - l;

		if (d_0i < 1.5 * psbmpc_pars.get_dpar(i_dpar_d_safe))
		{
			// ROS_WARN("DOM DO %d: INSIDE 1.5x SAFETY ZONE! | d_0i =  %.2f\n", i, d_0i);
			if (d_0i < psbmpc_pars.get_dpar(i_dpar_d_safe))
			{
				ROS_WARN("DOM DO %d: INSIDE SAFETY ZONE! | d_0i =  %.2f\n", i, d_0i);
			}
		}

		for (int ps = 0; ps < n_ps_i; ps++)
		{
			trajectory.ID = ps;
			trajectory.dt_p = dt_p;
			trajectory.trajectory.clear();

			n_samples = xs_i_p[ps].cols();

			for (int k = 0; k < n_samples; k++)
			{
				P_k = PSBMPC_LIB::CPU::reshape(P_p.col(k), 4, 4);

				estimate.pos_est.x = xs_i_p[ps](0, k);
				estimate.pos_est.y = xs_i_p[ps](1, k);
				estimate.vel_est.x = xs_i_p[ps](2, k);
				estimate.vel_est.y = xs_i_p[ps](3, k);

				estimate.pos_cov.var_x = P_k(0, 0);
				estimate.pos_cov.var_y = P_k(1, 1);
				estimate.pos_cov.corr_xy = P_k(0, 1);

				estimate.vel_cov.var_x = P_k(2, 2);
				estimate.vel_cov.var_y = P_k(3, 3);
				estimate.vel_cov.corr_xy = P_k(2, 1);

				estimate.pos_vel_corr.corr_px_vx = P_k(0, 2);
				estimate.pos_vel_corr.corr_px_vy = P_k(0, 3);
				estimate.pos_vel_corr.corr_py_vx = P_k(1, 2);
				estimate.pos_vel_corr.corr_py_vy = P_k(1, 3);
				trajectory.trajectory.push_back(estimate);
			}
			do_data_msg.do_data[i].predicted_trajectories.push_back(trajectory);
		}
	}
	// ROS_WARN("Publishing %d trajectories", (int)do_data_msg.do_data[0].predicted_trajectories.size());
	// ROS_INFO("Published DO data message!");
	do_data_publisher.publish(do_data_msg);

	t_prev = t_now;
}

/****************************************************************************************
 *  Name     : do_estimate_callback
 *  Function : Collects dynamic obstacle state information for the ownship. for all
 * 			  observable obstacles.
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void DOM_Node::do_estimate_callback(
	const ros_af_msgs::DynamicObstacleEstimate::ConstPtr &msg // In: ROS message containing a dynamic obstacle ID, size estimate, state estimate, covariance estimate
)
{
	std::scoped_lock do_estimate_data_lock(do_estimate_data_mutex);

	int ID(msg->ID);
	while (ID >= obstacle_states.cols())
	{
		obstacle_states.conservativeResize(9, obstacle_states.cols() + 1);
		obstacle_covariances.conservativeResize(16, obstacle_covariances.cols() + 1);
		do_time_stamps.conservativeResize(do_time_stamps.size() + 1);
	}

	double A(msg->dimensions.x), B(msg->dimensions.y), C(msg->dimensions.z), D(msg->dimensions.w);
	Eigen::Vector4d xs_i;
	Eigen::Matrix4d P;
	Eigen::Matrix2d pos_vel_cov;

	do_time_stamps(ID) = msg->kinematic_estimate.header.stamp.sec;
	xs_i(0) = msg->kinematic_estimate.kinematic_estimate.pos_est.x;
	xs_i(1) = msg->kinematic_estimate.kinematic_estimate.pos_est.y;
	xs_i(2) = msg->kinematic_estimate.kinematic_estimate.vel_est.x;
	xs_i(3) = msg->kinematic_estimate.kinematic_estimate.vel_est.y;
	obstacle_states.col(ID) << xs_i, A, B, C, D, ID;

	P(0, 0) = msg->kinematic_estimate.kinematic_estimate.pos_cov.var_x;
	P(0, 1) = msg->kinematic_estimate.kinematic_estimate.pos_cov.corr_xy;
	P(1, 0) = msg->kinematic_estimate.kinematic_estimate.pos_cov.corr_xy;
	P(1, 1) = msg->kinematic_estimate.kinematic_estimate.pos_cov.var_y;

	P(2, 2) = msg->kinematic_estimate.kinematic_estimate.vel_cov.var_x;
	P(2, 3) = msg->kinematic_estimate.kinematic_estimate.vel_cov.corr_xy;
	P(3, 2) = msg->kinematic_estimate.kinematic_estimate.vel_cov.corr_xy;
	P(3, 3) = msg->kinematic_estimate.kinematic_estimate.vel_cov.var_y;

	pos_vel_cov(0, 0) = msg->kinematic_estimate.kinematic_estimate.pos_vel_corr.corr_px_vx;
	pos_vel_cov(0, 1) = msg->kinematic_estimate.kinematic_estimate.pos_vel_corr.corr_px_vy;
	pos_vel_cov(1, 0) = msg->kinematic_estimate.kinematic_estimate.pos_vel_corr.corr_py_vx;
	pos_vel_cov(1, 1) = msg->kinematic_estimate.kinematic_estimate.pos_vel_corr.corr_py_vy;

	P.block<2, 2>(0, 2) = pos_vel_cov;
	P.block<2, 2>(2, 0) = pos_vel_cov;

	obstacle_covariances.col(ID) = PSBMPC_LIB::CPU::flatten(P);

	// ROS_INFO("Published DO %d estimate", ID);
}

/****************************************************************************************
 *  Name     : polygons_callback
 *  Function : Collects PSBMPC grounding hazard data (relevant inside a range of
 *			  d_so_relevant).
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void DOM_Node::polygons_callback(
	const custom_msgs::Polygons::ConstPtr &msg // In: Message containing PSBMPC polygons (grounding hazards)
)
{
	std::scoped_lock polygons_data_lock(polygons_data_mutex);

	polygons_ned.clear();
	int n_polygons = msg->polygons.size(), n_vertices(0);
	polygon_2D polygon, polygon_copy;
	typename boost::geometry::point_type<polygon_2D>::type v;
	for (int j = 0; j < n_polygons; j++)
	{
		n_vertices = msg->polygons[j].points.size();
		for (int i = 0; i < n_vertices; i++)
		{
			boost::geometry::assign_values(v, msg->polygons[j].points[i].x, msg->polygons[j].points[i].y);
			boost::geometry::append(polygon_copy, v);
		}
		polygons_ned.push_back(polygon_copy);
		polygon_copy = polygon;
	}
}

/****************************************************************************************
 *  Name     : ownship_state_callback
 *  Function : Updates the ownship state
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void DOM_Node::ownship_state_callback(
	const geometry_msgs::PoseStamped::ConstPtr &pose_msg,  // In: ROS message containing ownship pose
	const geometry_msgs::TwistStamped::ConstPtr &twist_msg // In: ROS message containing ownship twist in body
)
{

	std::scoped_lock ownship_data_lock(ownship_data_mutex);

	tf::Quaternion q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double heading = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(yaw);
	// ROS_INFO("HEADING = %.5f\n", RAD2DEG * PSBMPC_LIB::CPU::wrap_angle_to_02pi(heading));

	Eigen::Vector3d nu;
	nu(0) = twist_msg->twist.linear.x;
	nu(1) = twist_msg->twist.linear.y;
	nu(2) = twist_msg->twist.angular.z;

#if OWNSHIP_TYPE == 0
	ownship_state.resize(4);
	ownship_state(0) = pose_msg->pose.position.x;
	ownship_state(1) = pose_msg->pose.position.y;
	double crab_angle(0.0);
	if (fabs(nu(0)) < 1e-02 && nu(1) < 1e-02)
	{
		crab_angle = 0.0;
	}
	else
	{
		crab_angle = 0.0; // atan2(nu(1), nu(0));
	}
	ownship_state(2) = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(heading + crab_angle);
	double speed_over_ground = sqrt(pow(nu(0), 2) + pow(nu(1), 2));
	ownship_state(3) = speed_over_ground;
#else
	ownship_state.resize(6);
	ownship_state(0) = pose_msg->pose.position.x;
	ownship_state(1) = pose_msg->pose.position.y;
	ownship_state(2) = heading;
	ownship_state(3) = nu(0);
	ownship_state(4) = nu(1);
	ownship_state(5) = nu(2);
#endif
}

/****************************************************************************************
 *  Name     : main
 *  Function : Sets up, initializes, runs the DOM ROS node, and shuts it down.
 *  Method   :
 *  Author   :
 *****************************************************************************************/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dom_node");
	ros::start();

	ROS_INFO("Start DOM node initialization...");
	ros::NodeHandle nh("~");
	DOM_Node dom_node(nh);
	ROS_INFO("DOM node initialization complete.");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}