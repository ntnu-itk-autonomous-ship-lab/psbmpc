/*******************************************************************************************
*
*  File name : asv_ctrl_psbmpc.cpp
*
*  Function  : Class functions for the PSBMPC ROS link. Modified version of the one created 
*			   for SBMPC by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor
*			   through the Autosea project.
*
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*******************************************************************************************/

#include "asv_ctrl_psbmpc.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "utilities.h"
#include <iostream>
#include <list>
#include <random>

/****************************************************************************************
*  Name     : PSBMPC_ROS_Link
*  Function : Class constructor, initializes parameters, variables and objects
*  Method   : 
*  Author   : 
*****************************************************************************************/
PSBMPC_ROS_Link::PSBMPC_ROS_Link() : 
	asv_state(nullptr),
	u_d(nullptr),
	psi_d(nullptr),
	next_waypoint(nullptr),
	obstacle_states(nullptr),
	obstacle_covariances(nullptr),
	map(nullptr)
{
	psbmpc = new PSBMPC(); 
}

/****************************************************************************************
*  Name     : PSBMPC_ROS_Link
*  Function : Class destructor, clears dynamic objects
*  Method   : 
*  Author   : 
*****************************************************************************************/
PSBMPC_ROS_Link::~PSBMPC_ROS_Link(){
	delete asv_state;
	delete u_d;
	delete psi_d;
	delete next_waypoint;
	delete obstacle_states;
	delete obstacle_covariances;
	delete map;
	delete psbmpc;
}

/****************************************************************************************
*  Name     : initialize
*  Function : Sets up the ROS link, local map for debugging and its publisher
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_ROS_Link::initialize(
	const Eigen::Matrix<double, 6 ,1> *asv_state,					// In: Pointer to ASV state information
	const Eigen::Vector2d *u_d,										// In: Pointer to surge reference for the ASV
	const Eigen::Vector2d *psi_d,									// In: Pointer to heading reference for the ASV
	const std::vector<asv_msgs::WP> *next_waypoint,					// In: Pointer to next waypoint information 
	const std::vector<asv_msgs::State> *obstacle_states,			// In: Pointer to obstacle state information from tracker node
	const std::vector<asv_msgs::Covariance> *obstacle_covariances,	// In: Pointer to obstacle covariance information from tracker node
	const nav_msgs::OccupancyGrid *map 								// In: Pointer to occupancy grid information from map server
	)
{
	ROS_INFO("Initializing PSBMPC node...");
	
	// Set ROS logging output to be on DEBUG format
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	this->asv_state = asv_state;

	this->u_d = u_d;

	this->psi_d = psi_d;

	this->next_waypoint = next_waypoint;

	this->obstacle_states = obstacle_states;

	this->obstacle_covariances = obstacle_covariances;

	this->map = map;

	// NOTE: Only used for debugging
	local_map.header.frame_id ="map";
	local_map.info.resolution = 0.78;
	local_map.info.width  = 1362;
	local_map.info.height = 942;
	local_map.info.origin.position.x = -(float)496;
	local_map.info.origin.position.y = -(float)560;
	local_map.data.resize(local_map_.info.width*local_map_.info.height);

	ros::NodeHandle n;

	local_map_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);

	ROS_INFO("Initialization complete");
}

/****************************************************************************************
*  Name     : get_optimal_offsets
*  Function : Adds simulated noise to obstacle states for debugging?, sets up data for-
*			  and runs the PSBMPC algorithm. Natural place to display data
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_ROS_Link::calculate_optimal_offsets(
	double &u_m_opt, 												// In/out: Optimal surge offset 
	double &chi_m_opt												// In/out: Optimal course offset
	)
{
	static int loop_counter = 1;

	/************************************************************************************
	*	Variables for generating measurement (Guassian) noise
	*************************************************************************************/
	const double mean_pos = 0.0, mean_obs_pos = 0.0;
	const double stddev_pos = 0*1.0, stddev_obs_pos = 0*1.0; // 0.1 / 1 / 5.0 / 15.0
	static std::default_random_engine generator_pos(std::random_device{}()), generator_obs_pos;
	std::normal_distribution<double> distri_pos(mean_pos, stddev_pos), distri_obs_pos(mean_obs_pos, stddev_obs_pos);
	
	const double mean_chi = 0.0, mean_obs_chi = 0.0;
	const double stddev_chi = 0*0.02, stddev_obs_chi = 0*0.02; // (0.02rad -> 1deg, 0.53rad -> 30.37deg, 027rad -> 15.5deg)
	static std::mt19937 generator_chi(std::random_device{}()), generator_obs_chi;
	std::normal_distribution<double> distri_chi(mean_chi, stddev_chi), distri_obs_chi(mean_obs_chi, stddev_obs_chi);	
	
	const double mean_u = 0.0, mean_obs_u = 0.0;
	const double stddev_u = 0*1.0, stddev_obs_u = 0*1.0; // 1.0 / 1.0
	static std::minstd_rand generator_u(std::random_device{}()), generator_obs_u;
	std::normal_distribution<double> distri_u(mean_u, stddev_u), distri_obs_u(mean_obs_u, stddev_obs_u);

	/************************************************************************************
	*	Initialize and set up input parameters for the PSBMPC call
	*************************************************************************************/
	int n_obst = obstacle_states->size();
	int n_wps = next_waypoint->size(); 	 

	Eigen::Matrix<double, 6, 1> asv_state_pert;
	asv_state_pert << asv_state(0) + distri_pos(generator_pos), 
					  asv_state(1) + distri_pos(generator_pos), 
					  asv_state(2) + distri_chi(generator_chi), 
					  asv_state(3) + distri_u(generator_u), 
					  asv_state(4), 
					  asv_state(5); // MR interface output sign change

	Eigen::Matrix<double, 2, -1> next_waypoints;
	Eigen::Matrix<double, 2, -1> predicted_trajectory; 				

	// FORMAT: n_obst x [x, y, V_x, V_y, A, B, C, D, ID]!
	Eigen::Matrix<double, 9, -1> obstacle_states(9, n_obst); 
	Eigen::Matrix<double, 9, -1> obstacle_states_vary;
	Eigen::Matrix<double, 16, -1> obstacle_covariances(16, n_obst);
	
    Eigen::Matrix<double, 4, -1> static_obst(4, 1);
    static_obstacles << 50.0, 0.0, 50.0, 2050.0; //-40, 5, 1, 5;                // x_0, y_0, x_1, y_1					

	Eigen::VectorXd first_obstacle_state(4);

	/************************************************************************************
	*	Perturb obstacle states with simulated noise
	*************************************************************************************/
	int i;
	std::vector<asv_msgs::State>::iterator obst_it; 
	if (n_obst > 0){ 
		obstacle_states.resize(n_obst, 9);
		for (obst_it = obstacle_states->begin(); obst_it != obstacle_states->end(); ++obst_it){
			i = std::distance(obstacles->begin(), obst_it); 
			
			// save certain values of first for first obstacle to check variation in signal w.r.t. simulated uncertainty.
			if (it->header.id == 0){
				first_obstacle_state << obst_it->x, obst_it->y, obst_it->psi, obst_it->u;  
			}

			obstacle_states.row(i) << 
				obst_it->x + distri_obs_pos(generator_obs_pos), 
				obst_it->y + distri_obs_pos(generator_obs_pos), 
				wrap_angle_pmpi(obst_it->psi + distri_obs_chi(generator_obs_chi)), 
				obst_it->u + distri_obs_u(generator_obs_u), 
				obst_it->v, 
				obst_it->header.radius, 
				obst_it->header.radius, 
				obst_it->header.radius, 
				obst_it->header.radius, 
				obst_it->header.id; // + 5 + fabs(distri_obs_pos(generator_obs_pos)); // MR interface output sign change
		}
		// Do the same for covariances
	}
	
	/************************************************************************************
	*	Simulate obstacle leaving and re-entering list (i.e. out/in of sensor range)
	*************************************************************************************/
	if (false && n_obst > 0 ){
	
		if(loop_counter > 12 && loop_counter < 25){
		
			obstacle_states_vary.resize(9, n_obst - 1);
		
			for (int i = 0; i < n_obst - 1; i++)
				obstacle_states_vary.row(i) = obstacle_states.row(i); 	
		}
		else
		{
			obstacle_states_vary.resize(9, n_obst);
		
			for (int i = 0; i < n_obst; i++)
				obstacle_states_vary.col(i) = obstacle_states.col(i); 
		}	
	}
	else
	{
		obstacle_states_vary.resize(9, n_obst);
		
		for (int i = 0; i < n_obst; i++)
			obstacle_states_vary.col(i) = obstacle_states.col(i); 	
	}	

	loop_counter += 1;

	/************************************************************************************
	*	Extract next waypoint information
	*************************************************************************************/
	int j;
	std::vector<asv_msgs::WP>::iterator wp_it;
	Eigen::Vector2d v_os;
	//ROS_INFO("next_waypoint_ size: %0.2f   ", (double)next_waypoint_->size());
	if (n_wps > 0) { 
		
		next_waypoints.resize(n_nwps, 2); 
		//std::cout << "next_waypoint_ size : " << next_waypoint_->size() << std::endl;
		for (wp_it = next_waypoint_->begin(); wp_it != next_waypoint_->end(); wp_it++)
		{	
			j = std::distance(next_waypoint_->begin(), wp_it); 

			ROS_INFO("nwp_x: %0.2f   nwp_y: %0.2f", wp_it->x, wp_it->y);

			next_waypoints.col(j) << wp_it->x, wp_it->y;
		}
	}
	else
	{
		v_os(0) = asv_state_pert(3);
		v_os(1) = asv_state_pert(4);
		v_os = rotate_vector_2D(v_os, asv_state_pert(2));
		next_waypoints.resize(2, 2);
		next_waypoints.col(0) << asv_state_pert(0), asv_state_pert(1);
		next_waypoints.col(1) << asv_state_pert(0) + 300 * v_os(0), asv_state_pert(1) + 300 * v_os(1);
	}
	
	/************************************************************************************
	*	Run the Probabilitic Scenario-Based MPC
	*************************************************************************************/
	psbmpc->calculate_optimal_offsets(
		u_m_opt, 
		chi_m_opt, 
		predicted_trajectory, 
		*u_d, 
		*psi_d, 
		next_waypoints,
		*asv_state, 
		static_obstacles, 
		obstacle_manager.get_data()
		); // MR interface output sign change

	/************************************************************************************
	*	Display data
	*************************************************************************************/
	//ROS_INFO("asv_x: %0.2f  asv_y: %0.2f  asv_psi: %0.2f  asv_u: %0.2f  asv_v: %0.2f  asv_r: %0.2f", asv_state(0), asv_state(1), asv_state(2) * 180.0f / M_PI,
	//																								   asv_state(3), asv_state(4), asv_state(5));

	//ROS_INFO("obs_x: %0.2f  obs_y: %0.2f  obs_psi: %0.2f  obs_u: %0.2f  obs_v: %0.2f  obs_radius: %0.2f", 
	//	obstacle_states(0, 0), obstacle_states(1, 0), atan2(obstacle_states(3, 0)/obstacle_states(2, 0)) * 180.0f / M_PI, obstacle_states(2, 0), obstacle_states(3, 0), obstacle_states(4, 0));
	
	std::cout << "predicted_traj (x,y): " << predicted_trajectory.transpose() << std::endl;

	
	ROS_INFO("u_d: %0.2f    psi_d: %0.2f   u_opt: %0.2f    chi_opt: %0.2f", (*u_d), (*psi_d) * 180.0f / M_PI, u_m_opt, chi_m_opt * 180.0f / M_PI);

	//ROS_INFO("obst1_x: %0.2f   obst1_y: %0.2f   obst1_yaw: %0.2f   obst1_u: %0.2f", obst_states(0,0), obst_states(0,1), obst_states(0,2)*180.0f/M_PI, obst_states(0,3));

	ROS_INFO("asv_U: %0.2f   asv_x: %0.2f   asv_y: %0.2f   asv_yaw: %0.2f", 
		sqrt(asv_state(3)**2 + asv_state(4)**2), asv_state(0), asv_state(1), asv_state(2) * 180.0f / M_PI);

	ROS_INFO("m_asv_u: %0.2f   m_asv_x: %0.2f   m_asv_y: %0.2f   m_asv_yaw: %0.2f", 
		asv_state(3), asv_state(0), asv_state(1), asv_state(2) * 180.0f / M_PI);

	ROS_INFO("obst1_u: %0.2f   obst1_x: %0.2f   obst1_y: %0.2f   obst1_yaw: %0.2f", 
		first_obstacle_state(3), first_obstacle_state(0), first_obstacle_state(1), atan2(first_obstacle_state(3)/ first_obstacle_state(2)) * 180.0f / M_PI);

	ROS_INFO("m_obst1_U: %0.2f   m_obst1_x: %0.2f   m_obst1_y: %0.2f   m_obst1_yaw: %0.2f", 
		sqrt(obst_states(2, 0)**2 + obst_states(3, 0)**2), obst_states(0, 0), obst_states(1, 0), atan2(obstacle_states(3, 0) / obstacle_states(2, 0)) * 180.0f / M_PI;

	ROS_INFO("m_obst1_Vx: %0.2f   m_obst1_Vy: %0.2f   ", obst_states(2, 0), obst_states(3, 0));

	ROS_INFO("loop_counter: %0.2f   ", (double)loop_counter);

}
