/****************************************************************************************
*
*  File name : asv_ctrl_psbmpc.h
*
*  Function  : Header file for the PSBMPC ROS link. Modified version of the one created 
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
*****************************************************************************************/


#ifndef _ASV_CTRL_PSBMPC_ROS_LINK_H_
#define _ASV_CTRL_PSBMPC_ROS_LINK_H_

#include "psbmpc_lib/psbmpc.h" 
#include "asv_msgs/State.h"
#include "asv_msgs/WP.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"

class PSBMPC_ROS_Link
{
private:
				
	Eigen::Matrix<double, 6, 1> *asv_state;

	double *u_d;

	double *psi_d;
				
	std::vector<asv_msgs::WP> *next_waypoint;
		
	std::vector<asv_msgs::State> *obstacle_states;
	std::vector<asv_msgs::Covariance> *obstacle_covariances;
	
	nav_msgs::OccupancyGrid *map;
	
	nav_msgs::OccupancyGrid local_map;

	ros::Publisher local_map_pub;

	PSBMPC *psbmpc;

	Obstacle_Manager obstacle_manager;

public:

	PSBMPC_ROS_Link();

	~PSBMPC_ROS_Link();

	void initialize( const Eigen::VectorXd *asv_state,
					 const double *u_d, 
					 const double *psi_d,
					 const std::vector<asv_msgs::WP> *next_waypoint,
					 const std::vector<asv_msgs::State> *obstacle_states, 
					 const std::vector<asv_msgs::Covariance> *obstacle_covariances,
					 const nav_msgs::OccupancyGrid *map);

	void calculate_optimal_offsets(double &u_m_opt, double &chi_m_opt);

};

#endif 