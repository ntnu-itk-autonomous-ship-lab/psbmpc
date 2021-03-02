/****************************************************************************************
*
*  File name : asv_ctrl_psbmpc_node.h
*
*  Function  : Header file for the PSBMPC ROS node. Modified version of the one created 
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


#ifndef _ASV_CTRL_PSBMPC_NODE_H_
#define _ASV_CTRL_PSBMPC_NODE_H_

#include "asv_msgs/StateArray.h"
#include "asv_msgs/WPArray.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include <vector>

class PSBMPC_Node
{
private:

	geometry_msgs::Twist cmd_vel; 

	asv_msgs::Offset offset;

	Eigen::VectorXd asv_state(6);
	
	double u_d;
	double u_m_opt;
	
	double psi_d;
	double chi_m_opt;

	std::vector<asv_msgs::WP> next_waypoint;

	std::vector<asv_msgs::State> obstacle_states;
	std::vector<asv_msgs::Covariance> obstacle_covariances;

	nav_msgs::OccupancyGrid map;
	
	ros::Publisher *cmd_pub;
	ros::Publisher *offset_pub;

	ros::Subscriber *asv_sub;
	ros::Subscriber *cmd_sub;
	ros::Subscriber *wp_sub;

	ros::Subscriber *obstacle_state_sub;
	ros::Subscriber *obstacle_covariance_sub;

	ros::Subscriber *occupancy_grid_sub;

	PSBMPC_ROS_Link *psbmpc_link;

public:

	PSBMPC_Node();

	~PSBMPC_Node();

	void initialize(const ros::Publisher *cmd_pub,
					const ros::Publisher *offset_pub,
					const ros::Subscriber *asv_sub,
					const ros::Subscriber *cmd_sub,
					const ros::Subscriber *wp_sub,
					const ros::Subscriber *obstacle_state_sub,
					const ros::Subscriber *obstacle_covariance_sub,
					const ros::Subscriber *occupancy_grid_sub);
	
	void run();	
	
	void asv_callback(const nav_msgs::Odometry::ConstPtr &msg);

	void obstacle_callback(const asv_msgs::StateArray::ConstPtr &msg);

	void obstacle_callback(const asv_msgs::CovarianceArray::ConstPtr &msg);

	void cmd_callback(const geometry_msgs::Twist::ConstPtr &msg);

	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	void wp_callback(const asv_msgs::WPArray::ConstPtr &msg);
	
};

#endif