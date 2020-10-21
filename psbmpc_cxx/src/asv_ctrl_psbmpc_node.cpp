/****************************************************************************************
*
*  File name : asv_ctrl_psbmpc_node.h
*
*  Function  : Class functions for the PSBMPC ROS node, including the main loop for 
*			   running the PSBMPC real-time. Modified version of the one created 
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

#include "asv_ctrl_psbmpc_node.h"
#include "asv_ctrl_psbmpc.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <ctime>

/****************************************************************************************
*  Name     : PSBMPC_Node
*  Function : Class constructor, initializes parameters, variables and objects
*  Method   : 
*  Author   : 
*****************************************************************************************/
PSBMPC_Node::PSBMPC_Node() : 
	cmd_pub(NULL),
	offset_pub(NULL),
	asv_sub(NULL),
	cmd_sub(NULL),
	wp_sub(NULL),
	obstacle_sub(NULL),
	occupancy_grid_sub(NULL),
	psbmpc_link(NULL)
{
};

/****************************************************************************************
*  Name     : ~PSBMPC_Node
*  Function : Class destructor, clears dynamic objects
*  Method   : 
*  Author   : 
*****************************************************************************************/
PSBMPC_Node::~PSBMPC_Node() {
	delete cmd_pub;
	delete offset_pub;
	delete asv_sub;
	delete cmd_sub;
	delete wp_sub;
	delete obstacle_sub;
	delete occupancy_grid_sub;
	delete psbmpc_link;
};

/****************************************************************************************
*  Name     : initialize
*  Function : Initializes the publisher-subscriber communication for the node and the
*			  link to the PSBMPC library
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::initialize(
	ros::Publisher *cmd_pub,										// In: ASV guidance reference publisher
	ros::Publisher *offset_pub,										// In: ASV guidance reference offset publisher
	ros::Subscriber *asv_sub,										// In: ASV state information subscriber
	ros::Subscriber *cmd_sub,										// In: ASV guidance information subscriber
	ros::Subscriber *wp_sub,										// In: Waypoint information subscriber
	ros::Subscriber *obstacle_state_sub,							// In: Obstacle state information subscriber
	ros::Subscriber *obstacle_covariance_sub,						// In: Obstacle covariance information subscriber
	ros::Subscriber *occupancy_grid_sub,							// In: Occupancy grid information subscriber
	)
{
	this->cmd_pub = cmd_pub;
	this->offset_pub = offset_pub;

	this->asv_sub = asv_sub;
	this->cmd_sub = cmd_sub;
	this->wp_sub = wp_sub;

	this->obstacle_state_sub = obstacle_state_sub;
	this->obstacle_covariance_sub = obstacle_covariance_sub;

	this->occupancy_grid_sub = occupancy_grid_sub;
	
	this->psbmpc_link = psbmpc_link;

	u_m = 1;
	psi_m = 0;
	
	// Set up link from the ROS node to the PSBMPC library
	psbmpc_link = new PSBMPC_ROS_Link;
	psbmpc_link->initialize(&asv_state, &u_d, &psi_d, &next_waypoint, &obstacle_states, &obstacle_covariances, &map);
}										

/****************************************************************************************
*  Name     : run
*  Function : Starts and runs the PSBMPC node in aneternal while loop, gathers relevant data
*			  and uses it to calculate the optimal control behavior and publish the 
*			  corresponding modified guidance references for the asv
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::run()
{
	double rate = 10.0;
	ros::Rate loop_rate(rate);
	
	clock_t tick, tock;
	double t = 0;
	while (ros::ok())
	{
		if (t > 5){
			tick = clock();
			psbmpc_link->get_optimal_offsets(u_m, psi_m);
			tock = clock() - tick;

			offset.P_ca = u_m; 
			offset.Chi_ca = psi_m; // NB! negative for MR interface

			offset_pub->publish(offset);
			
			ROS_INFO("Runtime: %0.2f", ((float)tock)/CLOCKS_PER_SEC);	

			t = 0;
		}

		t += 1/rate;

		cmd_vel.linear.x = u_m * u_d;
		cmd_vel.angular.y = psi_m + psi_d; // NB! negative for MR interface

		cmd_pub->publish(cmd_vel);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/****************************************************************************************
*  Name     : asv_callback
*  Function : Updates the PSBMPC asv state, guidance references and next waypoint in one
*			  batch.
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::asv_callback(
	const nav_msgs::Odometry::ConstPtr &msg 						// In: ROS message containing ASV state
	)
{
	double yaw = tf::getYaw(msg->pose.pose.orientation);
  	asv_state(0) = msg->pose.pose.position.x;
  	asv_state(1) = msg->pose.pose.position.y;
  	asv_state(2) = yaw;
  	asv_state(3) = msg->twist.twist.linear.x;
  	asv_state(4) = msg->twist.twist.linear.y;
  	asv_state(5) = msg->twist.twist.angular.z;
}

/****************************************************************************************
*  Name     : cmd_callback
*  Function : Collects guidance reference information for the asv
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::cmd_callback(
	const geometry_msgs::Twist::ConstPtr &msg 						// In: ROS message containing guidance references
	)
{
	u_d = msg->linear.x;
	psi_d = msg->angular.y;
}

/****************************************************************************************
*  Name     : wp_callback
*  Function : Collects waypoint information for the asv
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::wp_callback(
	const asv_msgs::WPArray::ConstPtr &msg 							// In: ROS message containing the next waypoint information
	)
{	
	next_waypoint = msg->wp_xy;
}

/****************************************************************************************
*  Name     : obstacle_callback
*  Function : Collects obstacle state information for the asv
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::obstacle_callback(
	const asv_msgs::StateArray::ConstPtr &msg 						// In: ROS message containing obstacle states, n_obst x [x, y, psi, u, v, A, B, C, D, id]
	)
{
	obstacle_states = msg->states;
}

/****************************************************************************************
*  Name     : obstacle_callback
*  Function : Collects obstacle covariance information for the asv
*  Method   : 
*  Author   : 
*****************************************************************************************/
void PSBMPC_Node::obstacle_callback(
	const asv_msgs::CovarianceArray::ConstPtr &msg 					// In: ROS message containing obstacle covariances
	)
{
	obstacle_covariances = msg->covariances;
}

/****************************************************************************************
*  Name     : cmd_callback
*  Function : Collects occupancy grid information for the asv, containing information
*			  on static obstacles
*  Method   : 
*  Author   : 
*****************************************************************************************/
void::PSBMPC_Node::map_callback(
	const nav_msgs::OccupancyGrid::ConstPtr &msg 					// In: ROS message containing occupancy grid
	)
{
  // Copy what we need
  map.info.resolution = msg->info.resolution;
  map.info.height = msg->info.height;
  map.info.width = msg->info.width;
  map.info.origin.position.x = msg->info.origin.position.x;
  map.info.origin.position.y = msg->info.origin.position.y;

  ROS_INFO("r %f, h %d, w%d, px %f, py %f",
           map.info.resolution,
           map.info.height,
           map.info.width,
           map.info.origin.position.x,
           map.info.origin.position.y);

  map.data = msg->data;
}

/****************************************************************************************
*  Name     : main
*  Function : Sets up, initializes and runs the PSBMPC ROS node. A PSBMPC ROS link node
*			  is set up to act as a bridge between the PSBMPC library and the ROS node
*  Method   : 
*  Author   : 
*****************************************************************************************/
int main(int argc, char *argv[])
{

	/************************************************************************************
	*	Initialize PSBMPC node and communication channels
	*************************************************************************************/
	ros::init(argc, argv, "asv_ctrl_psbmpc_node");
	ros::start();
	
	ROS_INFO("Starting PSBMPC Node...");
	
	ros::NodeHandle n;
	
	PSBMPC_Node psbmpc_node;
	
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("asv/cmd_vel",10);
	
	ros::Publisher offset_pub = n.advertise<asv_msgs::Offset>("asv/offset",10);

    ros::Subscriber asv_sub	= n.subscribe("asv/state",
										  1,
										  &PSBMPC_Node::asv_callback,
										  &psbmpc_node);
	
	ros::Subscriber cmd_sub = n.subscribe("asv/LOS/cmd_vel",
										  1,
										  &PSBMPC_Node::cmd_callback,
										  &psbmpc_node);

	ros::Subscriber wp_sub = n.subscribe("asv/next_waypoint",
										 1,
										 &PSBMPC_Node::wp_callback,
										 &psbmpc_node);

	ros::Subscriber obstacle_sub = n.subscribe("obstacle_states", 
											   1,
											   &PSBMPC_Node::obstacle_callback,
											   &psbmpc_node);
											   
	ros::Subscriber occupancy_grid_sub = n.subscribe("/map",
													 1,
													 &PSBMPC_Node::map_callback,
													 &psbmpc_node);

	psbmpc_node.initialize( &cmd_pub, 
							&offset_pub, 
							&asv_sub, 
							&cmd_sub, 
							&wp_sub, 
							&obstacle_sub,
							&occupancy_grid_sub);

	/************************************************************************************
	*	Run the PSBMPC until aborted externally
	*************************************************************************************/
	psbmpc_node.run();
	
	/************************************************************************************
	*	Close the process
	*************************************************************************************/
	ros::shutdown();

	return 0;	
}