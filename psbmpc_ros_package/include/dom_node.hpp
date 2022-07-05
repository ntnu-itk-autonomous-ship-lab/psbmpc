/****************************************************************************************
 *
 *  File name : dom_node.hpp
 *
 *  Function  : Header file for the PSBMPC Dynamic Obstacle Manager (DOM) ROS node.
 *
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

#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Polygon.h"
#include "custom_msgs/Polygons.h"
#include "psbmpc/GetPolygons.h"
#include "ros_af_msgs/DynamicObstacleEstimate.h"
#include "ros_af_msgs/DynamicObstaclesData.h"

#include <string>
#include <mutex>
#include <thread>

#include "sbmpc_parameters.hpp"
#include "psbmpc_parameters.hpp"
#include "obstacle_manager.hpp"
#include "obstacle_predictor.hpp"
#include "grounding_hazard_manager.hpp"

class DOM_Node
{
private:
	ros::NodeHandle nh;
	//==================================================
	// Clients, subscribers and publishers
	//==================================================
	ros::Publisher do_data_publisher;
	ros::Timer do_data_publish_timer;

	ros::Subscriber psbmpc_mode_subscriber;
	ros::Subscriber do_estimate_subscriber;
	ros::Subscriber polygons_subscriber;
	message_filters::Subscriber<geometry_msgs::PoseStamped> ownship_pose_subscriber;
	message_filters::Subscriber<geometry_msgs::TwistStamped> ownship_velocity_subscriber;
	message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> synchronizer;

	ros::ServiceClient get_polygons_client;

	std::string do_data_topic_name;
	std::string do_estimate_topic_name;
	std::string polygons_topic_name;
	std::string ownship_pose_topic_name;
	std::string ownship_velocity_topic_name;
	std::string get_polygons_service_name;
	std::string psbmpc_mode_topic_name;

	double do_data_publish_rate;

	std::string local_ned_frame_name;

	//=======================================================
	// PODs, data structures and classes for use by the node
	//=======================================================
	std::mutex ownship_data_mutex, do_estimate_data_mutex, track_data_mutex, polygons_data_mutex, psbmpc_mode_data_mutex;

	std::string psbmpc_mode;
	bool first_update;
	double ownship_length;
	double t_now, t_prev;
	Eigen::Vector3d reference_frame_lla;
	Eigen::VectorXd ownship_state, do_time_stamps;
	Eigen::MatrixXd obstacle_states, obstacle_covariances;

	PSBMPC_LIB::Static_Obstacles polygons_ned, relevant_polygons_ned;

	PSBMPC_LIB::SBMPC_Parameters sbmpc_pars;
	PSBMPC_LIB::PSBMPC_Parameters psbmpc_pars;

	PSBMPC_LIB::Obstacle_Manager psbmpc_do_manager, sbmpc_do_manager;
	PSBMPC_LIB::Obstacle_Predictor psbmpc_do_predictor, sbmpc_do_predictor;
	//====================================================

public:
	DOM_Node(ros::NodeHandle &nh);

	void publish_do_data();

	void do_estimate_callback(const ros_af_msgs::DynamicObstacleEstimate::ConstPtr &msg);

	void polygons_callback(const custom_msgs::Polygons::ConstPtr &msg);

	void ownship_state_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg);

	void psbmpc_mode_callback(const std_msgs::String::ConstPtr &psbmpc_mode_msg)
	{
		std::scoped_lock psbmpc_mode_data_lock(psbmpc_mode_data_mutex);
		psbmpc_mode = psbmpc_mode_msg->data;
	}
};