/****************************************************************************************
 *
 *  File name : psbmpc_visualization_node.hpp
 *
 *  Function  : Header file for the node publishing marker array for the PSBMPC trajectory
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
#include "custom_msgs/Trajectory4.h"
#include "custom_msgs/Polygons.h"
#include "custom_msgs/NorthEastHeading.h"
#include "psbmpc/GetPolygons.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "ros_af_msgs/DynamicObstaclesData.h"

#include "psbmpc_parameters.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <mutex>

class PSBMPC_Visualization_Node
{
private:
    std::string trajectory_reference_vis_topic_name;
    std::string trajectory_vis_topic_name;
    std::string pose_vis_topic_name;
    std::string trajectory_reference_topic_name;
    std::string polygons_vis_topic_name;
    std::string ownship_pose_topic_name;
    std::string ownship_velocity_topic_name;
    std::string polygons_topic_name;
    std::string waypoint_topic_name;
    std::string get_polygons_service_name;
    std::string do_data_topic_name;
    std::string do_trajectories_vis_topic_name;

    ros::Publisher trajectory_reference_vis_publisher;
    ros::Publisher trajectory_vis_publisher;
    ros::Publisher pose_vis_publisher;
    ros::Publisher do_trajectories_vis_publisher;
    ros::Publisher polygons_vis_publisher;
    ros::Timer trajectory_reference_vis_publish_timer;
    ros::Timer polygons_vis_publish_timer;

    ros::Subscriber trajectory_reference_subscriber;
    message_filters::Subscriber<geometry_msgs::PoseStamped> ownship_pose_subscriber;
    message_filters::Subscriber<geometry_msgs::TwistStamped> ownship_velocity_subscriber;
    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> synchronizer;
    ros::Subscriber polygons_subscriber;
    ros::Subscriber waypoint_subscriber;
    ros::Subscriber do_trajectories_subscriber;

    ros::ServiceClient get_polygons_client;

    double trajectory_vis_publish_rate, trajectory_reference_vis_publish_rate, polygons_vis_publish_rate;
    double d_so_relevant_viz;

    Eigen::MatrixXd waypoints, trajectory, predicted_trajectory;
    std::vector<Eigen::MatrixXd> simplified_polygons, polygons;

    double trajectory_scale_x, trajectory_reference_scale_x, safety_zone_scale_x, polygon_scale_x;
    std::vector<double> trajectory_color, trajectory_reference_color, safety_zone_color, pose_color, pose_scale, wps_color, wps_scale, polygon_color;

    PSBMPC_LIB::PSBMPC_Parameters psbmpc_pars;

    std::mutex trajectory_reference_data_mutex, do_trajectory_data_mutex, polygons_data_mutex,
        simplified_polygons_data_mutex, wp_data_mutex, visibility_data_mutex, ownship_data_mutex;

    double t_now_trajectory_update, t_prev_trajectory_update, trajectory_update_threshold;
    bool first_trajectory_update;

    ros::NodeHandle &nh;

public:
    PSBMPC_Visualization_Node(ros::NodeHandle &nh);

    void publish_trajectory_reference_visualization();

    void publish_polygons_visualization();

    void publish_do_trajectories(const ros_af_msgs::DynamicObstaclesData::ConstPtr &msg);

    void trajectory_reference_callback(const custom_msgs::Trajectory4::ConstPtr &msg);

    void ownship_state_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg);

    void visualization_polygons_callback(const custom_msgs::Polygons::ConstPtr &msg);

    void relevant_psbmpc_polygons_callback(const custom_msgs::Polygons::ConstPtr &msg);

    void waypoint_callback(const custom_msgs::NorthEastHeading::ConstPtr &msg);
};
