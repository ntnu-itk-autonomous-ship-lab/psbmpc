/****************************************************************************************
*
*  File name : psbmpc_node.h
*
*  Function  : Class functions for the PSBMPC ROS2 node.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "psbmpc_interfaces/msg/trajectory2.hpp"
#include "psbmpc_interfaces/msg/trajectory6.hpp"
#include "psbmpc_interfaces/msg/offset.hpp"
#include "psbmpc_interfaces/msg/kinematic_estimate.hpp"

#include "gpu/psbmpc_gpu.cuh"

#include <string>
#include <memory>

class PSBMPC_Node : public rclcpp::Node
{
private:
  double u_opt, chi_opt;
  Eigen::MatrixXd predicted_trajectory;

  double u_d, chi_d;
  Eigen::VectorXd ownship_state;

  Eigen::MatrixXd waypoints;

  std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimate> obstacles;
  Eigen::MatrixXd obstacle_states, obstacle_covariances;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_subscription;
  std::shared_ptr<rclcpp::Subscription<psbmpc_interfaces::msg::Trajectory2>> waypoints_subscription;
  
  //std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Offset>> trajectory_publisher;
  std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Trajectory6>> trajectory_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  PSBMPC_LIB::Obstacle_Manager obstacle_manager;
  PSBMPC_LIB::GPU::PSBMPC psbmpc;

  publish_reference_trajectory();

public:

  explicit PSBMPC_Node(const rclcpp::NodeOptions &options) : PSBMPC_Node("PSBMPC_Node", options) {}
  explicit PSBMPC_Node(const std::string &node_name, const rclcpp::NodeOptions &options);

  void run();

};