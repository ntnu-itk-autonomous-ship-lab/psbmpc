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

  nav_msgs::msg::Odometry ownship_state;

  psbmpc_interfaces::msg::Trajectory2 waypoints;

  psbmpc_interfaces::msg::Trajectory6 predicted_trajectory;

  std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimate> obstacles;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_subscription;
  std::shared_ptr<rclcpp::Subscription<psbmpc_interfaces::msg::Trajectory2>> waypoints_subscription;
  std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Trajectory6>> trajectory_publisher;

  //psbmpc_interfaces::Offset offset;
  //std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Offset>> trajectory_publisher;

  PSBMPC_LIB::GPU::PSBMPC psbmpc;

public:

  explicit PSBMPC_Node(const rclcpp::NodeOptions &options) : PSBMPC_Node("PSBMPC_Node", options) {}
  explicit PSBMPC_Node(const std::string &node_name, const rclcpp::NodeOptions &options);

  void run();

};