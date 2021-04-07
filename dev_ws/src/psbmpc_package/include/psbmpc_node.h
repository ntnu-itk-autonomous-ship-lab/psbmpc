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
#include "psbmpc_interfaces/Trajectory2D.hpp"
#include "psbmpc_interfaces/Trajectory6D.hpp"
#include "psbmpc_interface/Offset.hpp"
#include "psbmpc_interfaces/KinematicEstimate.hpp"

#include "gpu/psbmpc_gpu.cuh"

#include <memory>

class PSBMPC_Node : public rclcpp::Node
{
private:

  nav_msgs::msg::Odometry ownship_state;

  psbmpc_interfaces::Trajectory2D waypoints;

  psbmpc_interfaces::Trajectory6D predicted_trajectory;

  std::unique_ptr<psbmpc_interfaces::KinematicEstimate> obstacles;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_subscription;

  std::shared_ptr<rclcpp::Subscription<psbmpc_interfaces::msg::Trajectory2D>> waypoints_subscription;

  std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Trajectory2D>> trajectory_publisher;

  //psbmpc_interfaces::Offset offset;
  //std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Offset>> trajectory_publisher;

  PSBMPC psbmpc;

public:

  PSBMPC_Node();

  ~PSBMPC_Node();

  void run();

};