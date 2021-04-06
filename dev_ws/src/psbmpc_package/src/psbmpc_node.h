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


#include <memory>

class PSBMPC_Node : public rclcpp::Node
{
private:

  psbmpc_interfaces::Trajectory2D waypoints;

  psbmpc_interfaces::Trajectory6D predicted_trajectory;

  psbmpc_interfaces::Offset offset;

  std::unique_ptr<psbmpc_interfaces::KinematicEstimate> obstacles;

  

public:

  PSBMPC_Node();

  ~PSBMPC_Node();

  void run();

};