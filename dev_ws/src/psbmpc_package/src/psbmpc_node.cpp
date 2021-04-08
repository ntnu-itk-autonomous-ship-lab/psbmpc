/****************************************************************************************
*
*  File name : psbmpc_node.cpp
*
*  Function  : Class functions for the PSBMPC ROS2 node, and the main function for
*              running it.
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

#include "psbmpc_node.hpp"

#include <cstdio>

/****************************************************************************************
*  Name     : Class constructor
*  Function : Initializor
*  Author   :
*  Modified :
*****************************************************************************************/
PSBMPC_Node::PSBMPC_Node(
  const std::string &node_name,
  const rclcpp::NodeOptions &options
  )
  : Node(node_name, options)
{
  trajectory_publisher = this->create_publisher<psbmpc_interfaces::msg::Trajectory6>("guidance/reference_trajectory", rclcpp::SystemDefaultsQoS());
}


/****************************************************************************************
*  Name     : main
*  Function : Starts and runs the PSBMPC node
*  Author   :
*  Modified :
*****************************************************************************************/
int main(int argc, char ** argv)
{
  int r = 0;
  try
  {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> psbmpc_node_ptr = std::make_shared<rclcpp::Node>("psbmpc_node");

    rclcpp::spin(psbmpc_node_ptr);

    rclcpp::shutdown();
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("psbmpc"), e.what());
    r = 3;
  } 
  catch (...) 
  {
    RCLCPP_INFO(rclcpp::get_logger("psbmpc"), "Unknown exception caught. ""Exiting...");
    r = -1;
  }

  return r; 
}