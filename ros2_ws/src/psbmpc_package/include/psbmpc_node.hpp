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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "psbmpc_interfaces/msg/trajectory2.hpp"
#include "psbmpc_interfaces/msg/trajectory4.hpp"
#include "psbmpc_interfaces/msg/offset.hpp"
#include "psbmpc_interfaces/msg/kinematic_estimate.hpp"

#include "gpu/psbmpc_gpu.cuh"

#include <string>
#include <memory>

class PSBMPC_Node : public rclcpp_lifecycle::LifecycleNode
{
private:
  //==================================================
  // Subscribers and publishers
  //==================================================
  rclcpp::Subscription<psbmpc_interfaces::msg::KinematicEstimate>::SharedPtr obstacle_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription;
  rclcpp::Subscription<psbmpc_interfaces::msg::Trajectory2>::SharedPtr waypoints_subscription;
  //std::shared_ptr<rclcpp::Publisher<psbmpc_interfaces::msg::Offset>> trajectory_publisher;
  rclcpp::Publisher<psbmpc_interfaces::msg::Trajectory4>::SharedPtr trajectory_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  const std::string obstacle_topic_name;
  const std::string state_topic_name;
  const std::string waypoints_topic_name;
  const std::string reference_topic_name;

  //==================================================
  // PODs, data structures and classes for use by the node
  //==================================================
  double u_opt, chi_opt;
  Eigen::MatrixXd predicted_trajectory;

  double u_d, chi_d;
  Eigen::VectorXd ownship_state;

  Eigen::MatrixXd waypoints;
  Eigen::MatrixXd obstacle_states, obstacle_covariances;

  PSBMPC_LIB::GPU::PSBMPC psbmpc;
  PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager;
  PSBMPC_LIB::Obstacle_Manager obstacle_manager;
  PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;

  void obstacle_callback(const psbmpc_interfaces::msg::KinematicEstimate::SharedPtr &msg);

  void state_callback(const nav_msgs::msg::Odometry::SharedPtr &msg);

  void waypoints_callback(const psbmpc_interfaces::msg::Trajectory2::SharedPtr &msg);

  void publish_reference_trajectory();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

public:

  explicit PSBMPC_Node(
    const std::string &map_data_filename, 
    const Eigen::Vector2d &map_origin, 
    const rclcpp::NodeOptions &options) : PSBMPC_Node("PSBMPC_Node", map_data_filename, map_origin, options) {}
  
  explicit PSBMPC_Node(
    const std::string &node_name, 
    const std::string &map_data_filename, 
    const Eigen::Vector2d &map_origin, 
    const rclcpp::NodeOptions &options);

};