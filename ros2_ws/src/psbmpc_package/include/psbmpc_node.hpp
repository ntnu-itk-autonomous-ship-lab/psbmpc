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
#include "psbmpc_interfaces/msg/dynamic_obstacle_estimates.hpp"

#include "gpu/psbmpc_gpu.cuh"

#include <string>
#include <memory>

class PSBMPC_Node : public rclcpp_lifecycle::LifecycleNode
{
private:
  //==================================================
  // Subscribers and publishers
  //==================================================
  std::shared_ptr<rclcpp::Subscription<psbmpc_interfaces::msg::DynamicObstacleEstimates>> dynamic_obstacle_subscription;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> state_subscription;
  std::shared_ptr<rclcpp::Subscription<psbmpc_interfaces::msg::Trajectory2>> waypoints_subscription;
  //rclcpp_lifecycle::LifecyclePublisher<psbmpc_interfaces::msg::Offset>::SharedPtr trajectory_publisher;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<psbmpc_interfaces::msg::Trajectory4>> trajectory_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  const std::string dynamic_obstacle_topic_name;
  const std::string state_topic_name;
  const std::string waypoints_topic_name;
  const std::string reference_topic_name;
  const std::string map_data_filename;
  const std::vector<double> map_origin;

  //==================================================
  // PODs, data structures and classes for use by the node
  //==================================================
  double ownship_length;
  double u_opt, chi_opt;
  Eigen::Matrix<double, 2, -1> predicted_trajectory;

  double u_d, chi_d;
  Eigen::VectorXd ownship_state;

  Eigen::MatrixXd waypoints;
  Eigen::MatrixXd obstacle_states, obstacle_covariances;
  std::vector<polygon_2D> relevant_polygons;

  PSBMPC_LIB::GPU::PSBMPC psbmpc;
  PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager;
  PSBMPC_LIB::Obstacle_Manager obstacle_manager;
  PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;

  std::chrono::milliseconds deadline_duration;
  uint32_t n_missed_deadlines_sub;
  uint32_t n_missed_deadlines_pub;

  void create_dynamic_obstacle_subscription();

  void create_state_subscription();

  void create_waypoints_subscription();

  void create_reference_trajectory_publisher();

  void create_publisher_timer_callback();

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

  explicit PSBMPC_Node(const rclcpp::NodeOptions &options) : PSBMPC_Node("PSBMPC_Node", options) {}
  
  explicit PSBMPC_Node(
    const std::string &node_name, 
    const rclcpp::NodeOptions &options);

};