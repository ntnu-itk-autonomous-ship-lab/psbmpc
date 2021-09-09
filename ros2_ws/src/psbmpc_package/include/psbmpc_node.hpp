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
#include "bridge_msgs/msg/trajectory2.hpp"
#include "bridge_msgs/msg/trajectory4.hpp"
#include "bridge_msgs/msg/offset.hpp"
#include "bridge_msgs/msg/dynamic_obstacle_estimates.hpp"

#include "cpu/psbmpc_cpu.hpp"
#if USE_GPU_PSBMPC
  #include "gpu/psbmpc_gpu.cuh"
#endif

#include <string>
#include <memory>

class PSBMPC_Node : public rclcpp_lifecycle::LifecycleNode
{
private:
  //==================================================
  // Subscribers and publishers
  //==================================================
  rclcpp::Subscription<bridge_msgs::msg::DynamicObstacleEstimates>::SharedPtr dynamic_obstacle_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription;
  rclcpp::Subscription<bridge_msgs::msg::Trajectory2>::SharedPtr waypoints_subscription;
  //rclcpp_lifecycle::LifecyclePublisher<psbmpc_interfaces::msg::Offset>::SharedPtr trajectory_publisher;
  rclcpp_lifecycle::LifecyclePublisher<bridge_msgs::msg::Trajectory4>::SharedPtr trajectory_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  bool enable_topic_stats;
  const std::string topic_stats_topic_name;
  const std::string dynamic_obstacle_topic_name;
  const std::string state_topic_name;
  const std::string waypoints_topic_name;
  const std::string reference_topic_name;

  std::chrono::milliseconds topic_stats_publish_period;
  std::chrono::milliseconds deadline_duration;
  std::chrono::milliseconds trajectory_publish_period;  

  uint32_t n_missed_deadlines_do_sub;
  uint32_t n_missed_deadlines_state_sub;
  uint32_t n_missed_deadlines_wps_sub;
  uint32_t n_missed_deadlines_pub;

  //==================================================
  // PODs, data structures and classes for use by the node
  //==================================================
  double ownship_length;
  double u_opt, chi_opt, mean_t;
  Eigen::Matrix<double, 2, -1> predicted_trajectory;

  double u_d, chi_d;
  Eigen::VectorXd ownship_state;

  Eigen::MatrixXd waypoints;
  Eigen::MatrixXd obstacle_states, obstacle_covariances;
  std::vector<polygon_2D> relevant_polygons;

  PSBMPC_LIB::PSBMPC_Parameters pars;
  PSBMPC_LIB::CPU::CPE cpe;
  #if USE_GPU_PSBMPC
    PSBMPC_LIB::GPU::Ownship ownship;
    PSBMPC_LIB::GPU::PSBMPC psbmpc;
  #else
    PSBMPC_LIB::CPU::Ownship ownship;
    PSBMPC_LIB::CPU::PSBMPC psbmpc;
  #endif
  
  PSBMPC_LIB::Obstacle_Manager obstacle_manager;
  PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;
  PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager;
  //====================================================

  void dynamic_obstacle_callback(const bridge_msgs::msg::DynamicObstacleEstimates::SharedPtr msg);

  void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void waypoints_callback(const bridge_msgs::msg::Trajectory2::SharedPtr msg);

  void publish_reference_trajectory();

  void log_psbmpc_information();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

public:

  explicit PSBMPC_Node(const rclcpp::NodeOptions &options) : PSBMPC_Node("PSBMPC_Node", options) {}
  
  explicit PSBMPC_Node(
    const std::string &node_name, 
    const rclcpp::NodeOptions &options);

};