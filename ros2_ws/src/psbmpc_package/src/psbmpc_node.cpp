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
#include "tf2/LinearMath/Transform.h"

#include <chrono>
#include <cstdio>
#include <functional>

using namespace std::chrono_literals;

/****************************************************************************************
*  Name     : Class constructor
*  Function : Initializer
*  Author   :
*  Modified :
*****************************************************************************************/
PSBMPC_Node::PSBMPC_Node(
  const std::string &node_name,                         // In: Self explanatory
  const std::string &map_data_filename,                 // In: Self explanatory
  const Eigen::Vector2d &map_origin,                    // In: Vector of north-east coordinates for the defined map origin
  const rclcpp::NodeOptions &options                    // In: 
  )
  : LifecycleNode(node_name, options), 
  obstacle_topic_name(declare_parameter("obstacle_topic_name").get<std::string>()),
  state_topic_name(declare_parameter("state_topic_name").get<std::string>()),
  waypoints_topic_name(declare_parameter("waypoints_topic_name").get<std::string>()),
  reference_topic_name(declare_parameter("reference_topic_name").get<std::string>()),
  psbmpc(PSBMPC_LIB::GPU::PSBMPC()), 
  grounding_hazard_manager(map_data_filename, map_origin, psbmpc)
{
  obstacle_subscription = this->create_subscription<psbmpc_interfaces::msg::KinematicEstimate>(
    obstacle_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::obstacle_callback, this, std::placeholders::_1));

  state_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
    state_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::state_callback, this, std::placeholders::_1));

  waypoints_subscription = this->create_subscription<psbmpc_interfaces::msg::Trajectory2>(
    waypoints_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::obstacle_callback, this, std::placeholders::_1));  

  trajectory_publisher = this->create_publisher<psbmpc_interfaces::msg::Trajectory4>(reference_topic_name, rclcpp::QoS(1));

  timer = this->create_wall_timer(5s, std::bind(&PSBMPC_Node::publish_reference_trajectory, this));
}

/****************************************************************************************
*  Name     : obstacle_callback
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::obstacle_callback(
  const psbmpc_interfaces::msg::KinematicEstimate::SharedPtr &msg   // Obstacle state and covariance estimate message
  )
{
  
}

/****************************************************************************************
*  Name     : state_callback
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::state_callback(
  const nav_msgs::msg::Odometry::SharedPtr &msg                      // In: State message
  )
{
  double heading(0.0);
  #if OWNSHIP_TYPE == 0
    ownship_state.resize(4);
    ownship_state(0) = msg->pose.pose.position.x;
    ownship_state(1) = msg->pose.pose.position.y;
    heading = tf2::getYaw(msg->pose.pose.orientation);
    ownship_state(2) = heading;

  #else
    ownship_state.resize(6);
    heading = tf2::getYaw(msg->pose.pose.orientation);
    ownship_state(0) = msg->pose.pose.position.x;
    ownship_state(1) = msg->pose.pose.position.y;
    ownship_state(2) = heading;
    ownship_state(3) = msg->twist.twist.linear.x;
    ownship_state(4) = msg->twist.twist.linear.y;
    ownship_state(5) = msg->twist.twist.angular.z;
  #endif
  
}

/****************************************************************************************
*  Name     : waypoints_callback
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::waypoints_callback(
  const psbmpc_interfaces::msg::Trajectory2::SharedPtr &msg         // In: Waypoint array message     
  )
{
  int n_wps = msg->waypoints.size();
  if (n_wps < 2)
  {
    throw "Less than two waypoints published!";
  }
  waypoints.resize(2, n_wps);
  for (int i = 0; i < n_wps; i++)
  {
    waypoints(0, i) = msg->waypoints[i].x;
    waypoints(1, i) = msg->waypoints[i].y;
  }
}
/****************************************************************************************
*  Name     : publish_reference_trajectory
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::publish_reference_trajectory()
{
  double V_w = 0.0;
  Eigen::Vector2d wind_direction; wind_direction << 1.0, 0.0;
  /***********************************************************************************
   * Preprocess own-ship state information
  ***********************************************************************************/
  #if (OWNSHIP_TYPE == 0)
    ownship_state.resize(4);
  #else
    ownship_state.resize(6);
  #endif

  /***********************************************************************************
   * Preprocess obstacle information
  ***********************************************************************************/
  Eigen::Matrix<double, 4, -1> static_obstacles; static_obstacles.resize(4, 0);

  obstacle_manager.operator()(
    psbmpc.pars, 
    ownship_state, 
    asv_sim.get_length(),
    obstacle_states, 
    obstacle_covariances);

  /***********************************************************************************
   * Run the COLAV algorithm
  ***********************************************************************************/
  auto start = std::chrono::system_clock::now();		

  psbmpc.calculate_optimal_offsets(
    u_opt,
    chi_opt, 
    predicted_trajectory,
    u_d,
    chi_d,
    waypoints,
    ownship_state,
    V_w,
    wind_direction,
    relevant_polygons,
    obstacle_manager.get_data());

  auto end = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  double mean_t = elapsed.count();

  std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;

  std::cout << "u_d = " << u_d << " | chi_d = " << chi_d << std::endl;

  /***********************************************************************************
   * Publish the results, either as an Offset, Trajectory2 or Trajectory6 message
  ***********************************************************************************/
  obstacle_manager.update_obstacle_status(ownship_state);
  obstacle_manager.display_obstacle_information();

  int n_samples = predicted_trajectory.cols();
  psbmpc_interfaces::msg::Reference reference_k; // r_k = [x, y, psi, u, v, r]^T
  psbmpc_interfaces::msg::Trajectory4 trajectory_msg;
  

  for (int k = 0; k < n_samples; k++)
  {
    reference_k.north = predicted_trajectory(0, k);
    reference_k.east = predicted_trajectory(1, k);
    reference_k.course = predicted_trajectory(2, k);
    #if (OWNSHIP_TYPE == 0)
      reference_k.speed = predicted_trajectory(3, k);
    #else
      reference_k.speed = sqrt(pow(predicted_trajectory(3, k), 2) + pow(predicted_trajectory(4, k), 2));
    #endif

    trajectory_msg.predicted_trajectory.push_back(reference_k);
  }
  trajectory_publisher.publish(trajectory_msg);
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_configure(const rclcpp_lifecycle::State &previous_state) 
{
  RCLCPP_INFO(get_logger(), "Configuring");

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_activate(const rclcpp_lifecycle::State &previous_state) 
{
  RCLCPP_INFO(get_logger(), "Activating");

  pub_->on_activate();
  controller_state_pub_->on_activate();
  timer_->reset();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_deactivate(const rclcpp_lifecycle::State &previous_state) 
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  
  state_subscription->on_deactivate();
  trajectory_publisher->on_deactivate();
  timer->cancel();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_cleanup(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_shutdown(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
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