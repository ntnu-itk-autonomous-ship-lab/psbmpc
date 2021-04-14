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

#include <chrono>
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

  timer = this->create_wall_timer(1s, )
}

/****************************************************************************************
*  Name     : run_psbmpc
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
PSBMPC_Node::publish_reference_trajectory()
{
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
    obstacle_covariances, 
    obstacle_intention_probabilities, 
    obstacle_a_priori_CC_probabilities);

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
    static_obstacles,
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
    #if (OWNSHIP_TYPE == 0)
      reference_k.north = predicted_trajectory(0, k);
      reference_k.east = predicted_trajectory(1, k);
      reference_k.course = predicted_trajectory(2, k);
      reference_k.speed = predicted_trajectory(3, k);
    #else
      reference_k.configuration.x = predicted_trajectory(0, k);
      reference_k.configuration.y = predicted_trajectory(1, k);
      reference_k.configuration.z = predicted_trajectory(2, k); 
    #endif

    trajectory_msg.predicted_trajectory.push_back(reference_k);
  }
  trajectory_publisher.publish(trajectory_msg)
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