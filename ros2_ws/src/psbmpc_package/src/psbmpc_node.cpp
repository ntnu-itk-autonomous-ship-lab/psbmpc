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
  const rclcpp::NodeOptions &options                    // In: Configuration options for the node
  )
  : LifecycleNode(node_name, options), 
  map_data_filename(declare_parameter("grounding_hazard_manager.map_data_filename").get<std::string>()),
  map_origin(declare_parameter("grounding_hazard_manager.map_origin").get<std::vector<double>>()),
  enable_topic_stats(declare_parameter("enable_topic_stats").get<bool>()),
  topic_stats_topic_name{declare_parameter("topic_stats_topic_name").get<std::string>()},
  dynamic_obstacle_topic_name(declare_parameter("dynamic_obstacle_topic_name").get<std::string>()),
  state_topic_name(declare_parameter("state_topic_name").get<std::string>()),
  waypoints_topic_name(declare_parameter("waypoints_topic_name").get<std::string>()),
  reference_topic_name(declare_parameter("reference_topic_name").get<std::string>()),
  topic_stats_publish_period(std::chrono::milliseconds{declare_parameter("topic_stats_publish_period_ms").get<std::uint16_t>()}),
  deadline_duration(std::chrono::milliseconds{declare_parameter("deadline_duration_ms").get<std::uint16_t>()}),
  trajectory_publish_period(std::chrono::milliseconds{declare_parameter("trajectory_publish_period_ms").get<std::uint16_t>()}),
  n_missed_deadlines_do_sub(0U),
  n_missed_deadlines_state_sub(0U),
  n_missed_deadlines_wps_sub(0U),
  n_missed_deadlines_pub(0U),
  pars(
    PSBMPC_LIB::CPU::parse_VVD(declare_parameter("psbmpc.u_offsets").get<std::string>()),
    PSBMPC_LIB::CPU::parse_VVD(declare_parameter("psbmpc.chi_offsets").get<std::string>()),
    static_cast<PSBMPC_LIB::CPE_Method>(declare_parameter("psbmpc.cpe_method").get<int>()),
    static_cast<PSBMPC_LIB::Prediction_Method>(declare_parameter("psbmpc.prediction_method").get<int>()),
    static_cast<PSBMPC_LIB::Guidance_Method>(declare_parameter("psbmpc.guidance_method").get<int>()),
    declare_parameter("psbmpc.ipars").get<std::vector<int>>(),
    declare_parameter("psbmpc.dpars").get<std::vector<double>>()),
  cpe(
    static_cast<PSBMPC_LIB::CPE_Method>(declare_parameter("psbmpc.cpe_method").get<int>()),
    declare_parameter("cpe.n_CE").get<int>(),
    declare_parameter("cpe.n_MCSKF").get<int>(),
    declare_parameter("cpe.alpha_n").get<double>(),
    declare_parameter("cpe.gate").get<double>(),
    declare_parameter("cpe.rho").get<double>(),
    declare_parameter("cpe.max_it").get<double>(),
    declare_parameter("cpe.q").get<double>(),
    declare_parameter("cpe.r").get<double>()),
  #if (OWNSHIP_TYPE == 0)
    ownship(
      declare_parameter("ownship.l").get<double>(),
      declare_parameter("ownship.w").get<double>(), 
      declare_parameter("ownship.T_U").get<double>(), 
      declare_parameter("ownship.T_chi").get<double>(), 
      declare_parameter("ownship.R_a").get<double>(), 
      declare_parameter("ownship.LOS_LD").get<double>(), 
      declare_parameter("ownship.LOS_K_i").get<double>())
  #endif,
  psbmpc(pars, ownship, cpe), 
  obstacle_manager(
    declare_parameter("obstacle_manager.T_lost_limit").get<double>(), 
    declare_parameter("obstacle_manager.T_tracked_limit").get<double>(), 
    declare_parameter("obstacle_manager.obstacle_filter_on").get<bool>()),
  obstacle_predictor(
    declare_parameter("obstacle_predictor.r_ct").get<double>(),
    declare_parameter("obstacle_predictor.sigma_x").get<double>(),
    declare_parameter("obstacle_predictor.sigma_xy").get<double>(),
    declare_parameter("obstacle_predictor.sigma_y").get<double>(),
    declare_parameter("obstacle_predictor.gamma_x").get<double>(),
    declare_parameter("obstacle_predictor.gamma_y").get<double>(),
    pars),
  grounding_hazard_manager(map_data_filename, map_origin, psbmpc)
{
  //=======================================================================
  // Creating dynamic obstacle subscriber
  //=======================================================================
  rclcpp::SubscriptionOptions do_subscription_options;
  do_subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &) -> void 
  {
    n_missed_deadlines_do_sub++;
  };
  if (enable_topic_stats)
  {
    do_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    do_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name;
    do_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period;
  }

  dynamic_obstacle_subscription = this->create_subscription<psbmpc_interfaces::msg::DynamicObstacleEstimates>(
    dynamic_obstacle_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::dynamic_obstacle_callback, this, std::placeholders::_1),
    do_subscription_options);

  //=======================================================================
  // Creating own-ship state subscriber
  //=======================================================================
  rclcpp::SubscriptionOptions state_subscription_options;
  state_subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &) -> void 
  {
    n_missed_deadlines_state_sub++;
  };
  if (enable_topic_stats)
  {
    state_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    state_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name;
    state_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period;
  }

  state_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
    state_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::state_callback, this, std::placeholders::_1),
    state_subscription_options);

  //=======================================================================
  // Creating waypoints subscriber
  //=======================================================================
  rclcpp::SubscriptionOptions waypoints_subscription_options;
  waypoints_subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &) -> void 
  {
    n_missed_deadlines_wps_sub++;
  };
  if (enable_topic_stats)
  {
    waypoints_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    waypoints_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name;
    waypoints_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period;
  }

  waypoints_subscription = this->create_subscription<psbmpc_interfaces::msg::Trajectory2>(
    waypoints_topic_name, 
    rclcpp::QoS(1), 
    std::bind(&PSBMPC_Node::waypoints_callback, this, std::placeholders::_1),
    waypoints_subscription_options);  

  //=======================================================================
  // Creating PSB-MPC trajectory publisher
  //=======================================================================
  rclcpp::PublisherOptions trajectory_publisher_options;
  trajectory_publisher_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineOfferedInfo &) -> void 
  {
    n_missed_deadlines_pub++;
  };

  trajectory_publisher = this->create_publisher<psbmpc_interfaces::msg::Trajectory4>(
    reference_topic_name, 
    rclcpp::QoS(1).deadline(deadline_duration),
    trajectory_publisher_options);

  timer = this->create_wall_timer(trajectory_publish_period, std::bind(&PSBMPC_Node::publish_reference_trajectory, this));
}

/****************************************************************************************
*  Name     : obstacle_callback
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::dynamic_obstacle_callback(
  const psbmpc_interfaces::msg::DynamicObstacleEstimates::SharedPtr msg   // Dynamic obstacle information (ID, state and covariances)
  )
{
  int n_obst = msg->obstacle_ids.size();
  obstacle_states.resize(9, n_obst);
  obstacle_covariances.resize(16, n_obst);

  int ID;
  double A, B, C, D;
  Eigen::Vector4d xs_i;
  Eigen::Matrix4d P;
  Eigen::Matrix2d pos_vel_cov;
  for (int i = 0; i < n_obst; i++)
  {
    ID = msg->obstacle_ids[i];
    A = msg->obstacle_dimensions[i].x;
    B = msg->obstacle_dimensions[i].y;
    C = msg->obstacle_dimensions[i].z;
    D = msg->obstacle_dimensions[i].w;
    xs_i(0) = msg->obstacle_estimates[i].pos_est.x;
    xs_i(1) = msg->obstacle_estimates[i].pos_est.y;
    xs_i(2) = msg->obstacle_estimates[i].vel_est.x;
    xs_i(3) = msg->obstacle_estimates[i].vel_est.y;
    obstacle_states.col(i) << xs_i, A, B, C, D, ID;

    P(0, 0) = msg->obstacle_estimates[i].pos_cov.var_x;
    P(0, 1) = msg->obstacle_estimates[i].pos_cov.cor_xy;
    P(1, 0) = msg->obstacle_estimates[i].pos_cov.cor_xy;
    P(1, 1) = msg->obstacle_estimates[i].pos_cov.var_y;

    P(2, 2) = msg->obstacle_estimates[i].vel_cov.var_x;
    P(2, 3) = msg->obstacle_estimates[i].vel_cov.cor_xy;
    P(2, 2) = msg->obstacle_estimates[i].vel_cov.cor_xy;
    P(3, 3) = msg->obstacle_estimates[i].vel_cov.var_y;

    pos_vel_cov(0, 0) = msg->obstacle_estimates[i].pos_vel_corr.cor_px_vx;
    pos_vel_cov(0, 1) = msg->obstacle_estimates[i].pos_vel_corr.cor_px_vy;
    pos_vel_cov(1, 0) = msg->obstacle_estimates[i].pos_vel_corr.cor_py_vx;
    pos_vel_cov(1, 1) = msg->obstacle_estimates[i].pos_vel_corr.cor_py_vy;

    P.block<2, 2>(0, 2) = pos_vel_cov;
    P.block<2, 2>(2, 0) = pos_vel_cov;
    obstacle_covariances.col(i) << PSBMPC_LIB::CPU::flatten(P);
  }
}

/****************************************************************************************
*  Name     : state_callback
*  Function :
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::state_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg                      // In: State message
  )
{
  double heading(0.0), SOG(0.0);
  Eigen::Vector4d q;
  q(0) = msg->pose.pose.orientation.x;
  q(1) = msg->pose.pose.orientation.y;
  q(2) = msg->pose.pose.orientation.z;
  q(3) = msg->pose.pose.orientation.w;
  heading = atan2(2.0 * (q(3) * q(0) + q(1) * q(2)) , - 1.0 + 2.0 * (q(0) * q(0) + q(1) * q(1)));
  #if OWNSHIP_TYPE == 0
    ownship_state.resize(4);
    ownship_state(0) = msg->pose.pose.position.x;
    ownship_state(1) = msg->pose.pose.position.y;
    ownship_state(2) = heading; // approximate course to heading, as no crab angle info is available
    SOG = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    ownship_state(3) = SOG;
  #else
    ownship_state.resize(6);
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
  const psbmpc_interfaces::msg::Trajectory2::SharedPtr msg         // In: Waypoint array message     
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

  relevant_polygons = grounding_hazard_manager(ownship_state);

  obstacle_manager(ownship_state, ownship_length, obstacle_states, obstacle_covariances, psbmpc);

  obstacle_predictor(obstacle_manager.get_data(), ownship_state, psbmpc);

  ownship.determine_active_waypoint_segment(waypoints, ownship_state);

  ownship.update_guidance_references(u_d, chi_d, waypoints, ownship_state, 0.0, PSBMPC_LIB::LOS);
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
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);

  mean_t = elapsed.count();

  //std::cout << "PSBMPC time usage: " << mean_t << " seconds" << std::endl;

  //std::cout << "u_d = " << u_d << " | chi_d = " << chi_d << std::endl;

  obstacle_manager.update_obstacle_status(ownship_state);
  obstacle_manager.display_obstacle_information();

  int n_samples = predicted_trajectory.cols();
  psbmpc_interfaces::msg::Reference reference_k; // r_k = [x, y, chi, U]^T
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
  trajectory_publisher->publish(trajectory_msg);
}

/****************************************************************************************
*  Name     : log_psbmpc_information
*  Function : 
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC_Node::log_psbmpc_information()
{
  #if OWNSHIP_TYPE
    RCLCPP_INFO(get_logger(), "Input Own-ship state = %lf, %lf, %lf, %lf, %lf, %lf", ownship_state(0), ownship_state(1), ownship_state(2),
      ownship_state(3), ownship_state(4), ownship_state(5));
  #else
    RCLCPP_INFO(get_logger(), "Input Own-ship state = %lf, %lf, %lf, %lf", ownship_state(0), ownship_state(1), ownship_state(2), ownship_state(3));
  #endif

  RCLCPP_INFO(get_logger(), "Optimal offsets = %lf, %lf", u_opt, chi_opt);
  RCLCPP_INFO(get_logger(), "PSB-MPC run-time = %lf", mean_t);
  
  RCLCPP_INFO(get_logger(), "DO subscription missed deadlines = %lu", n_missed_deadlines_do_sub);
  RCLCPP_INFO(get_logger(), "Own-ship state subscription missed deadlines = %lu", n_missed_deadlines_state_sub);
  RCLCPP_INFO(get_logger(), "Waypoints subscription missed deadlines = %lu", n_missed_deadlines_wps_sub);
  RCLCPP_INFO(get_logger(), "Trajectory publisher missed deadlines = %lu", n_missed_deadlines_pub);
}

/****************************************************************************************
*  Name     : on_xxx
*  Function : Implements the transitioning functions for the ROS2 lifecycle node
*  Author   :
*  Modified :
*****************************************************************************************/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_configure(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Configuring");

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_activate(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Activating");

  trajectory_publisher->on_activate();
  timer->reset();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PSBMPC_Node::on_deactivate(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Deactivating");

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

    rclcpp_lifecycle::LifecycleNode::SharedPtr psbmpc_node_ptr = std::make_shared<rclcpp_lifecycle::LifecycleNode>("psbmpc_node");

    rclcpp::executors::StaticSingleThreadedExecutor executor;

    executor.add_node(psbmpc_node_ptr->get_node_base_interface());
    
    executor.spin();

    rclcpp::shutdown();
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("PSB-MPC"), e.what());
    r = 3;
  } 
  catch (...) 
  {
    RCLCPP_INFO(rclcpp::get_logger("PSB-MPC"), "Unknown exception caught. ""Exiting...");
    r = -1;
  }

  return r; 
}