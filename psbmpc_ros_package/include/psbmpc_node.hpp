#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include "custom_msgs/NorthEastHeading.h"
#include "custom_msgs/Polygons.h"
#include "custom_msgs/Trajectory4.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "psbmpc/LoadWaypoints.h"
#include "psbmpc/SetPSBMPCMode.h"
#include "ros_af_msgs/DynamicObstaclesData.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "cpu/psbmpc_cpu.hpp"
#include "sbmpc.hpp"
#include <yaml-cpp/yaml.h>
#if USE_GPU_PSBMPC
#include "gpu/psbmpc_gpu.cuh"
#endif

class PSBMPC_Node {
private:
  ros::NodeHandle nh;
  //==================================================
  // Clients, subscribers and publishers
  //==================================================
  ros::Publisher psbmpc_mode_publisher;
  ros::Publisher trajectory_publisher;
  ros::Timer psbmpc_mode_publish_timer;
  ros::Timer trajectory_publish_timer;

  ros::Subscriber polygons_subscriber;
  ros::Subscriber do_data_subscriber;
  message_filters::Subscriber<geometry_msgs::PoseStamped>
      ownship_pose_subscriber;
  message_filters::Subscriber<geometry_msgs::TwistStamped>
      ownship_velocity_subscriber;
  message_filters::TimeSynchronizer<geometry_msgs::PoseStamped,
                                    geometry_msgs::TwistStamped>
      synchronizer;
  ros::Subscriber waypoint_subscriber;
  ros::Subscriber wind_subscriber;
  ros::Subscriber supervisor_mode_subscriber;

  ros::ServiceServer set_psbmpc_mode_service;
  ros::ServiceServer waypoint_loader_service;

  std::string polygons_topic_name;
  std::string do_data_topic_name;
  std::string ownship_pose_topic_name;
  std::string ownship_velocity_topic_name;
  std::string waypoint_topic_name;
  std::string psbmpc_mode_topic_name;
  std::string trajectory_reference_topic_name;
  std::string wind_topic_name;
  std::string supervisor_mode_topic_name;

  std::string set_psbmpc_mode_service_name;
  std::string waypoint_loader_service_name;

  double psbmpc_mode_publish_rate, trajectory_publish_rate;
  double trajectory_downsampling_factor;

  std::string local_ned_frame_name;

  //==================================================
  // PODs, data structures and classes for use by the node
  //==================================================
  std::mutex ownship_data_mutex, obstacles_data_mutex, polygons_data_mutex,
      wp_data_mutex, wind_data_mutex, supervisor_mode_data_mutex,
      psbmpc_mode_data_mutex;

  std::string supervisor_mode, psbmpc_mode;
  double ownship_length;
  double V_w, u_opt, chi_opt, mean_t;
  Eigen::MatrixXd predicted_trajectory;

  std::vector<double> U_d = {1.0}; // Default
  double chi_d;
  Eigen::Vector2d wind_direction;
  Eigen::Vector3d reference_frame_lla;
  Eigen::VectorXd ownship_state;

  Eigen::MatrixXd waypoints;

  PSBMPC_LIB::SBMPC_Parameters sbmpc_pars;
  PSBMPC_LIB::PSBMPC_Parameters psbmpc_pars;

  PSBMPC_LIB::CPU::CPE cpe;
  PSBMPC_LIB::CPU::Ownship ownship;
  PSBMPC_LIB::SBMPC sbmpc;
#if USE_GPU_PSBMPC
  PSBMPC_LIB::GPU::PSBMPC psbmpc;
#else
  PSBMPC_LIB::CPU::PSBMPC psbmpc;
#endif

  PSBMPC_LIB::Static_Obstacles polygons;
  PSBMPC_LIB::Dynamic_Obstacles obstacles;
  //====================================================

public:
  PSBMPC_Node(ros::NodeHandle &nh);

  void publish_psbmpc_mode();

  void publish_reference_trajectory();

  void polygons_callback(const custom_msgs::Polygons::ConstPtr &msg);

  void do_data_callback(const ros_af_msgs::DynamicObstaclesData::ConstPtr &msg);

  void ownship_state_callback(
      const geometry_msgs::PoseStamped::ConstPtr &pose_msg,
      const geometry_msgs::TwistStamped::ConstPtr &twist_msg);

  void waypoint_callback(const custom_msgs::NorthEastHeading::ConstPtr &msg);

  void wind_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);

  void supervisor_mode_callback(const std_msgs::String::ConstPtr &msg) {
    std::scoped_lock supervisor_mode_lock(supervisor_mode_data_mutex,
                                          psbmpc_mode_data_mutex);
    supervisor_mode = msg->data;

    // Temporary:
    /* if (supervisor_mode == "sbmpc")
    {
            psbmpc_mode = "sbmpc";
    }
    else if (supervisor_mode == "psbmpc")
    {
            psbmpc_mode = "psbmpc";
    } */
  }

  bool set_psbmpc_mode_handler(psbmpc::SetPSBMPCMode::Request &req,
                               psbmpc::SetPSBMPCMode::Response &res) {
    std::scoped_lock psbmpc_mode_lock(psbmpc_mode_data_mutex);
    psbmpc_mode = req.psbmpc_mode;
    res.success = true;
    if (psbmpc_mode == "sbmpc") {
      res.status_msg = "Transitioning to SBMPC.";
    } else if (psbmpc_mode == "psbmpc") {
      res.status_msg = "Transitioning to PSBMPC.";
    } else if (psbmpc_mode == "wp-tracking") {
      res.status_msg = "Transitioning to WP-tracking.";
    }
    return res.success;
  }

  bool load_waypoints_handler(psbmpc::LoadWaypoints::Request &req,
                              psbmpc::LoadWaypoints::Response &res);
};