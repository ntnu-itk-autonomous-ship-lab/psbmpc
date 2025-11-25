#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include "custom_msgs/Polygons.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "psbmpc/GetPolygons.h"
#include "std_msgs/String.h"

#include <mutex>
#include <string>

#include "grounding_hazard_manager.hpp"
#include "psbmpc_parameters.hpp"
#include "sbmpc_parameters.hpp"

class GHM_Node {
private:
  ros::NodeHandle nh;
  //==================================================
  // Clients, subscribers and publishers
  //==================================================
  ros::Publisher polygons_publisher;
  ros::Timer polygons_publish_timer;

  ros::Subscriber psbmpc_mode_subscriber;
  message_filters::Subscriber<geometry_msgs::PoseStamped>
      ownship_pose_subscriber;
  message_filters::Subscriber<geometry_msgs::TwistStamped>
      ownship_velocity_subscriber;
  message_filters::TimeSynchronizer<geometry_msgs::PoseStamped,
                                    geometry_msgs::TwistStamped>
      synchronizer;

  ros::ServiceServer get_polygons_server;

  std::string polygons_topic_name;
  std::string ownship_pose_topic_name;
  std::string ownship_velocity_topic_name;
  std::string get_polygons_service_name;
  std::string psbmpc_mode_topic_name;

  double polygons_publish_rate;

  std::string local_ned_frame_name;

  //==================================================
  // PODs, data structures and classes for use by the node
  //==================================================
  std::string psbmpc_mode;
  Eigen::Vector3d reference_frame_lla;
  Eigen::VectorXd ownship_state;

  PSBMPC_LIB::Static_Obstacles polygons_ned, simplified_polygons_ned,
      relevant_polygons_ned;

  PSBMPC_LIB::SBMPC_Parameters sbmpc_pars;
  PSBMPC_LIB::PSBMPC_Parameters psbmpc_pars;

  PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager;

  std::mutex ownship_data_mutex, polygon_data_mutex, psbmpc_mode_data_mutex;
  //====================================================

public:
  GHM_Node(ros::NodeHandle &nh);

  void publish_polygons();

  void ownship_state_callback(
      const geometry_msgs::PoseStamped::ConstPtr &pose_msg,
      const geometry_msgs::TwistStamped::ConstPtr &twist_msg);

  void psbmpc_mode_callback(const std_msgs::String::ConstPtr &psbmpc_mode_msg) {
    std::scoped_lock psbmpc_mode_data_lock(psbmpc_mode_data_mutex);
    psbmpc_mode = psbmpc_mode_msg->data;
  }

  bool get_polygons_handler(psbmpc::GetPolygons::Request &req,
                            psbmpc::GetPolygons::Response &res);
};