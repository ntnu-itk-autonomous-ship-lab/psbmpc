#include "ghm_node.hpp"

#include <ctime>
#include <exception>
#include <iostream>
#include <limits>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

/****************************************************************************************
 *  Name     : GHM_Node
 *  Function : Class constructor, initializes parameters, variables and objects
 *  Method   :
 *  Author   :
 *****************************************************************************************/
GHM_Node::GHM_Node(ros::NodeHandle &nh // In: Handle to the GHM node,
                                       // initialized in the main loop
                   )
    : nh(nh),
      synchronizer(ownship_pose_subscriber, ownship_velocity_subscriber, 1) {
  //=========================================================================
  // Fetch parameters from the server, initialize relevant objects
  //=========================================================================
  psbmpc_mode = nh.param<std::string>("initial_psbmpc_mode", "psbmpc");

  polygons_topic_name = nh.param<std::string>("polygons_topic_name", "");
  ownship_pose_topic_name =
      nh.param<std::string>("ownship_pose_topic_name", "");
  ownship_velocity_topic_name =
      nh.param<std::string>("ownship_velocity_topic_name", "");
  get_polygons_service_name =
      nh.param<std::string>("get_polygons_service_name", "");
  psbmpc_mode_topic_name = nh.param<std::string>("psbmpc_mode_topic_name", "");

  polygons_publish_rate = nh.param<double>("polygons_publish_rate", 0.001);

  ownship_state.resize(0);

  std::string u_offsets, chi_offsets;
  std::vector<double> dpars;
  std::vector<int> ipars;
  int cpe_method(0), prediction_method(0), guidance_method(0);
  u_offsets =
      nh.param<std::string>("/psbmpc/sbmpc_parameters/u_offsets", u_offsets);
  chi_offsets = nh.param<std::string>("/psbmpc/sbmpc_parameters/chi_offsets",
                                      chi_offsets);
  cpe_method = nh.param<int>("/psbmpc/sbmpc_parameters/cpe_method", 100);
  prediction_method =
      nh.param<int>("/psbmpc/sbmpc_parameters/prediction_method", 100);
  guidance_method =
      nh.param<int>("/psbmpc/sbmpc_parameters/guidance_method", 100);
  ipars = nh.param<std::vector<int>>("/psbmpc/sbmpc_parameters/ipars", ipars);
  dpars =
      nh.param<std::vector<double>>("/psbmpc/sbmpc_parameters/dpars", dpars);

  sbmpc_pars = PSBMPC_LIB::SBMPC_Parameters(
      PSBMPC_LIB::CPU::parse_VVD(u_offsets),
      PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
      static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
      static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method), ipars, dpars);

  u_offsets =
      nh.param<std::string>("/psbmpc/psbmpc_parameters/u_offsets", u_offsets);
  chi_offsets = nh.param<std::string>("/psbmpc/psbmpc_parameters/chi_offsets",
                                      chi_offsets);
  cpe_method = nh.param<int>("/psbmpc/psbmpc_parameters/cpe_method", 1);
  prediction_method =
      nh.param<int>("/psbmpc/psbmpc_parameters/prediction_method", 1);
  guidance_method =
      nh.param<int>("/psbmpc/psbmpc_parameters/guidance_method", 1);
  ipars = nh.param<std::vector<int>>("/psbmpc/psbmpc_parameters/ipars", ipars);
  dpars =
      nh.param<std::vector<double>>("/psbmpc/psbmpc_parameters/dpars", dpars);

  psbmpc_pars = PSBMPC_LIB::PSBMPC_Parameters(
      PSBMPC_LIB::CPU::parse_VVD(u_offsets),
      PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
      static_cast<PSBMPC_LIB::CPE_Method>(cpe_method),
      static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
      static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method), ipars, dpars);

  std::string package_path = ros::package::getPath("psbmpc");
  std::string local_ned_frame_lla_parameter_name{""};
  std::string map_data_filename{""};
  Eigen::Vector2d lla_origin;
  std::vector<double> frame_origin;
  double equatorial_radius(0.0), flattening_factor(0.0);
  int utm_zone(1);
  map_data_filename =
      nh.param<std::string>("map_data_filename", map_data_filename);
  equatorial_radius =
      nh.param<double>("coordinate_frame_parameters/equatorial_radius", 0.0);
  flattening_factor =
      nh.param<double>("coordinate_frame_parameters/flattening_factor", 0.0);
  utm_zone = nh.param<int>("coordinate_frame_parameters/utm_zone", 32);
  local_ned_frame_name = nh.param<std::string>(
      "coordinate_frame_parameters/reference_frame/name", "");
  local_ned_frame_name = nh.param<std::string>(local_ned_frame_name, "");
  local_ned_frame_lla_parameter_name = nh.param<std::string>(
      "coordinate_frame_parameters/reference_frame/lla", "");
  frame_origin = nh.param<std::vector<double>>(
      local_ned_frame_lla_parameter_name, frame_origin);
  lla_origin(0) = frame_origin[0];
  lla_origin(1) = frame_origin[1];

  if (psbmpc_mode == "sbmpc") {
    grounding_hazard_manager = PSBMPC_LIB::Grounding_Hazard_Manager(
        "", // package_path + "/" + map_data_filename,
        equatorial_radius, flattening_factor, utm_zone, true, lla_origin,
        local_ned_frame_name, sbmpc_pars);
  } else if (psbmpc_mode == "psbmpc") {
    grounding_hazard_manager = PSBMPC_LIB::Grounding_Hazard_Manager(
        "", // package_path + "/" + map_data_filename,
        equatorial_radius, flattening_factor, utm_zone, true, lla_origin,
        local_ned_frame_name, psbmpc_pars);
  }

  map_data_filename = nh.param<std::string>("extra_map_data_filename", "");
  grounding_hazard_manager.read_other_polygons(
      package_path + "/" + map_data_filename, true, false);
  polygons_ned = grounding_hazard_manager.get_polygons_ned();
  simplified_polygons_ned =
      grounding_hazard_manager.get_simplified_polygons_ned();

  //=========================================================================
  // Initialize clients, services, publishers and subscribers
  //=========================================================================
  polygons_publisher =
      nh.advertise<custom_msgs::Polygons>(polygons_topic_name, 1);
  polygons_publish_timer =
      nh.createTimer(ros::Duration(1.0 / polygons_publish_rate),
                     std::bind(&GHM_Node::publish_polygons, this));

  ownship_pose_subscriber.subscribe(nh, ownship_pose_topic_name, 1);
  ownship_velocity_subscriber.subscribe(nh, ownship_velocity_topic_name, 1);
  synchronizer.registerCallback(
      boost::bind(&GHM_Node::ownship_state_callback, this, _1, _2));

  psbmpc_mode_subscriber = nh.subscribe(psbmpc_mode_topic_name, 1,
                                        &GHM_Node::psbmpc_mode_callback, this);

  get_polygons_server = nh.advertiseService(
      get_polygons_service_name, &GHM_Node::get_polygons_handler, this);
}

/****************************************************************************************
 *  Name     : publish_polygons
 *  Function : Publishes relevant local polygons from map data
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void GHM_Node::publish_polygons() {
  bool initialized = ownship_state.size() >= 4;
  if (!initialized) {
    return;
  }

  std::scoped_lock ghm_data_lock(ownship_data_mutex, polygon_data_mutex,
                                 psbmpc_mode_data_mutex);

  if (psbmpc_mode == "sbmpc") {
    relevant_polygons_ned = grounding_hazard_manager(
        ownship_state, sbmpc_pars.get_dpar(i_dpar_d_so_relevant_SBMPC));
  } else if (psbmpc_mode == "psbmpc") {
    relevant_polygons_ned = grounding_hazard_manager(
        ownship_state, psbmpc_pars.get_dpar(i_dpar_d_so_relevant));
  }

  custom_msgs::Polygons polygons_msg;
  geometry_msgs::Point32 point;
  int n_polygons = relevant_polygons_ned.size();
  for (int j = 0; j < n_polygons; j++) {
    geometry_msgs::Polygon polygon;
    for (auto it = boost::begin(
             boost::geometry::exterior_ring(relevant_polygons_ned[j]));
         it !=
         boost::end(boost::geometry::exterior_ring(relevant_polygons_ned[j]));
         it++) {
      point.x = boost::geometry::get<0>(*it);
      point.y = boost::geometry::get<1>(*it);
      point.z = 0.0;
      polygon.points.push_back(point);
    }
    polygons_msg.polygons.push_back(polygon);
  }
  polygons_publisher.publish(polygons_msg);
}

/****************************************************************************************
 *  Name     : ownship_state_callback
 *  Function : Updates the ownship state
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void GHM_Node::ownship_state_callback(
    const geometry_msgs::PoseStamped::ConstPtr
        &pose_msg, // In: ROS message containing ownship pose
    const geometry_msgs::TwistStamped::ConstPtr
        &twist_msg // In: ROS message containing ownship twist in body
) {
  std::scoped_lock ownship_data_lock(ownship_data_mutex);

  tf::Quaternion q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y,
                   pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double heading = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(yaw);
  // ROS_INFO("HEADING = %.5f\n", RAD2DEG *
  // PSBMPC_LIB::CPU::wrap_angle_to_02pi(heading));

  Eigen::Vector3d nu;
  nu(0) = twist_msg->twist.linear.x;
  nu(1) = twist_msg->twist.linear.y;
  nu(2) = twist_msg->twist.angular.z;

#if OWNSHIP_TYPE == 0
  ownship_state.resize(4);
  ownship_state(0) = pose_msg->pose.position.x;
  ownship_state(1) = pose_msg->pose.position.y;
  double crab_angle(0.0);
  if (fabs(nu(0)) < 1e-02 && nu(1) < 1e-02) {
    crab_angle = 0.0;
  } else {
    crab_angle = 0.0; // atan2(nu(1), nu(0));
  }
  ownship_state(2) = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(heading + crab_angle);
  double speed_over_ground = sqrt(pow(nu(0), 2) + pow(nu(1), 2));
  ownship_state(3) = speed_over_ground;
#else
  ownship_state.resize(6);
  ownship_state(0) = pose_msg->pose.position.x;
  ownship_state(1) = pose_msg->pose.position.y;
  ownship_state(2) = heading;
  ownship_state(3) = nu(0);
  ownship_state(4) = nu(1);
  ownship_state(5) = nu(2);
#endif
}

/****************************************************************************************
 *  Name     : get_polygons_handler
 *  Function : Sends all polygons in the map data back to the client.
 *  Method   :
 *  Author   :
 *****************************************************************************************/
bool GHM_Node::get_polygons_handler(
    psbmpc::GetPolygons::Request
        &req, // In: Request information for getting polygons (empty)
    psbmpc::GetPolygons::Response
        &res // Out: Response information after sending polygons
) {
  try {
    std::scoped_lock polygon_server_data_lock(ownship_data_mutex,
                                              polygon_data_mutex);

    bool initialized = ownship_state.size() >= 4;
    if (!initialized) {
      return false;
    }

    double d_so_relevant = req.d_so_relevant;
    PSBMPC_LIB::Static_Obstacles response_polygons =
        grounding_hazard_manager(ownship_state, d_so_relevant);

    custom_msgs::Polygons polygons_msg;
    int n_polygons = response_polygons.size();

    geometry_msgs::Point32 point;
    for (int j = 0; j < n_polygons; j++) {
      geometry_msgs::Polygon polygon;
      for (auto it = boost::begin(
               boost::geometry::exterior_ring(response_polygons[j]));
           it !=
           boost::end(boost::geometry::exterior_ring(response_polygons[j]));
           it++) {
        point.x = boost::geometry::get<0>(*it);
        point.y = boost::geometry::get<1>(*it);
        point.z = 0.0;
        polygon.points.push_back(point);
      }
      polygons_msg.polygons.push_back(polygon);
    }
    res.polygons = polygons_msg;
    res.success = true;
    return true;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}

/****************************************************************************************
 *  Name     : main
 *  Function : Sets up, initializes, runs the GHM ROS node, and shuts it down.
 *  Method   :
 *  Author   :
 *****************************************************************************************/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ghm_node");
  ros::start();

  ROS_INFO("Start GHM node initialization...");
  ros::NodeHandle nh("~");
  GHM_Node ghm_node(nh);
  ROS_INFO("GHM node initialization complete.");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}