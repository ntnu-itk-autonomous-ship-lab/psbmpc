#include "psbmpc_node.hpp"

#include <ctime>
#include <exception>
#include <iostream>
#include <limits>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

/****************************************************************************************
 *  Name     : PSBMPC_Node
 *  Function : Class constructor, initializes parameters, variables and objects
 *  Method   :
 *  Author   :
 *****************************************************************************************/
PSBMPC_Node::PSBMPC_Node(ros::NodeHandle &nh // In: Handle to the PSBMPC node,
                                             // initialized in the main loop
                         )
    : nh(nh),
      synchronizer(ownship_pose_subscriber, ownship_velocity_subscriber, 1) {
  //=========================================================================
  // Fetch parameters from the server, initialize relevant objects
  //=========================================================================
  psbmpc_mode = nh.param<std::string>("initial_psbmpc_mode", "psbmpc");

  polygons_topic_name = nh.param<std::string>("polygons_topic_name", "");
  do_data_topic_name = nh.param<std::string>("do_data_topic_name", "");
  ownship_pose_topic_name =
      nh.param<std::string>("ownship_pose_topic_name", "");
  ownship_velocity_topic_name =
      nh.param<std::string>("ownship_velocity_topic_name", "");
  waypoint_topic_name = nh.param<std::string>("waypoint_topic_name", "");
  wind_topic_name = nh.param<std::string>("wind_topic_name", "");
  trajectory_reference_topic_name =
      nh.param<std::string>("trajectory_reference_topic_name", "");
  supervisor_mode_topic_name =
      nh.param<std::string>("supervisor_mode_topic_name", "");
  psbmpc_mode_topic_name = nh.param<std::string>("psbmpc_mode_topic_name", "");

  set_psbmpc_mode_service_name =
      nh.param<std::string>("set_psbmpc_mode_service_name", "");
  waypoint_loader_service_name =
      nh.param<std::string>("waypoint_loader_service_name", "");

  psbmpc_mode_publish_rate = nh.param<double>("psbmpc_mode_publish_rate", 0.5);
  trajectory_publish_rate = nh.param<double>("trajectory_publish_rate", 0.5);
  trajectory_downsampling_factor =
      nh.param<double>("trajectory_downsampling_factor", 0.5);

  ownship_length = nh.param<double>("ownship/l", 6);

  V_w = 0.0;
  wind_direction(0) = 1.0;
  wind_direction(1) = 0.0;

  ownship_state.resize(0);
  waypoints.resize(2, 0);

  std::string u_offsets, chi_offsets;
  std::vector<double> dpars;
  std::vector<int> ipars;
  int cpe_method(0), prediction_method(0), guidance_method(0);
  u_offsets = nh.param<std::string>("sbmpc_parameters/u_offsets", u_offsets);
  chi_offsets =
      nh.param<std::string>("sbmpc_parameters/chi_offsets", chi_offsets);
  cpe_method = nh.param<int>("sbmpc_parameters/cpe_method", 100);
  prediction_method = nh.param<int>("sbmpc_parameters/prediction_method", 100);
  guidance_method = nh.param<int>("sbmpc_parameters/guidance_method", 100);
  ipars = nh.param<std::vector<int>>("sbmpc_parameters/ipars", ipars);
  dpars = nh.param<std::vector<double>>("sbmpc_parameters/dpars", dpars);

  sbmpc_pars = PSBMPC_LIB::SBMPC_Parameters(
      PSBMPC_LIB::CPU::parse_VVD(u_offsets),
      PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
      static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
      static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method), ipars, dpars);

  u_offsets = nh.param<std::string>("psbmpc_parameters/u_offsets", u_offsets);
  chi_offsets =
      nh.param<std::string>("psbmpc_parameters/chi_offsets", chi_offsets);
  cpe_method = nh.param<int>("psbmpc_parameters/cpe_method", 1);
  prediction_method = nh.param<int>("psbmpc_parameters/prediction_method", 1);
  guidance_method = nh.param<int>("psbmpc_parameters/guidance_method", 1);
  ipars = nh.param<std::vector<int>>("psbmpc_parameters/ipars", ipars);
  dpars = nh.param<std::vector<double>>("psbmpc_parameters/dpars", dpars);

  psbmpc_pars = PSBMPC_LIB::PSBMPC_Parameters(
      PSBMPC_LIB::CPU::parse_VVD(u_offsets),
      PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
      static_cast<PSBMPC_LIB::CPE_Method>(cpe_method),
      static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
      static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method), ipars, dpars);

  int n_CE(0), n_MCSKF(0), max_it(0);
  double alpha_n(0.0), gate(0.0), rho(0.0), q(0.0), r(0.0);
  n_CE = nh.param<int>("cpe/n_CE", 500);
  n_MCSKF = nh.param<int>("cpe/n_MCSKF", 500);
  alpha_n = nh.param<double>("cpe/alpha_n", 0.9);
  gate = nh.param<double>("cpe/gate", 11.6183);
  rho = nh.param<double>("cpe/rho", 0.9);
  max_it = nh.param<int>("cpe/max_it", 6);
  q = nh.param<double>("cpe/q", 0.0008);
  r = nh.param<double>("cpe/r", 0.001);

  cpe = PSBMPC_LIB::CPU::CPE(static_cast<PSBMPC_LIB::CPE_Method>(cpe_method),
                             n_CE, n_MCSKF, alpha_n, gate, rho, max_it, q, r);

  double w(3.0), T_chi(0.92), T_U(1.44), R_a(5.0), LOS_LD(66.0), LOS_K_i(0.0);
  w = nh.param<double>("ownship/w", w);
  T_chi = nh.param<double>("ownship/T_chi", T_chi);
  T_U = nh.param<double>("ownship/T_U", T_U);
  R_a = nh.param<double>("ownship/R_a", R_a);
  LOS_LD = nh.param<double>("ownship/LOS_LD", LOS_LD);
  LOS_K_i = nh.param<double>("ownship/LOS_K_i", LOS_K_i);

  ownship = PSBMPC_LIB::CPU::Ownship(ownship_length, w, T_chi, T_U, R_a, LOS_LD,
                                     LOS_K_i);
  sbmpc = PSBMPC_LIB::SBMPC(ownship, sbmpc_pars);

#if USE_GPU_PSBMPC
  PSBMPC_LIB::CVE_Pars<float> cve_pars;
  cve_pars.d_init_colregs_situation = nh.param<float>(
      "cve/d_init_colregs_situation", cve_pars.d_init_colregs_situation);
  cve_pars.head_on_width =
      nh.param<float>("cve/head_on_width", cve_pars.head_on_width) * DEG2RAD;
  cve_pars.overtaking_angle =
      nh.param<float>("cve/overtaking_angle", cve_pars.overtaking_angle) *
      DEG2RAD;
  cve_pars.critical_distance_to_ignore_SO =
      nh.param<float>("cve/critical_distance_to_ignore_SO",
                      cve_pars.critical_distance_to_ignore_SO);
  cve_pars.max_acceptable_SO_speed_change =
      nh.param<float>("cve/max_acceptable_SO_speed_change",
                      cve_pars.max_acceptable_SO_speed_change);
  cve_pars.max_acceptable_SO_course_change =
      nh.param<float>("cve/max_acceptable_SO_course_change",
                      cve_pars.max_acceptable_SO_course_change) *
      DEG2RAD;
  cve_pars.min_acceptable_GW_speed_change =
      nh.param<float>("cve/min_acceptable_GW_speed_change",
                      cve_pars.min_acceptable_GW_speed_change);
  cve_pars.min_acceptable_GW_course_change =
      nh.param<float>("cve/min_acceptable_GW_course_change",
                      cve_pars.min_acceptable_GW_course_change) *
      DEG2RAD;
  cve_pars.GW_safety_margin =
      nh.param<float>("cve/GW_safety_margin", cve_pars.GW_safety_margin);
  PSBMPC_LIB::GPU::Ownship ownship_gpu(ownship_length, w, T_chi, T_U, R_a,
                                       LOS_LD, LOS_K_i);
  psbmpc = PSBMPC_LIB::GPU::PSBMPC(ownship_gpu, cpe, psbmpc_pars, cve_pars);
#else
  /* PSBMPC_LIB::CVE_Pars<double> cve_pars;
  cve_pars.max_distance_at_cpa = nh.param<double>("cve/max_distance_at_cpa",
  cve_pars.max_distance_at_cpa); cve_pars.d_init_colregs_situation =
  nh.param<double>("cve/d_init_colregs_situation",
  cve_pars.d_init_colregs_situation); cve_pars.head_on_width =
  nh.param<double>("cve/head_on_width", cve_pars.head_on_width);
  cve_pars.overtaking_angle = nh.param<double>("cve/overtaking_angle",
  cve_pars.overtaking_angle); cve_pars.max_acceptable_SO_speed_change =
  nh.param<double>("cve/max_acceptable_SO_speed_change",
  cve_pars.max_acceptable_SO_speed_change);
  cve_pars.max_acceptable_SO_course_change =
  nh.param<double>("cve/max_acceptable_SO_course_change",
  cve_pars.max_acceptable_SO_course_change);
  cve_pars.critical_distance_to_ignore_SO =
  nh.param<double>("cve/critical_distance_to_ignore_SO",
  cve_pars.critical_distance_to_ignore_SO); cve_pars.GW_safety_margin =
  nh.param<double>("cve/max_acceptable_SO_course_change",
  cve_pars.GW_safety_margin); */
  psbmpc = PSBMPC_LIB::CPU::PSBMPC(ownship, cpe, psbmpc_pars);
#endif

  obstacles.resize(0);
  polygons.resize(0);
  //=========================================================================
  // Initialize clients, services, publishers and subscribers
  //=========================================================================
  psbmpc_mode_publisher =
      nh.advertise<std_msgs::String>(psbmpc_mode_topic_name, 1);
  psbmpc_mode_publish_timer =
      nh.createTimer(ros::Duration(1.0 / psbmpc_mode_publish_rate),
                     std::bind(&PSBMPC_Node::publish_psbmpc_mode, this));

  trajectory_publisher = nh.advertise<custom_msgs::Trajectory4>(
      trajectory_reference_topic_name, 1);
  trajectory_publish_timer = nh.createTimer(
      ros::Duration(1.0 / trajectory_publish_rate),
      std::bind(&PSBMPC_Node::publish_reference_trajectory, this));

  polygons_subscriber = nh.subscribe(polygons_topic_name, 1,
                                     &PSBMPC_Node::polygons_callback, this);

  do_data_subscriber =
      nh.subscribe(do_data_topic_name, 1, &PSBMPC_Node::do_data_callback, this);

  ownship_pose_subscriber.subscribe(nh, ownship_pose_topic_name, 1);
  ownship_velocity_subscriber.subscribe(nh, ownship_velocity_topic_name, 1);
  synchronizer.registerCallback(
      boost::bind(&PSBMPC_Node::ownship_state_callback, this, _1, _2));

  waypoint_subscriber = nh.subscribe(waypoint_topic_name, 1,
                                     &PSBMPC_Node::waypoint_callback, this);

  wind_subscriber =
      nh.subscribe(wind_topic_name, 1, &PSBMPC_Node::wind_callback, this);

  supervisor_mode_subscriber =
      nh.subscribe(supervisor_mode_topic_name, 1,
                   &PSBMPC_Node::supervisor_mode_callback, this);

  set_psbmpc_mode_service =
      nh.advertiseService(set_psbmpc_mode_service_name,
                          &PSBMPC_Node::set_psbmpc_mode_handler, this);

  waypoint_loader_service = nh.advertiseService(
      waypoint_loader_service_name, &PSBMPC_Node::load_waypoints_handler, this);
}

/****************************************************************************************
 *  Name     : publish_psbmpc_mode
 *  Function : Runs the PSB-MPC algorithm on the current information and
 * publishes the optimal solution. Method   : Author   :
 *****************************************************************************************/
void PSBMPC_Node::publish_psbmpc_mode() {
  std::scoped_lock psbmpc_mode_lock(psbmpc_mode_data_mutex);
  std_msgs::String psbmpc_mode_msg;
  psbmpc_mode_msg.data = psbmpc_mode;
  psbmpc_mode_publisher.publish(psbmpc_mode_msg);
}

/****************************************************************************************
 *  Name     : publish_reference_trajectory
 *  Function : Runs the PSB-MPC algorithm on the current information and
 * publishes the optimal solution. Method   : Author   :
 *****************************************************************************************/
void PSBMPC_Node::publish_reference_trajectory() {
  std::scoped_lock psbmpc_data_lock(ownship_data_mutex, obstacles_data_mutex,
                                    polygons_data_mutex, wp_data_mutex,
                                    wind_data_mutex, supervisor_mode_data_mutex,
                                    psbmpc_mode_data_mutex);

  bool initialized = ownship_state.size() >= 4 && waypoints.cols() > 1;
  if (!initialized) {
    return;
  }
  auto start = std::chrono::system_clock::now();

  /* std::cout << "Ownship state = " << ownship_state.transpose() << std::endl;

  std::cout << "U_d = " << U_d[ownship.get_wp_counter()] << " | chi_d = " <<
  chi_d << std::endl; for (size_t i = 0; i < obstacles.size(); i++)
  {
          std::cout << "DO " << i << ": state = " <<
  obstacles[i].kf.get_state().transpose() << std::endl; std::cout << "DO " << i
  << ": Pr_WGW = " << obstacles[i].get_Pr_WGW() << "Pr_CCEM = " <<
  obstacles[i].get_Pr_CCEM() << std::endl; std::cout << "DO " << i << ": Pr_s_i
  = " << obstacles[i].get_scenario_probabilities().transpose() << std::endl;
  } */

  double dt_mpc(0.0);
  if (psbmpc_mode == "sbmpc") {
    sbmpc.calculate_optimal_offsets(u_opt, chi_opt, predicted_trajectory,
                                    U_d[ownship.get_wp_counter()], chi_d,
                                    waypoints, ownship_state, V_w,
                                    wind_direction, polygons, obstacles, false);

    dt_mpc = sbmpc.pars.get_dpar(i_dpar_dt_SBMPC);
  } else if (psbmpc_mode == "psbmpc") {
    psbmpc.calculate_optimal_offsets(
        u_opt, chi_opt, predicted_trajectory, U_d[ownship.get_wp_counter()],
        chi_d, waypoints, ownship_state, V_w, wind_direction, polygons,
        obstacles, false);

    dt_mpc = psbmpc.pars.get_dpar(i_dpar_dt);
  } else if (psbmpc_mode == "wp-tracking") {
    psbmpc.calculate_optimal_offsets(u_opt, chi_opt, predicted_trajectory,
                                     U_d[ownship.get_wp_counter()], chi_d,
                                     waypoints, ownship_state, V_w,
                                     wind_direction, polygons, obstacles, true);

    dt_mpc = psbmpc.pars.get_dpar(i_dpar_dt);
  }

  int n_samples = predicted_trajectory.cols();
  custom_msgs::Trajectory4 trajectory_msg;

  trajectory_msg.dt_p = dt_mpc * trajectory_downsampling_factor;
  for (int k = 0; k < n_samples; k += trajectory_downsampling_factor) {
    trajectory_msg.x_p.push_back(predicted_trajectory(0, k));
    trajectory_msg.y_p.push_back(predicted_trajectory(1, k));
    trajectory_msg.chi_p.push_back(
        PSBMPC_LIB::CPU::wrap_angle_to_pmpi(predicted_trajectory(2, k)));
#if (OWNSHIP_TYPE == 0)
    trajectory_msg.U_p.push_back(predicted_trajectory(3, k));
#else
    trajectory_msg.U_p.push_back(sqrt(pow(predicted_trajectory(3, k), 2) +
                                      pow(predicted_trajectory(4, k), 2)));
#endif
  }
  trajectory_publisher.publish(trajectory_msg);

  auto end = std::chrono::system_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  if (psbmpc_mode == "sbmpc") {
    ROS_INFO("SB-MPC runtime: %.4f secs", elapsed.count() / 1000.0);
  } else if (psbmpc_mode == "psbmpc") {
    ROS_INFO("PSB-MPC runtime: %.4f secs", elapsed.count() / 1000.0);
  }
}

/****************************************************************************************
 *  Name     : polygons_callback
 *  Function : Collects PSBMPC grounding hazard data (relevant inside a range of
 *			  d_so_relevant).
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Node::polygons_callback(
    const custom_msgs::Polygons::ConstPtr
        &msg // In: Message containing PSBMPC polygons (grounding hazards)
) {
  std::scoped_lock polygons_data_lock(polygons_data_mutex);

  polygons.clear();
  int n_polygons = msg->polygons.size(), n_vertices(0);
  polygon_2D polygon, polygon_copy;
  typename boost::geometry::point_type<polygon_2D>::type v;
  for (int j = 0; j < n_polygons; j++) {
    n_vertices = msg->polygons[j].points.size();
    for (int i = 0; i < n_vertices; i++) {
      boost::geometry::assign_values(v, msg->polygons[j].points[i].x,
                                     msg->polygons[j].points[i].y);
      boost::geometry::append(polygon_copy, v);
    }
    polygons.push_back(polygon_copy);
    polygon_copy = polygon;
  }
}

/****************************************************************************************
 *  Name     : do_data_callback
 *  Function : Collects dynamic obstacle data: ID, dims, state, predicted
 * trajectories. For all obstacles. Includes intent info if possible. Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Node::do_data_callback(
    const ros_af_msgs::DynamicObstaclesData::ConstPtr
        &msg // In: ROS message containing obstacle data
) {
  std::scoped_lock obstacles_lock(obstacles_data_mutex);

  obstacles.clear();

  Eigen::Vector4d dims, xs_i_ps_k;
  Eigen::Matrix4d P;
  Eigen::Matrix2d pv_corr;
  Eigen::MatrixXd P_i_p;
  Eigen::VectorXd Pr_s_i;
  std::vector<Eigen::MatrixXd> xs_i_p;
  PSBMPC_LIB::Tracked_Obstacle to;
  double duration_tracked(0.0), duration_lost(0.0), Pr_WGW(0.0), Pr_CCEM(0.0);
  int n_do = msg->do_data.size(), ID(0), n_ps_i(0), n_samples(0);
  for (int i = 0; i < n_do; i++) {
    ID = msg->do_data[i].ID;

    dims << msg->do_data[i].dimensions.x, msg->do_data[i].dimensions.y,
        msg->do_data[i].dimensions.z, msg->do_data[i].dimensions.w;

    duration_tracked = msg->do_data[i].duration_tracked;
    duration_lost = msg->do_data[i].duration_lost;

    Pr_WGW = msg->do_data[i].Pr_WGW;
    Pr_CCEM = msg->do_data[i].Pr_CCEM;
    n_ps_i = msg->do_data[i].predicted_trajectories.size();
    Pr_s_i.resize(n_ps_i);
    Pr_s_i.setZero();
    xs_i_p.resize(n_ps_i);
    for (int ps = 0; ps < n_ps_i; ps++) {
      n_samples = msg->do_data[i].predicted_trajectories[ps].trajectory.size();
      Pr_s_i(ps) = msg->do_data[i].predicted_trajectories[ps].probability;
      xs_i_p[ps].resize(4, n_samples);
      if (ps == 0) {
        P_i_p.resize(16, n_samples);
      }
      for (int k = 0; k < n_samples; k++) {
        xs_i_ps_k(0) =
            msg->do_data[i].predicted_trajectories[ps].trajectory[k].pos_est.x;
        xs_i_ps_k(1) =
            msg->do_data[i].predicted_trajectories[ps].trajectory[k].pos_est.y;
        xs_i_ps_k(2) =
            msg->do_data[i].predicted_trajectories[ps].trajectory[k].vel_est.x;
        xs_i_ps_k(3) =
            msg->do_data[i].predicted_trajectories[ps].trajectory[k].vel_est.y;
        xs_i_p[ps].col(k) = xs_i_ps_k;

        if (ps == 0) {
          P(0, 0) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .pos_cov.var_x;
          P(0, 1) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .pos_cov.corr_xy;
          P(1, 0) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .pos_cov.corr_xy;
          P(1, 1) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .pos_cov.var_y;

          P(2, 2) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .vel_cov.var_x;
          P(0, 1) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .vel_cov.corr_xy;
          P(1, 0) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .vel_cov.corr_xy;
          P(3, 3) = msg->do_data[i]
                        .predicted_trajectories[ps]
                        .trajectory[k]
                        .vel_cov.var_y;

          pv_corr(0, 0) = msg->do_data[i]
                              .predicted_trajectories[ps]
                              .trajectory[k]
                              .pos_vel_corr.corr_px_vx;
          pv_corr(0, 1) = msg->do_data[i]
                              .predicted_trajectories[ps]
                              .trajectory[k]
                              .pos_vel_corr.corr_px_vy;
          pv_corr(1, 0) = msg->do_data[i]
                              .predicted_trajectories[ps]
                              .trajectory[k]
                              .pos_vel_corr.corr_py_vx;
          pv_corr(1, 1) = msg->do_data[i]
                              .predicted_trajectories[ps]
                              .trajectory[k]
                              .pos_vel_corr.corr_py_vy;

          P.block<2, 2>(0, 2) = pv_corr;
          P.block<2, 2>(2, 0) = pv_corr;

          P_i_p.col(k) = PSBMPC_LIB::CPU::flatten(P);
        }
      }
    }
    if (Pr_s_i.isZero()) {
      Pr_s_i((int)std::floor(Pr_s_i.size() / 2)) = 1.0;
    }
    Pr_s_i = Pr_s_i / Pr_s_i.sum();

    if (Pr_WGW > 0.999) {
      Pr_WGW = 0.999;
    } else if (Pr_WGW < 0.001) {
      Pr_WGW = 0.001;
    }

    if (Pr_CCEM > 0.999) {
      Pr_CCEM = 0.999;
    } else if (Pr_CCEM < 0.001) {
      Pr_CCEM = 0.001;
    }
    // std::cout << "PN i = " << i << ": Pr_WGW = " << Pr_WGW << " | Pr_CCEM = "
    // << Pr_CCEM << std::endl;
    to =
        PSBMPC_LIB::Tracked_Obstacle(xs_i_p, P_i_p, Pr_s_i, Pr_WGW, Pr_CCEM,
                                     dims, duration_tracked, duration_lost, ID);

    obstacles.push_back(to);
  }
}

/****************************************************************************************
 *  Name     : ownship_state_callback
 *  Function : Updates the ownship state
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Node::ownship_state_callback(
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
  // ROS_INFO("OWNSHIP STATE = %.2f, %.2f, %.2f, %.2f\n", ownship_state(0),
  // ownship_state(1), ownship_state(2), ownship_state(3));
#else
  ownship_state.resize(6);
  ownship_state(0) = pose_msg->pose.position.x;
  ownship_state(1) = pose_msg->pose.position.y;
  ownship_state(2) = heading;
  ownship_state(3) = nu(0);
  ownship_state(4) = nu(1);
  ownship_state(5) = nu(2);
#endif

  if (waypoints.cols() > 1) {
    ownship.determine_active_waypoint_segment(waypoints, ownship_state);

    ownship.update_guidance_references(U_d[ownship.get_wp_counter()], chi_d,
                                       waypoints, ownship_state, 0.0,
                                       PSBMPC_LIB::LOS);
  }
}

/****************************************************************************************
 *  Name     : waypoint_callback
 *  Function : Collects waypoint information for the ownship
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Node::waypoint_callback(
    const custom_msgs::NorthEastHeading::ConstPtr
        &msg // In: ROS message containing waypoint information
) {
  std::scoped_lock wp_data_lock(wp_data_mutex);

  int n_wps = waypoints.cols();
  waypoints.conservativeResize(2, n_wps + 1);
  waypoints(0, n_wps) = msg->north;
  waypoints(1, n_wps) = msg->east;
  U_d.push_back(1.0); // 1.5 = default ref speed
  std::cout << "WAYPOINTS = \n" << waypoints << std::endl;
}

/****************************************************************************************
 *  Name     : wind_callback
 *  Function : Collects wind estimate information for the ownship
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Node::wind_callback(
    const geometry_msgs::TwistStamped::ConstPtr
        &msg // In: ROS message containing wind information
) {
  std::scoped_lock wind_data_lock(wind_data_mutex);

  wind_direction(0) = msg->twist.linear.x;
  wind_direction(1) = msg->twist.linear.y;
  V_w = wind_direction.norm();
  wind_direction.normalize();
}

/****************************************************************************************
 *  Name     : load_waypoints_handler
 *  Function : Collects waypoint information for the ownship from a yaml file
 *  Method   :
 *  Author   :
 *****************************************************************************************/
bool PSBMPC_Node::load_waypoints_handler(
    psbmpc::LoadWaypoints::Request
        &req, // In: Request information for loading waypoints
    psbmpc::LoadWaypoints::Response
        &res // Out: Response information after loading waypoints
) {
  try {
    std::scoped_lock wp_data_lock(wp_data_mutex, ownship_data_mutex);

    std::string absolute_filename = ros::package::getPath("psbmpc") +
                                    "/waypoint_data/" + req.waypoints_yaml_file;
    YAML::Node yaml_node = YAML::LoadFile(absolute_filename);
    if (!yaml_node["psbmpc_waypoints"]) {
      res.status_msg = "Loading waypoints failed.";
      res.success = false;
      throw std::logic_error("Erroneous waypoint data format or empty input");
    }

    int n_wps = yaml_node["psbmpc_waypoints"].size();
    bool add_ownship_state(false);
    if (n_wps == 1) {
      // Add current own-ship state as first waypoint
      n_wps += 1;
      add_ownship_state = true;
      res.status_msg = "Added ownship state as initial WP. ";
    }

    waypoints.resize(2, n_wps);
    U_d.resize(n_wps);
    if (add_ownship_state) {
      waypoints(0, 0) = ownship_state(0);
      waypoints(1, 0) = ownship_state(1);
      U_d[0] = yaml_node["psbmpc_waypoints"][0]["U_d"].as<double>();
      waypoints(0, 1) = yaml_node["psbmpc_waypoints"][0]["N"].as<double>();
      waypoints(1, 1) = yaml_node["psbmpc_waypoints"][0]["E"].as<double>();
      U_d[1] = U_d[0];
    } else {
      for (int i = 0; i < n_wps; i++) {
        waypoints(0, i) = yaml_node["psbmpc_waypoints"][i]["N"].as<double>();
        waypoints(1, i) = yaml_node["psbmpc_waypoints"][i]["E"].as<double>();
        U_d[i] = yaml_node["psbmpc_waypoints"][i]["U_d"].as<double>();
      }
    }

    res.status_msg += "Loading waypoints succeeded.";
    res.success = true;
    return res.success;
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return res.success;
  }
}

/****************************************************************************************
 *  Name     : main
 *  Function : Sets up, initializes, runs the PSBMPC ROS node, and shuts it
 * down. Method   : Author   :
 *****************************************************************************************/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "psbmpc_node");
  ros::start();

  ROS_INFO("Start PSBMPC node initialization...");
  ros::NodeHandle nh("~");
  PSBMPC_Node psbmpc_node(nh);
  ROS_INFO("PSBMPC node initialization complete.");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
