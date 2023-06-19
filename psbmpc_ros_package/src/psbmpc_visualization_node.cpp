/****************************************************************************************
 *
 *  File name : psbmpc_visualization_node.cpp
 *
 *  Function  : Class functions for the PSBMPC visualization node, which publishes
 *              information relevant for the PSBMPC
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

#include "psbmpc_visualization_node.hpp"
#include "cpu/utilities_cpu.hpp"
#include "Eigen/Dense"
#include <ros/package.h>
#include <ros/console.h>
#include <ctime>
#include <exception>
#include <limits>
#include <iostream>

/****************************************************************************************
 *  Name     : PSBMPC_Visualization_Node
 *  Function : Class constructor, initializes parameters, variables and objects
 *  Method   :
 *  Author   :
 *****************************************************************************************/
PSBMPC_Visualization_Node::PSBMPC_Visualization_Node(
    ros::NodeHandle &nh // In: Handle to the visualization node, initialized in the main loop
    ) : synchronizer(ownship_pose_subscriber, ownship_velocity_subscriber, 1), nh(nh)
{
    trajectory_reference_vis_topic_name = nh.param<std::string>("trajectory_reference_vis_topic_name", "");
    trajectory_vis_topic_name = nh.param<std::string>("trajectory_vis_topic_name", "");
    pose_vis_topic_name = nh.param<std::string>("pose_vis_topic_name", "");
    trajectory_reference_topic_name = nh.param<std::string>("trajectory_reference_topic_name", "");
    polygons_vis_topic_name = nh.param<std::string>("polygons_vis_topic_name", "");
    polygons_topic_name = nh.param<std::string>("polygons_topic_name", "");
    waypoint_topic_name = nh.param<std::string>("waypoint_topic_name", "");

    ownship_pose_topic_name = nh.param<std::string>("ownship_pose_topic_name", "");
    ownship_velocity_topic_name = nh.param<std::string>("ownship_velocity_topic_name", "");

    do_data_topic_name = nh.param<std::string>("do_data_topic_name", "");
    do_trajectories_vis_topic_name = nh.param<std::string>("do_trajectories_vis_topic_name", "");

    get_polygons_service_name = nh.param<std::string>("get_polygons_service_name", "");

    trajectory_reference_vis_publish_rate = nh.param<double>("trajectory_reference_vis_publish_rate", 0.5);
    polygons_vis_publish_rate = nh.param<double>("polygons_vis_publish_rate", 0.1);

    d_so_relevant_viz = nh.param<double>("d_so_relevant_viz", 200.0);

    trajectory_reference_scale_x = nh.param<double>("trajectory_reference/scale_x", 0.2);
    trajectory_reference_color = nh.param<std::vector<double>>("trajectory_reference/color", {});
    trajectory_update_threshold = nh.param<double>("trajectory/update_threshold", 0.2);
    trajectory_scale_x = nh.param<double>("trajectory/scale_x", 0.2);
    trajectory_color = nh.param<std::vector<double>>("trajectory/color", {});
    safety_zone_scale_x = nh.param<double>("safety_zone/scale_x", {});
    safety_zone_color = nh.param<std::vector<double>>("safety_zone/color", {});
    pose_color = nh.param<std::vector<double>>("pose/color", {});
    pose_scale = nh.param<std::vector<double>>("pose/scale", {});
    wps_color = nh.param<std::vector<double>>("wps/color", {});
    wps_scale = nh.param<std::vector<double>>("wps/scale", {});
    polygon_scale_x = nh.param<double>("polygon/scale_x", 0.2);
    polygon_color = nh.param<std::vector<double>>("polygon/color", {});

    std::string u_offsets, chi_offsets;
    std::vector<double> dpars;
    std::vector<int> ipars;
    int cpe_method(0), prediction_method(0), guidance_method(0);
    u_offsets = nh.param<std::string>("/psbmpc/psbmpc_parameters/u_offsets", u_offsets);
    chi_offsets = nh.param<std::string>("/psbmpc/psbmpc_parameters/chi_offsets", chi_offsets);
    cpe_method = nh.param<int>("/psbmpc/psbmpc_parameters/cpe_method", 100);
    prediction_method = nh.param<int>("/psbmpc/psbmpc_parameters/prediction_method", 100);
    guidance_method = nh.param<int>("/psbmpc/psbmpc_parameters/guidance_method", 100);
    ipars = nh.param<std::vector<int>>("/psbmpc/psbmpc_parameters/ipars", ipars);
    dpars = nh.param<std::vector<double>>("/psbmpc/psbmpc_parameters/dpars", dpars);

    psbmpc_pars = PSBMPC_LIB::PSBMPC_Parameters(
        PSBMPC_LIB::CPU::parse_VVD(u_offsets),
        PSBMPC_LIB::CPU::parse_VVD(chi_offsets),
        static_cast<PSBMPC_LIB::CPE_Method>(cpe_method),
        static_cast<PSBMPC_LIB::Prediction_Method>(prediction_method),
        static_cast<PSBMPC_LIB::Guidance_Method>(guidance_method),
        ipars,
        dpars);

    trajectory.resize(4, 0);
    predicted_trajectory.resize(4, 0);
    waypoints.resize(2, 0);
    polygons.resize(0);
    //=========================================================================
    // Initialize clients, services, publishers and subscribers
    //=========================================================================
    trajectory_reference_vis_publisher = nh.advertise<visualization_msgs::MarkerArray>(trajectory_reference_vis_topic_name, 10);
    trajectory_reference_vis_publish_timer = nh.createTimer(
        ros::Duration(1.0 / trajectory_reference_vis_publish_rate),
        std::bind(&PSBMPC_Visualization_Node::publish_trajectory_reference_visualization, this));

    trajectory_vis_publisher = nh.advertise<visualization_msgs::Marker>(trajectory_vis_topic_name, 10);

    pose_vis_publisher = nh.advertise<visualization_msgs::Marker>(pose_vis_topic_name, 10);

    polygons_vis_publisher = nh.advertise<visualization_msgs::MarkerArray>(polygons_vis_topic_name, 10);
    polygons_vis_publish_timer = nh.createTimer(
        ros::Duration(1.0 / polygons_vis_publish_rate),
        std::bind(&PSBMPC_Visualization_Node::publish_polygons_visualization, this));

    do_trajectories_vis_publisher = nh.advertise<visualization_msgs::MarkerArray>(do_trajectories_vis_topic_name, 10);
    do_trajectories_subscriber = nh.subscribe(do_data_topic_name, 1, &PSBMPC_Visualization_Node::publish_do_trajectories, this);

    ownship_pose_subscriber.subscribe(nh, ownship_pose_topic_name, 1);
    ownship_velocity_subscriber.subscribe(nh, ownship_velocity_topic_name, 1);
    synchronizer.registerCallback(boost::bind(&PSBMPC_Visualization_Node::ownship_state_callback, this, _1, _2));

    trajectory_reference_subscriber = nh.subscribe(
        trajectory_reference_topic_name,
        1,
        &PSBMPC_Visualization_Node::trajectory_reference_callback,
        this);

    waypoint_subscriber = nh.subscribe(waypoint_topic_name, 10, &PSBMPC_Visualization_Node::waypoint_callback, this);

    get_polygons_client = nh.serviceClient<psbmpc::GetPolygons>(get_polygons_service_name);

    first_trajectory_update = true;
}

/****************************************************************************************
 *  Name     : publish_trajectory_visualization
 *  Function : Publishes a visualization marker for  the current predicted PSBMPC
 *             trajectory
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::publish_trajectory_reference_visualization()
{
    std::scoped_lock trajectory_reference_data_lock(trajectory_reference_data_mutex);

    int n_samples = predicted_trajectory.cols();
    if (n_samples == 0)
    {
        return;
    }

    visualization_msgs::MarkerArray tviz_msg;

    // Create marker for the trajectory reference and the safety_zone at the current time
    visualization_msgs::Marker trajectory_reference, safety_zone, wps;
    trajectory_reference.header.frame_id = safety_zone.header.frame_id = wps.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
    trajectory_reference.header.stamp = safety_zone.header.stamp = wps.header.stamp = ros::Time::now();
    trajectory_reference.ns = safety_zone.ns = wps.ns = "psbmpc_trajectory_visualization";
    trajectory_reference.action = safety_zone.action = wps.action = visualization_msgs::Marker::ADD;
    trajectory_reference.pose.orientation.w = safety_zone.pose.orientation.w = wps.pose.orientation.w = 1.0;
    trajectory_reference.type = visualization_msgs::Marker::LINE_STRIP;
    safety_zone.type = visualization_msgs::Marker::LINE_STRIP;
    wps.type = visualization_msgs::Marker::POINTS;
    wps.points.resize(1);
    wps.points[0].x = wps.points[0].y = wps.points[0].z = 1e10;

    trajectory_reference.lifetime = ros::Duration(10.0);
    trajectory_reference.scale.x = trajectory_reference_scale_x;
    wps.scale.x = wps_scale[0];
    wps.scale.y = wps_scale[1];
    wps.scale.z = wps_scale[2];

    // Trajectory lines are blue, course indicating arrows red
    trajectory_reference.color.r = trajectory_reference_color[0];
    trajectory_reference.color.g = trajectory_reference_color[1];
    trajectory_reference.color.b = trajectory_reference_color[2];
    trajectory_reference.color.a = trajectory_reference_color[3];
    safety_zone.color.r = safety_zone_color[0];
    safety_zone.color.g = safety_zone_color[1];
    safety_zone.color.b = safety_zone_color[2];
    safety_zone.color.a = safety_zone_color[3];
    wps.color.r = wps_color[0];
    wps.color.g = wps_color[1];
    wps.color.b = wps_color[2];
    wps.color.a = wps_color[3];

    trajectory_reference.id = 0;
    geometry_msgs::Point point;
    for (int k = 0; k < n_samples; k++)
    {
        point.x = predicted_trajectory(0, k);
        point.y = predicted_trajectory(1, k);
        point.z = 0.0;

        trajectory_reference.points.push_back(point);
    }
    tviz_msg.markers.push_back(trajectory_reference);

    double d_safe(psbmpc_pars.get_dpar(i_dpar_d_safe));
    safety_zone.scale.x = 0.9;
    safety_zone.id = 1;
    double incr(0.05), theta(0.0);
    for (int ii = 0; ii < 150; ii++)
    {
        theta = incr * ii;
        point.x = d_safe * cos(theta) + predicted_trajectory(0, 0);
        point.y = d_safe * sin(theta) + predicted_trajectory(1, 0);
        point.z = 0.0;

        safety_zone.points.push_back(point);
    }
    tviz_msg.markers.push_back(safety_zone);

    wps.id = 2;
    int n_wps = waypoints.cols();
    if (n_wps > 0)
    {
        for (int w = 0; w < n_wps; w++)
        {
            point.x = waypoints(0, w);
            point.y = waypoints(1, w);
            point.z = 0.0;
            wps.points.push_back(point);
        }
        tviz_msg.markers.push_back(wps);
    }

    trajectory_reference_vis_publisher.publish(tviz_msg);
}

/****************************************************************************************
 *  Name     : publish_polygons_visualization
 *  Function :
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::publish_polygons_visualization()
{
    std::scoped_lock polygons_data_lock(polygons_data_mutex);

    // Fetch non-simplified PSBMPC grounding hazard data
    psbmpc::GetPolygons srv;
    srv.request.d_so_relevant = d_so_relevant_viz;
    int n_polygons(0), n_vertices(0);

    get_polygons_client.waitForExistence();
    if (get_polygons_client.call(srv))
    {
        n_polygons = srv.response.polygons.polygons.size();
        polygons.resize(n_polygons);
        Eigen::MatrixXd vertices;
        for (int j = 0; j < n_polygons; j++)
        {
            n_vertices = srv.response.polygons.polygons[j].points.size();
            vertices.resize(2, n_vertices);
            for (int v = 0; v < n_vertices; v++)
            {
                vertices(0, v) = srv.response.polygons.polygons[j].points[v].x;
                vertices(1, v) = srv.response.polygons.polygons[j].points[v].y;
            }
            polygons[j] = vertices;
        }
    }
    else
    {
        ROS_WARN("Failed to call service get_polygons");
        return;
    }

    visualization_msgs::MarkerArray polygons_msg;
    visualization_msgs::Marker polygon;
    polygon.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
    polygon.header.stamp = ros::Time::now();
    polygon.ns = "psbmpc_polygons_visualization";
    polygon.action = visualization_msgs::Marker::ADD;
    polygon.pose.orientation.w = 1.0;
    polygon.type = visualization_msgs::Marker::LINE_STRIP;

    polygon.scale.x = polygon_scale_x;

    polygon.color.r = polygon_color[0];
    polygon.color.g = polygon_color[1];
    polygon.color.b = polygon_color[2];
    polygon.color.a = polygon_color[3];

    int count(0);
    visualization_msgs::Marker polygon_copy;
    geometry_msgs::Point point;
    for (int j = 0; j < n_polygons; j++)
    {
        polygon_copy = polygon;
        polygon_copy.id = count;
        n_vertices = polygons[j].cols();
        for (int v = 0; v < n_vertices; v++)
        {
            point.x = polygons[j](0, v);
            point.y = polygons[j](1, v);
            point.z = 0.0;
            polygon_copy.points.push_back(point);
        }
        count++;
        polygons_msg.markers.push_back(polygon_copy);
    }

    polygons_vis_publisher.publish(polygons_msg);
}

/****************************************************************************************
 *  Name     : publish_trajectory_visualization
 *  Function : Publishes a visualization marker for  the current predicted PSBMPC
 *             trajectory
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::publish_do_trajectories(
    const ros_af_msgs::DynamicObstaclesData::ConstPtr &msg // In: Message containing dynamic obstacle data
)
{
    std::scoped_lock do_trajectory_data_lock(do_trajectory_data_mutex);

    visualization_msgs::MarkerArray trajectory_candidates_vis;
    visualization_msgs::Marker trajectory_vis;
    trajectory_vis.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
    trajectory_vis.header.stamp = ros::Time::now();
    trajectory_vis.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_vis.action = 0;
    trajectory_vis.pose.orientation.w = 1;
    trajectory_vis.color.a = 1.0;

    Eigen::VectorXd probs;
    geometry_msgs::Point point;
    for (const auto &ship_msg : msg->do_data)
    {
        probs.resize(ship_msg.predicted_trajectories.size());
        trajectory_vis.ns = "Trajectories of DO:" + std::to_string(ship_msg.ID);
        for (const auto &trajectory_msg : ship_msg.predicted_trajectories)
        {
            probs(trajectory_msg.ID) = trajectory_msg.probability;
            trajectory_vis.id = trajectory_msg.ID;
            trajectory_vis.lifetime = ros::Duration(10.0);
            trajectory_vis.points.clear();
            trajectory_vis.color.r = trajectory_msg.probability;
            trajectory_vis.color.g = 0.0;
            trajectory_vis.color.b = 1.0 - trajectory_msg.probability;
            trajectory_vis.scale.x = 0.5;

            for (const auto &state_msg : trajectory_msg.trajectory)
            {
                point.x = state_msg.pos_est.x;
                point.y = state_msg.pos_est.y;
                point.z = 0;
                trajectory_vis.points.push_back(point);
            }
            trajectory_candidates_vis.markers.push_back(trajectory_vis);
        }
        // std::cout << "DO " << ship_msg.ID << " | trajectory probs = " << probs.transpose() << std::endl;
    }

    do_trajectories_vis_publisher.publish(trajectory_candidates_vis);
}

/****************************************************************************************
 *  Name     : trajectory_reference_callback
 *  Function : Collects PSBMPC trajectory data
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::trajectory_reference_callback(
    const custom_msgs::Trajectory4::ConstPtr &msg // In: Message containing PSBMPC predicted trajectory
)
{
    std::scoped_lock trajectory_reference_data_lock(trajectory_reference_data_mutex);
    predicted_trajectory.resize(4, msg->x_p.size());
    for (size_t k = 0; k < msg->x_p.size(); k++)
    {
        predicted_trajectory(0, k) = msg->x_p[k];
        predicted_trajectory(1, k) = msg->y_p[k];
        predicted_trajectory(2, k) = msg->chi_p[k];
        predicted_trajectory(3, k) = msg->U_p[k];
    }
}

/****************************************************************************************
 *  Name     : ownship_state_callback
 *  Function : Updates the ownship state
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::ownship_state_callback(
    const geometry_msgs::PoseStamped::ConstPtr &pose_msg,  // In: ROS message containing ownship pose
    const geometry_msgs::TwistStamped::ConstPtr &twist_msg // In: ROS message containing ownship twist in body
)
{
    std::scoped_lock ownship_data_lock(ownship_data_mutex);

    Eigen::VectorXd ownship_state;
    Eigen::Quaterniond q(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    Eigen::VectorXd euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    double heading = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(euler_angles(2));

    Eigen::Vector3d nu;
    nu(0) = twist_msg->twist.linear.x;
    nu(1) = twist_msg->twist.linear.y;
    nu(2) = twist_msg->twist.angular.z;

#if OWNSHIP_TYPE == 0
    ownship_state.resize(4);
    ownship_state(0) = pose_msg->pose.position.x;
    ownship_state(1) = pose_msg->pose.position.y;
    double crab_angle(0.0);
    if (fabs(nu(0)) < 1e-02 && nu(1) < 1e-02)
    {
        crab_angle = 0.0;
    }
    else
    {
        crab_angle = 0.0; // atan2(nu(1), nu(0));
    }
    ownship_state(2) = PSBMPC_LIB::CPU::wrap_angle_to_pmpi(heading + crab_angle);
    double speed_over_ground = sqrt(pow(nu(0), 2) + pow(nu(1), 2));
    ownship_state(3) = speed_over_ground;
    // ROS_INFO("OWNSHIP STATE = %.2f, %.2f, %.2f, %.2f\n", ownship_state(0), ownship_state(1), ownship_state(2), ownship_state(3));
#else
    ownship_state.resize(6);
    ownship_state(0) = pose_msg->pose.position.x;
    ownship_state(1) = pose_msg->pose.position.y;
    ownship_state(2) = heading;
    ownship_state(3) = nu(0);
    ownship_state(4) = nu(1);
    ownship_state(5) = nu(2);
#endif

    visualization_msgs::Marker pose_vis;
    pose_vis.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
    pose_vis.header.stamp = ros::Time::now();
    pose_vis.ns = "psbmpc_polygons_visualization";
    pose_vis.action = visualization_msgs::Marker::ADD;
    pose_vis.pose.orientation.w = 1.0;
    pose_vis.type = visualization_msgs::Marker::ARROW;

    pose_vis.scale.x = pose_scale[0];
    pose_vis.scale.y = pose_scale[1];
    pose_vis.scale.z = pose_scale[2];

    pose_vis.color.r = pose_color[0];
    pose_vis.color.g = pose_color[1];
    pose_vis.color.b = pose_color[2];
    pose_vis.color.a = pose_color[3];
    pose_vis.pose = pose_msg->pose;
    pose_vis.lifetime.sec = 5;

    pose_vis_publisher.publish(pose_vis);

    trajectory.conservativeResize(ownship_state.size(), trajectory.cols() + 1);
    trajectory.col(trajectory.cols() - 1) = ownship_state;

    t_now_trajectory_update = (double)ros::Time::now().sec + (double)ros::Time::now().nsec * 1e-9;
    if (first_trajectory_update)
    {
        t_prev_trajectory_update = t_now_trajectory_update;
        first_trajectory_update = false;
        return;
    }

    double dt_trajectory_update = t_now_trajectory_update - t_prev_trajectory_update;
    if (dt_trajectory_update < trajectory_update_threshold)
    {
        return;
    }

    visualization_msgs::MarkerArray trajectory_vis_msg;
    visualization_msgs::Marker trajectory_vis;
    trajectory_vis.header.frame_id = nh.param<std::string>("/frames/local_NED", "");
    trajectory_vis.header.stamp = ros::Time::now();
    trajectory_vis.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_vis.action = visualization_msgs::Marker::ADD;
    trajectory_vis.pose.orientation.w = 1.0;
    trajectory_vis.scale.x = trajectory_scale_x;

    trajectory_vis.color.r = trajectory_color[0];
    trajectory_vis.color.g = trajectory_color[1];
    trajectory_vis.color.b = trajectory_color[2];
    trajectory_vis.color.a = trajectory_color[3];

    int n_samples = trajectory.cols();
    geometry_msgs::Point point;
    for (int k = 0; k < n_samples; k++)
    {
        point.x = trajectory(0, k);
        point.y = trajectory(1, k);
        point.z = 0.0;
        trajectory_vis.points.push_back(point);
    }
    trajectory_vis_publisher.publish(trajectory_vis);

    t_prev_trajectory_update = t_now_trajectory_update;
}

/****************************************************************************************
 *  Name     : polygons_callback
 *  Function : Collects local simplified PSBMPC grounding hazard data
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::relevant_psbmpc_polygons_callback(
    const custom_msgs::Polygons::ConstPtr &msg // In: Message containing PSBMPC polygons (grounding hazards)
)
{
    std::scoped_lock simplified_polygons_data_lock(simplified_polygons_data_mutex);

    simplified_polygons.clear();
    int n_polygons = msg->polygons.size(), n_vertices(0);
    Eigen::MatrixXd vertices;
    for (int j = 0; j < n_polygons; j++)
    {
        n_vertices = msg->polygons[j].points.size();
        vertices.resize(2, n_vertices);
        for (int v = 0; v < n_vertices; v++)
        {
            vertices(0, v) = msg->polygons[j].points[v].x;
            vertices(1, v) = msg->polygons[j].points[v].y;
        }
        simplified_polygons.push_back(vertices);
    }
}

/****************************************************************************************
 *  Name     : waypoint_callback
 *  Function : Collects waypoint information for the ownship
 *  Method   :
 *  Author   :
 *****************************************************************************************/
void PSBMPC_Visualization_Node::waypoint_callback(
    const custom_msgs::NorthEastHeading::ConstPtr &msg // In: ROS message containing waypoint information
)
{
    std::scoped_lock wp_data_lock(wp_data_mutex);

    int n_wps = waypoints.cols();
    waypoints.conservativeResize(2, n_wps + 1);
    waypoints(0, n_wps) = msg->north;
    waypoints(1, n_wps) = msg->east;
}

/****************************************************************************************
 *  Name     : main
 *  Function : Sets up, initializes, runs the visualization node, and shuts it down.
 *  Method   :
 *  Author   :
 *****************************************************************************************/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "psbmpc_visualization_node");
    ros::start();

    ROS_INFO("Start PSBMPC visualization node initialization...");
    ros::NodeHandle nh("~");
    PSBMPC_Visualization_Node psbmpc_visualization_node(nh);
    ROS_INFO("PSBMPC visualization node initialization complete.");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
