#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <random>

//#include "asv_ctrl_drvo/shipModel.h"
//#include "asv_ctrl_drvo/obstacle.h"
//#include "asv_ctrl_drvo/asv_ctrl_drvo.h"

#include "asv_ctrl_drvo.h"
#include "drvo_lib/obstacle.h"
#include "drvo_lib/drvoAgent.h"
#include "drvo_lib/ship_model.h"

using namespace std;

dynamicRvoRosLink::dynamicRvoRosLink(){

	drvo = new Agent(); // the link to the drvoAgent library fxns!

}


dynamicRvoRosLink::~dynamicRvoRosLink(){

	delete drvo;
	drvo = NULL;
}


void dynamicRvoRosLink::initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map)
{
	ROS_INFO("Initializing drvo node...");
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}

	/// @todo Remove local_map_! Only used for debugging purposes...
	local_map_.header.frame_id ="map";
	local_map_.info.resolution = 0.78;
	local_map_.info.width  = 1362;
	local_map_.info.height = 942;
	local_map_.info.origin.position.x = -(float)496;
	local_map_.info.origin.position.y = -(float)560;

	local_map_.data.resize(local_map_.info.width*local_map_.info.height);
	ros::NodeHandle n;
	lm_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);
	obstacles_ = obstacles;
	map_ = map;
	
	ROS_INFO("Initialization complete");
}

void dynamicRvoRosLink::updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d)
{
  	double yaw = tf::getYaw(msg->pose.pose.orientation);
  	asv_pose_[0] = msg->pose.pose.position.x;
  	asv_pose_[1] = msg->pose.pose.position.y;
  	asv_pose_[2] = yaw;
  	asv_twist_[0] = msg->twist.twist.linear.x;
  	asv_twist_[1] = msg->twist.twist.linear.y;
  	asv_twist_[2] = msg->twist.twist.angular.z;

  	u_d_ = u_d;
  	psi_d_ = psi_d;
}


//void dynamicRvoRosLink::getBestControlInput(double &u_best, double &psi_best, ofstream &log_asv, ofstream &log_obst_1, ofstream &log_obst_2, ofstream &log_obst_3)
void dynamicRvoRosLink::getBestControlOffset(double &u_os_best, double &psi_os_best)
{
	// variables for generating measurement (Guassian) noise
	
	// position
	const double mean_pos = 0.0, mean_obs_pos = 0.0;
	const double stddev_pos = 1*5.0, stddev_obs_pos = 1*15.0; // 0.1 / 5.0
	static std::default_random_engine generator_pos(std::random_device{}()), generator_obs_pos;
	std::normal_distribution<double> distri_pos(mean_pos, stddev_pos), distri_obs_pos(mean_obs_pos, stddev_obs_pos);
	
	// course
	const double mean_chi = 0.0, mean_obs_chi = 0.0;
	const double stddev_chi = 1*0.02, stddev_obs_chi = 1*0.53; // 0.02 / 0.27 / 0.53  (0.02rad -> 1deg, 0.53rad -> 30.37deg, 027rad -> 15.5deg)
	static std::mt19937 generator_chi(std::random_device{}()), generator_obs_chi;
	std::normal_distribution<double> distri_chi(mean_chi, stddev_chi), distri_obs_chi(mean_obs_chi, stddev_obs_chi);	
	
	// speed	
	const double mean_u = 0.0, mean_obs_u = 0.0;
	const double stddev_u = 1*1.0, stddev_obs_u = 1*1.0; // 1.0 / 1.0
	static std::minstd_rand generator_u(std::random_device{}()), generator_obs_u;
	std::normal_distribution<double> distri_u(mean_u, stddev_u), distri_obs_u(mean_obs_u, stddev_obs_u);
	
	// asv and obstacle matrices
	Eigen::Matrix<double, 6, 1> asv_state;
	Eigen::Matrix<double, -1, 10> obst_states; // replace 5 with -1 or Dynamic; used to be 9, now 10!
	Eigen::Matrix<double, -1, 10> obst_states_vary;
	static int loop_counter=1;
	
	Eigen::Matrix<double, -1, 2> next_waypoints; // 
	

	std::vector<asv_msgs::State>::iterator it; // Obstacles iterator
	int i, n_obst=obstacles_->size(), j, n_nwps=1; //next_waypoint_->size(); 	 
	
	double obst_x, obst_y, obst_yaw, obst_u, u_best, psi_best;  

	// prepare all parameters for the drvo fxn! 
	// u_d_, psi_d_ are updated by updateAsvState callback in main(), so they can be used directly!

	// get asv_state: x,y,psi,u,v,r (with measurement noise)
	//if (isnan(asv_pose_[0]) || isnan(asv_pose_[1]) || isnan(asv_pose_[2])) return; //bug!

	asv_state << asv_pose_[0] + distri_pos(generator_pos), asv_pose_[1] + distri_pos(generator_pos), asv_pose_[2] + distri_chi(generator_chi), asv_twist_[0] + distri_u(generator_u), asv_twist_[1], asv_twist_[2]; // MR interface output sign change
	
	// get obst_states
	if (n_obst > 0){ 
		obst_states.resize(n_obst, 10); //9 
		for (it = obstacles_->begin(); it != obstacles_->end(); ++it){
			i = std::distance(obstacles_->begin(), it); 
			
			// save certain values of first obstacle to check variation in signal w.r.t. simulated uncertainty.
			if (it->header.id == 0){
				obst_x = it->x; obst_y = it->y; obst_yaw = it->psi; obst_u = it->u;  
			}

			if (it->header.id == 0){
				obst_states.row(i) << it->x + distri_obs_pos(generator_obs_pos), it->y + distri_obs_pos(generator_obs_pos), normalize(it->psi + distri_obs_chi(generator_obs_chi)), it->u + distri_obs_u(generator_obs_u), it->v, it->header.radius, it->header.radius, it->header.radius, it->header.radius, it->header.id; // + 5 + abs(distri_obs_pos(generator_obs_pos)); // MR interface output sign change
			
			}else{
			
				obst_states.row(i) << it->x + distri_obs_pos(generator_obs_pos), it->y + distri_obs_pos(generator_obs_pos), normalize(it->psi + distri_obs_chi(generator_obs_chi)), it->u + distri_obs_u(generator_obs_u), it->v, it->header.radius, it->header.radius, it->header.radius, it->header.radius, it->header.id;// + 15 + abs(distri_obs_pos(generator_obs_pos)); // MR interface output sign change
			}

// obstacle state
//ROS_INFO("obs_x: %0.2f  obs_y: %0.2f  obs_psi: %0.2f  obs_u: %0.2f  obs_v: %0.2f  obs_radius: %0.2f", obst_states(i,0), obst_states(i,1), obst_states(i,2)*180.0f/M_PI, obst_states(i,3), obst_states(i,4), obst_states(i,5));
		}
	}
	
	// simulate obstacle leaving and re-entering list (i.e. out/in sensor range)
		
	if (false && n_obst > 0 ){
	
		if(loop_counter > 12 && loop_counter < 25){
		
			obst_states_vary.resize(n_obst-1, 10);
		
			for (int i=0; i< n_obst-1; i++)
				obst_states_vary.row(i) = obst_states.row(i); 	
			
		}else{
			obst_states_vary.resize(n_obst, 10);
		
			for (int i=0; i< n_obst; i++)
				obst_states_vary.row(i) = obst_states.row(i); 
						
		}	
			
	}else{
		obst_states_vary.resize(n_obst, 10);
		
		for (int i=0; i< n_obst; i++)
			obst_states_vary.row(i) = obst_states.row(i); 	
	}
		
	
	/*	
	if (n_obst > 2 && fmod(loop_counter, 3)==0){
		obst_states_vary.resize(n_obst-1, 10);
		
		for (int i=0; i< n_obst-1; i++)
			obst_states_vary.row(i) = obst_states.row(i); 
	}else{
		obst_states_vary.resize(n_obst, 10);
		
		for (int i=0; i< n_obst; i++)
			obst_states_vary.row(i) = obst_states.row(i); 	
	}
	*/
	
	loop_counter++;
	
	
	// get next waypoint
	//std::vector<asv_msgs::WP>::iterator iter;// iterator
		//ROS_INFO("next_waypoint_ size: %0.2f   ", (double)next_waypoint_->size());
		
	if (n_nwps >0) { 
		
		next_waypoints.resize(n_nwps, 2); 
		//std::cout << "next_waypoint_ size : " << next_waypoint_->size() << std::endl; // check test!
		/*
		for (iter = next_waypoint_->begin(); iter != next_waypoint_->end(); ++iter){	
			j = std::distance(next_waypoint_->begin(), iter); 
			ROS_INFO("nwp_x: %0.2f   nwp_y: %0.2f", iter->x, iter->y);
			
			next_waypoints.row(j) << iter->x, iter->y;
		}
		*/		
			next_waypoints.row(0) << 0.0, 0.0;
	
	}
	
	
    	Eigen::Matrix<double, 1,4> static_obst;
    	static_obst << -50.0, 0.0, -50.0, 2050.0, //-40, 5, 1, 5;                // x_0, y_0, x_1, y_1	
	
	
	
	// compute best offset using drvo fxn
	//u_best=5; psi_best=1.57;
	//drvo->getBestControlInput(u_best, psi_best, u_d_, psi_d_, asv_state, obst_states, log_asv, log_obst_1, log_obst_2, log_obst_3); // MR interface output sign change

	drvo->getBestControlOffset(u_os_best, psi_os_best, u_d_, psi_d_, asv_state, obst_states_vary, static_obst, next_waypoints); // MR interface output sign change

// asv state
//ROS_INFO("asv_x: %0.2f  asv_y: %0.2f  asv_psi: %0.2f  asv_u: %0.2f  asv_v: %0.2f  asv_r: %0.2f", asv_pose_[0], asv_pose_[1], asv_pose_[2]*180.0f/M_PI, asv_twist_[0], asv_twist_[1], asv_twist_[2]);

// obstacle state
//ROS_INFO("obs_x: %0.2f  obs_y: %0.2f  obs_psi: %0.2f  obs_u: %0.2f  obs_v: %0.2f  obs_radius: %0.2f", obst_states(0,0), obst_states(0,1), obst_states(0,2)*180.0f/M_PI, obst_states(0,3), obst_states(0,4), obst_states(0,5));


 u_best = u_d_*u_os_best; psi_best = psi_d_ + psi_os_best;

ROS_INFO("u_d: %0.2f    psi_d: %0.2f   u: %0.2f    psi: %0.2f", u_d_, psi_d_*180.0f/M_PI, u_best, psi_best*180.0f/M_PI);

ROS_INFO("asv_u: %0.2f   asv_x: %0.2f   asv_y: %0.2f   asv_yaw: %0.2f", sqrt(asv_twist_[0]*asv_twist_[0] + asv_twist_[1]*asv_twist_[1]), asv_pose_[0], asv_pose_[1], asv_pose_[2]*180.0f/M_PI);

ROS_INFO("m_asv_u: %0.2f   m_asv_x: %0.2f   m_asv_y: %0.2f   m_asv_yaw: %0.2f", asv_state(3), asv_state(0), asv_state(1), asv_state(2)*180.0f/M_PI);

ROS_INFO("obst1_u: %0.2f   obst1_x: %0.2f   obst1_y: %0.2f   obst1_yaw: %0.2f", obst_u, obst_x, obst_y, obst_yaw*180.0f/M_PI);


ROS_INFO("m_obst1_u: %0.2f   m_obst1_x: %0.2f   m_obst1_y: %0.2f   m_obst1_yaw: %0.2f", sqrt(obst_states(0,3)*obst_states(0,3) + obst_states(0,4)*obst_states(0,4)), obst_states(0,0), obst_states(0,1), obst_states(0,2)*180.0f/M_PI);

ROS_INFO("m_obst1_u: %0.2f   m_obst1_v: %0.2f   ", obst_states(0,3), obst_states(0,4));

ROS_INFO("loop_counter: %0.2f   ", (double)loop_counter);

//ROS_INFO("%0.2f    %0.2f    %0.2f    %0.2f    %0.2f    %0.2f    %0.2f    %0.2f    %0.2f    %0.2f", dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos), dist_pos(generator_pos) );


//ROS_INFO("asv_yaw: %0.2f    obst1_yaw: %0.2f    obst2_yaw: %0.2f    obst3_yaw: %0.2f", asv_pose_[2]*180.0f/M_PI, obst_states(0,2)*180.0f/M_PI, obst_states(1,2)*180.0f/M_PI, obst_states(2,2)*180.0f/M_PI);

// check also: dist, cost, 


}

inline double dynamicRvoRosLink::normalize(double angle)
{
	while(angle <= -M_PI) angle += 2*M_PI; 
	while (angle > M_PI) angle -= 2*M_PI;
	return angle;
}

inline double dynamicRvoRosLink::normalize_angle_360(double angle){
	angle = fmod(angle,2*M_PI);
	if (angle < 0)
	angle += 2*M_PI;
	return angle;
}





