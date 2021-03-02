#ifndef ASV_CTRL_DRVO
#define ASV_CTRL_DRVO

#include <Eigen/Dense>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
//#include "asv_ctrl_drvo/obstacle.h"
//#include "asv_ctrl_drvo/shipModel.h"

#include "drvo_lib/drvoAgent.h" // in the lib folder!

#include "iostream"
#include "fstream" 

using namespace std;

class dynamicRvoRosLink
{

	
	public:
		/// Constructor
		dynamicRvoRosLink();
		/// Destructor
		~dynamicRvoRosLink();
		
		/**
		* @brief Initializes the controller.
		*
		* @param obstacles A pointer to a vector of obstacles provided by the ROS 
		* wrapper (by subscribing to an obstacle tracker node).
		* @param map A pointer to the occupancy grid published by the map_server.
		*/
		void initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map);
		
		/**
		* @brief Callback for updating the internal ASV state (data provided by ROS wrapper).
		*
		* @param msg The Odometry message which contains the state data.
		* @param u_d The desired surge speed set point provided by, e.g., a LOS algorithm.
		* @param psi_d The desired heading set point provided by, e.g., a LOS algorithm.
		*/		
		void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg,
							const double &u_d,
							const double &psi_d);

		/**
		* @brief Called after the velocity field has been updated to get the (u, psi) pair
		* with the lowest cost.
		*
		* @param u_best The reference parameter to store the "best" surge speed.
		* @param psi_best The reference parameter to store the "best" heading.
		*/
		//void getBestControlInput(double &u_best, double &psi_best, ofstream &log_asv, ofstream &log_obst_1, ofstream &log_obst_2, ofstream &log_obst_3);
		
		void getBestControlOffset(double &u_os_best, double &psi_os_best);
		
		
		/* normalize angle */
		inline double normalize(double angle);
		inline double normalize_angle_360(double angle);
	
		
	private:
				
		Eigen::Vector3d asv_pose_;
		Eigen::Vector3d asv_twist_;
		double u_d_;
		double psi_d_;
				
		std::vector<asv_msgs::State> *obstacles_;
		nav_msgs::OccupancyGrid *map_;
		nav_msgs::OccupancyGrid local_map_;
		ros::Publisher lm_pub;


		//shipModel *asv; // use similar to keep the resources of the dynamicRvo object
		Agent *drvo;

		//double cost;
				
		
		//int n_samp;

			
};

#endif 
