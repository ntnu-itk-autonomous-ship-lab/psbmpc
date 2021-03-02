#ifndef ASV_CTRL_SB_MPC
#define ASV_CTRL_SB_MPC

#include <Eigen/Dense>

#include "asv_msgs/State.h"
#include "asv_msgs/WP.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
//#include "asv_ctrl_sb_mpc/obstacle.h"
//#include "asv_ctrl_sb_mpc/shipModel.h"

#include "sb_mpc_lib/sb_mpc.h" // in the lib folder!


class simulationBasedMpcRosLink
{

	
	public:
		/// Constructor
		simulationBasedMpcRosLink();
		/// Destructor
		~simulationBasedMpcRosLink();
		
		/**
		* @brief Initializes the controller.
		*
		* @param obstacles A pointer to a vector of obstacles provided by the ROS 
		* wrapper (by subscribing to an obstacle tracker node).
		* @param map A pointer to the occupancy grid published by the map_server.
		*/
		void initialize(std::vector<asv_msgs::State> *obstacles, std::vector<asv_msgs::WP> *next_waypoint, nav_msgs::OccupancyGrid *map);
		
		/**
		* @brief Callback for updating the internal ASV state (data provided by ROS wrapper).
		*
		* @param msg The Odometry message which contains the state data.
		* @param u_d The desired surge speed set point provided by, e.g., a LOS algorithm.
		* @param psi_d The desired heading set point provided by, e.g., a LOS algorithm.
		*/		
		void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg,
							const double &u_d,
							const double &psi_d,
							std::vector<asv_msgs::WP> *next_waypoint
							);

		/**
		* @brief Called after the velocity field has been updated to get the (u, psi) pair
		* with the lowest cost.
		*
		* @param u_best The reference parameter to store the "best" surge speed.
		* @param psi_best The reference parameter to store the "best" heading.
		*/
		void getBestControlOffset(double &u_os_best, double &psi_os_best);
		
		
		
		/* normalize angle */
		inline double normalize(double angle);
		inline double normalize_angle_360(double angle);
		
		
	private:
				
		Eigen::Vector3d asv_pose_;
		Eigen::Vector3d asv_twist_;
		double u_d_;
		double psi_d_;
				
		std::vector<asv_msgs::WP> *next_waypoint_;
		
		std::vector<asv_msgs::State> *obstacles_;
		nav_msgs::OccupancyGrid *map_;
		nav_msgs::OccupancyGrid local_map_;
		ros::Publisher lm_pub;


		//shipModel *asv; // use similar to keep the resources of the simualationBasedMpc object
		simulationBasedMpc *sb_mpc;

		//double cost;
				
		
		//int n_samp;

			
};

#endif 
