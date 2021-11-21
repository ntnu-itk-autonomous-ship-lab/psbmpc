/****************************************************************************************
*
*  File name : obstacle_manager.hpp
*
*  Function  : Header file for the obstacle management interface and data structure for
*			   keeping information on dynamic obstacles. As of now only
*			   used for dynamic obstacle management.
*
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#include "tracked_obstacle.hpp"

#include <vector>
#include <Eigen/Dense>
#include <string>

namespace PSBMPC_LIB
{
	using Dynamic_Obstacles = std::vector<Tracked_Obstacle, Eigen::aligned_allocator<Tracked_Obstacle>>;

	class Obstacle_Manager
	{
	private:

		double T_lost_limit, T_tracked_limit;

		bool obstacle_filter_on;

		Dynamic_Obstacles obstacles;
		Dynamic_Obstacles new_obstacles;

		/****************************************************************************************
		*  Name     : update_obstacles
		*  Function : Takes in new obstacle information and updates the single obstacle data 
		*			  structures.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Parameters>
		void update_obstacles(
			const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
			const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
			const MPC_Parameters &mpc_pars,														// In: Parameters of calling MPC (either PSB-MPC or SB-MPC)
			const double dt_prev_upd															// In: Time difference between now and last obstacle update
			) 			
		{
			int n_do_old = obstacles.size();
			int n_do_new = obstacle_states.cols();

			bool obstacle_exist;
			for (int i = 0; i < n_do_new; i++)
			{
				obstacle_exist = false;
				for (int j = 0; j < n_do_old; j++)
				{
					if ((double)obstacles[j].get_ID() == obstacle_states(8, i))
					{
						obstacles[j].reset_duration_lost();

						obstacles[j].update(
							obstacle_states.col(i), 
							obstacle_covariances.col(i), 
							obstacle_filter_on,
							dt_prev_upd);

						new_obstacles.push_back(std::move(obstacles[j]));

						obstacle_exist = true;

						break;
					}
				}
				if (!obstacle_exist)
				{
					new_obstacles.push_back(std::move(Tracked_Obstacle(
						obstacle_states.col(i), 
						obstacle_covariances.col(i),
						obstacle_filter_on,  
						mpc_pars.T, 
						mpc_pars.dt)));
				}
			}
			// Keep terminated obstacles that may still be relevant, and compute duration lost as input to the cost of collision risk
			// Obstacle track may be lost due to sensor/detection failure, or the obstacle may go out of COLAV-target range
			// Detection failure will lead to the start (creation) of a new track (obstacle), typically after a short duration,
			// whereas an obstacle that is out of COLAV-target range may re-enter range with the same id.
			if (obstacle_filter_on)
			{
				for (size_t j = 0; j < obstacles.size(); j++)
				{
					obstacles[j].increment_duration_lost(dt_prev_upd);

					if (obstacles[j].get_duration_tracked() >= T_tracked_limit 	&&
						(obstacles[j].get_duration_lost() < T_lost_limit || obstacles[j].kf.get_covariance()(0,0) <= 5.0))
					{
						obstacles[j].update(obstacle_filter_on, dt_prev_upd);

						new_obstacles.push_back(std::move(obstacles[j]));
					}
				}
			}
			// Then, after having all relevant obstacles (includes transferred ones from the old vector, and newly detected ones)
			// in "new_obstacles", transfer all of these to the "obstacle" vector to be used by other classes.
			obstacles = std::move(new_obstacles);	
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Obstacle_Manager() : T_lost_limit(15.0), T_tracked_limit(15.0), obstacle_filter_on(false) {}

		Obstacle_Manager(const double T_lost_limit, const double T_tracked_limit, const bool obstacle_filter_on)
			: T_lost_limit(T_lost_limit), T_tracked_limit(T_tracked_limit), obstacle_filter_on(obstacle_filter_on) {}

		Dynamic_Obstacles& get_data() { return obstacles; }

		/****************************************************************************************
		*  Name     : operator()
		*  Function : Manages new obstacle information, updates data structures and situation
		*			  information accordingly 
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Parameters>
		void operator()(
			const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
			const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
			const MPC_Parameters &mpc_pars,														// In: Parameters of calling MPC (either PSB-MPC or SB-MPC)
			const double dt_prev_upd															// In: Time difference between now and last obstacle update
			) 			
		{
			update_obstacles(obstacle_states, obstacle_covariances, mpc_pars, dt_prev_upd);
		}
	};
}