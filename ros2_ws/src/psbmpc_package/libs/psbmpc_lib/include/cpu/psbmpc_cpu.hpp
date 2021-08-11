/****************************************************************************************
*
*  File name : psbmpc_cpu.hpp
*
*  Function  : Header file for Probabilistic Scneario-based Model Predictive Control
*			   on the CPU.
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

#include "psbmpc_defines.hpp"
#include "psbmpc_parameters.hpp"
#include "obstacle_predictor.hpp"
#include "cpu/obstacle_sbmpc_cpu.hpp"
#if OWNSHIP_TYPE == 0
	#include "cpu/kinematic_ship_models_cpu.hpp"
#else 
	#include "cpu/kinetic_ship_models_cpu.hpp"
#endif
#include "cpu/cpe_cpu.hpp"

#include <vector>
#include <memory>	

namespace PSBMPC_LIB
{
	namespace CPU
	{
		class PSBMPC
		{
		private:
			// Amount of prediction scenarios for each obstacle
			std::vector<int> n_ps;

			// Control behavior related vectors and the own-ship maneuver times in the prediction horizon
			Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

			// Previous optimal offsets/modifications
			double u_opt_last;
			double chi_opt_last;

			// Cost at the optimal solution
			double min_cost;

			// Own-ship predicted trajectory
			Eigen::MatrixXd trajectory;

			Ownship ownship;

			CPE cpe;

			std::vector<Prediction_Obstacle> pobstacles;

			bool use_joint_prediction;

			bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

			void reset_control_behaviour();

			void increment_control_behaviour();

			void setup_prediction(Obstacle_Data<Tracked_Obstacle> &data);

			void prune_obstacle_scenarios(Obstacle_Data<Tracked_Obstacle> &data);

			void calculate_collision_probabilities(
				Eigen::MatrixXd &P_c_i, 
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double dt, 
				const int p_step);
				
			void calculate_ps_collision_probabilities(Eigen::VectorXd &P_c_i_ps, const Eigen::MatrixXd &P_c_i, const int i);

			void calculate_ps_collision_consequences(Eigen::VectorXd &C_i, const Obstacle_Data<Tracked_Obstacle> &data, const int i, const double dt, const int p_step);

			void calculate_ps_collision_risks(
				Eigen::VectorXd &R_c_i, 
				Eigen::VectorXi &indices_i, 
				const Eigen::VectorXd &C_i, 
				const Eigen::VectorXd &P_c_i_ps, 
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i);

			void predict_trajectories_jointly(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1>& static_obstacles, const bool overwrite);
			void predict_trajectories_jointly(Obstacle_Data<Tracked_Obstacle> &data, const std::vector<polygon_2D> &polygons, const int n_static_obst, const bool overwrite);

			void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

		public:

			PSBMPC_Parameters pars;

			MPC_Cost<PSBMPC_Parameters> mpc_cost;

			PSBMPC();

			// Resets previous optimal offsets and predicted own-ship waypoint following
			void reset() { u_opt_last = 1.0; chi_opt_last = 0.0; ownship.set_wp_counter(0); }

			// For use when grounding hazards are simplified as straight lines
			void calculate_optimal_offsets(
				double &u_opt, 
				double &chi_opt, 
				Eigen::Matrix<double, 2, -1> &predicted_trajectory,
				const double u_d, 
				const double chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::VectorXd &ownship_state,
				const Eigen::Matrix<double, 4, -1> &static_obstacles,
				Obstacle_Data<Tracked_Obstacle> &data);

			// For use when reading grounding hazards as polygons from shapefiles
			void calculate_optimal_offsets(
				double &u_opt, 
				double &chi_opt, 
				Eigen::Matrix<double, 2, -1> &predicted_trajectory,
				const double u_d, 
				const double chi_d, 
				const Eigen::Matrix<double, 2, -1> &waypoints,
				const Eigen::VectorXd &ownship_state,
				const double V_w,
				const Eigen::Vector2d &wind_direction,
				const std::vector<polygon_2D> &polygons,
				Obstacle_Data<Tracked_Obstacle> &data);

		};
	}
}