/****************************************************************************************
*
*  File name : sbmpc.hpp
*
*  Function  : Header file for the original Scenario-based Model Predictive Control.
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

#include "sbmpc_parameters.hpp"
#include "cpu/mpc_cost_cpu.hpp"
#if OWNSHIP_TYPE == 0
	#include "cpu/kinematic_ship_models_cpu.hpp"
#else 
	#include "cpu/kinetic_ship_models_cpu.hpp"
#endif


namespace PSBMPC_LIB
{
	class SBMPC
	{
	private:
		// Control behavior related vectors and the own-ship maneuver times in the prediction horizon
		Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

		// Previous optimal offsets/modifications
		double u_opt_last;
		double chi_opt_last;

		// Cost at the optimal solution
		double min_cost;

		// Own-ship predicted trajectory
		Eigen::MatrixXd trajectory;

		CPU::Ownship ownship;

		void reset_control_behaviour();

		void increment_control_behaviour();

		void setup_prediction(Obstacle_Data<Tracked_Obstacle> &data);

		bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

		//
		void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

	public:

		SBMPC_Parameters pars;

		CPU::MPC_Cost<SBMPC_Parameters> mpc_cost;

		SBMPC();
			
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
	};
} 