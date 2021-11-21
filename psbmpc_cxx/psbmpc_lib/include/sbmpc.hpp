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
		Eigen::VectorXd offset_sequence, maneuver_times;
		Eigen::VectorXi offset_sequence_counter;

		// Previous optimal offsets/modifications
		double u_opt_last;
		double chi_opt_last;

		double min_cost;

		Eigen::MatrixXd trajectory;

		CPU::Ownship ownship;

		Transitional_Variables tv;

		void reset_control_behaviour();

		void increment_control_behaviour();

		void update_transitional_variables(
			const Eigen::VectorXd &ownship_state, 
			const Dynamic_Obstacles &obstacles);

		void setup_prediction(const Dynamic_Obstacles &obstacles);

		bool determine_colav_active(const Dynamic_Obstacles &obstacles, const int n_static_obst, const bool disable);

		void assign_optimal_trajectory(Eigen::MatrixXd &optimal_trajectory);

	public:

		SBMPC_Parameters pars;

		CPU::MPC_Cost<SBMPC_Parameters> mpc_cost;

		SBMPC();
		SBMPC(const CPU::Ownship &ownship, const SBMPC_Parameters &pars);
			
		void calculate_optimal_offsets(
			double &u_opt, 
			double &chi_opt, 
			Eigen::MatrixXd &predicted_trajectory,
			const double u_d, 
			const double chi_d, 
			const Eigen::Matrix<double, 2, -1> &waypoints,
			const Eigen::VectorXd &ownship_state,
			const Eigen::Matrix<double, 4, -1> &static_obstacles,
			const Dynamic_Obstacles &obstacles,
			const bool disable);

		void calculate_optimal_offsets(
			double &u_opt, 
			double &chi_opt, 
			Eigen::MatrixXd &predicted_trajectory,
			const double u_d, 
			const double chi_d, 
			const Eigen::Matrix<double, 2, -1> &waypoints,
			const Eigen::VectorXd &ownship_state,
			const double V_w,
			const Eigen::Vector2d &wind_direction,
			const Static_Obstacles &polygons,
			const Dynamic_Obstacles &obstacles,
			const bool disable);
	};
} 