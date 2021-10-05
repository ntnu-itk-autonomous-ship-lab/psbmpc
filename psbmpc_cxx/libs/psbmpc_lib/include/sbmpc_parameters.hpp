/****************************************************************************************
*
*  File name : sbmpc_parameters.hpp
*
*  Function  : Header file for the SB-MPC parameter class.
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

#include "sbmpc_index.hpp"
#include "psbmpc_parameters.hpp"

namespace PSBMPC_LIB
{
	class SBMPC;
	class Obstacle_Predictor;
	class Obstacle_Manager;
	namespace CPU
	{
		template <typename Parameters> class MPC_Cost;
		class Obstacle_SBMPC;
	}

	class SBMPC_Parameters
	{
	private:

		friend class SBMPC;
		friend class Obstacle_Manager;
		friend class CPU::MPC_Cost<SBMPC_Parameters>;
		friend class CPU::Obstacle_SBMPC;
		
		// Number of control behaviours and sequential maneuvers for the ownship, respectively
		int n_cbs, n_M, n_r;

		int p_step, p_step_grounding;
		
		// Finite sets of offsets considered to the own-ship surge and course references,
		// for each maneuver in the horizon
		std::vector<Eigen::VectorXd> u_offsets;
		std::vector<Eigen::VectorXd> chi_offsets;

		Prediction_Method prediction_method;

		Guidance_Method guidance_method;

		double T, dt;
		double t_ts;
		double d_safe, d_close, d_init, d_so_relevant;
		double K_coll;
		double phi_AH, phi_OT, phi_HO, phi_CR;
		double kappa, kappa_TC;
		double K_u, K_du;
		double K_chi_strb, K_dchi_strb;
		double K_chi_port, K_dchi_port; 
		double K_sgn, T_sgn;
		double G_1, G_2, G_3, G_4;
		double epsilon_rdp;
		double q, p;

		void initialize_pars(const bool is_obstacle_sbmpc);
		void initialize_pars(
			const std::vector<std::vector<double>> &u_offsets, 
			const std::vector<std::vector<double>> &chi_offsets, 
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars, 
			const std::vector<double> &dpars);

	public:

		SBMPC_Parameters() {}

		SBMPC_Parameters(const bool is_obstacle_sbmpc) { initialize_pars(is_obstacle_sbmpc); }
		SBMPC_Parameters(
			const std::vector<std::vector<double>> &u_offsets, 
			const std::vector<std::vector<double>> &chi_offsets, 
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars, 
			const std::vector<double> &dpars)
		{
			initialize_pars(u_offsets, chi_offsets, prediction_method, guidance_method, ipars, dpars);
		}
	};
}