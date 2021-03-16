/****************************************************************************************
*
*  File name : obstacle_sbmpc_parameters.cuh
*
*  Function  : Header file for the Obstacle SB-MPC parameter class.
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

#include "psbmpc_parameters.h"
#include "psbmpc_defines.h"
#include "tml.cuh"

namespace PSBMPC_LIB
{
	namespace GPU
	{
		template <typename Parameters> class MPC_Cost;
		class Obstacle_SBMPC;
	}

	class Obstacle_SBMPC_Parameters
	{
	private:
		friend class GPU::MPC_Cost<Obstacle_SBMPC_Parameters>;
		friend class GPU::Obstacle_SBMPC;

		int n_cbs, n_M;

		TML::PDMatrix<float, MAX_N_M, MAX_N_U_OFFSETS_OSBMPC> u_offsets;
		TML::PDMatrix<float, MAX_N_M, MAX_N_CHI_OFFSETS_OSBMPC> chi_offsets;
		TML::PDMatrix<int, 2, MAX_N_M> offsets_size;

		Prediction_Method prediction_method;

		Guidance_Method guidance_method;

		float T, T_static, dt, p_step;
		float t_ts;
		float d_safe, d_close, d_init;
		float K_coll;
		float phi_AH, phi_OT, phi_HO, phi_CR;
		float kappa, kappa_TC;
		float K_u, K_du;
		float K_chi_strb, K_dchi_strb;
		float K_chi_port, K_dchi_port; 
		float K_sgn, T_sgn;
		float G;
		float q, p;

		void initialize_pars();

	public:

		Obstacle_SBMPC_Parameters() { initialize_pars(); }

		Obstacle_SBMPC_Parameters(std::string tuning_file); // Not implemented yet
	};

}