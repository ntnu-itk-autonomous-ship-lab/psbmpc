/****************************************************************************************
*
*  File name : cb_cost_functor_structures.cuh
*
*  Function  : Header file for the data/parameter structures used in the control behaviour 
*			   cost functor. Used in the thrust framework for GPU calculations.
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

#include "obstacle_manager.h"
#include "kinematic_ship_models_gpu.cuh"
#include "kinetic_ship_models_gpu.cuh"
#include "tml.cuh"
#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		/****************************************************************************************
		*  Name     : Obstacle_Data_GPU_Friendly
		*  Function : Data used by prediction obstacles in the joint prediction scheme in the 
		*			  GPU version of the PSBMPC
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		struct Obstacle_Data_GPU_Friendly
		{
			// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
			// and <obstacle is passed> (IP_0) indicators
			TML::PDMatrix<bool, MAX_N_OBST, 1> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

			// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
			TML::PDMatrix<ST, MAX_N_OBST, 1> ST_0, ST_i_0;

			__host__ __device__ Obstacle_Data_GPU_Friendly() {}

			__host__ __device__ ~Obstacle_Data_GPU_Friendly() {}
		};

		/****************************************************************************************
		*  Name     : CB_Functor_Pars
		*  Function : Struct containing a subset of the PSB-MPC parameters for use in the GPU.
		*			  Read-only data.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		struct CB_Functor_Pars
		{
			int n_M;

			CPE_Method cpe_method;

			Prediction_Method prediction_method;

			Guidance_Method guidance_method;

			float T, T_static, dt, p_step;
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
			
			bool obstacle_colav_on;

			__host__ __device__ CB_Functor_Pars() {}

			__host__ __device__ CB_Functor_Pars(const PSBMPC_Parameters &pars)
			{
				this->n_M = pars.n_M;

				this->cpe_method = pars.cpe_method;

				this->prediction_method = pars.prediction_method;

				this->guidance_method = pars.guidance_method;

				this->T = pars.T; this->T_static = pars.T_static; this->dt = pars.dt; this->p_step = pars.p_step; 

				this->d_safe = pars.d_safe; this->d_close = pars.d_close; this->d_init = pars.d_init;

				this->K_coll = pars.K_coll;

				this->phi_AH = pars.phi_AH; this->phi_OT = pars.phi_OT; this->phi_HO = pars.phi_HO; this->phi_CR = pars.phi_CR;

				this->kappa = pars.kappa; this->kappa_TC = pars.kappa_TC;

				this->K_u = pars.K_u; this->K_du = pars.K_du;

				this->K_chi_strb = pars.K_chi_strb; this->K_dchi_strb = pars.K_dchi_strb;
				this->K_chi_port = pars.K_chi_port; this->K_dchi_port = pars.K_dchi_port;

				this->K_sgn = pars.K_sgn; this->T_sgn = pars.T_sgn;

				this->G = pars.G;

				this->q = pars.q; this->p = pars.p;

				this->obstacle_colav_on = pars.obstacle_colav_on;
			}
		};

		/****************************************************************************************
		*  Name     : CB_Functor_Data
		*  Function : Class containing read-only data/classes/information needed to evaluate 
		*			  the cost of one PSB-MPC control behaviour.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		class CB_Functor_Data
		{
		public:
			float ownship_length;

			TML::PDVector6f ownship_state;

			TML::PDMatrix<float, MAX_N_M, 1> maneuver_times;

			float u_d, chi_d;

			float u_opt_last;
			float chi_opt_last;

			bool use_joint_prediction;

			int wp_c_0;

			TML::PDMatrix<float, 2, MAX_N_WPS> waypoints;

			TML::PDMatrix<float, 4, MAX_N_OBST> static_obstacles;

			int n_obst; 

			// Number of prediction scenarios for each obstacle, includes the intelligent prediction scenario
			// if not pruned away
			TML::PDMatrix<int, MAX_N_OBST, 1> n_ps;

			// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
			// and <obstacle is passed> (IP_0) indicators
			TML::PDMatrix<bool, MAX_N_OBST, 1> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0; 

			//=======================================================================================
			//  Name     : CB_Functor_Data
			//  Function : Class constructor
			//  Author   : 
			//  Modified :
			//=======================================================================================
			__host__ CB_Functor_Data() {}

			__host__ CB_Functor_Data(
				const Eigen::MatrixXd &trajectory,
				const Eigen::VectorXd &maneuver_times,
				const double u_opt_last,
				const double chi_opt_last,
				const double u_d, 
				const double chi_d, 
				const bool use_joint_prediction,
				const int wp_c_0,
				const double ownship_length,
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const Eigen::Matrix<double, 4, -1> &static_obstacles,
				const std::vector<int> &n_ps,
				const Obstacle_Data<Tracked_Obstacle> &data)
			{
				this->ownship_length = ownship_length;
				
				this->wp_c_0 = wp_c_0;

				TML::assign_eigen_object(ownship_state, trajectory.col(0));

				TML::assign_eigen_object(this->maneuver_times, maneuver_times);

				this->u_d = u_d;
				this->chi_d = chi_d;

				this->u_opt_last = u_opt_last;
				this->chi_opt_last = chi_opt_last;	

				this->use_joint_prediction = use_joint_prediction;

				TML::assign_eigen_object(this->waypoints, waypoints);	

				TML::assign_eigen_object(this->static_obstacles, static_obstacles);

				n_obst = data.obstacles.size();

				this->n_ps.resize(n_obst, 1);

				AH_0.resize(n_obst, 1); 	S_TC_0.resize(n_obst, 1); S_i_TC_0.resize(n_obst, 1);
				O_TC_0.resize(n_obst, 1); 	Q_TC_0.resize(n_obst, 1); IP_0.resize(n_obst, 1);
				H_TC_0.resize(n_obst, 1); 	X_TC_0.resize(n_obst, 1);

				for (int i = 0; i < n_obst; i++)
				{
					this->n_ps[i] = n_ps[i];

					AH_0[i] = data.AH_0[i]; 
					S_TC_0[i] = data.S_TC_0[i];
					S_i_TC_0[i] = data.S_i_TC_0[i]; 
					O_TC_0[i] = data.O_TC_0[i];
					Q_TC_0[i] = data.Q_TC_0[i]; 
					IP_0[i] = data.IP_0[i];
					H_TC_0[i] = data.H_TC_0[i]; 
					X_TC_0[i] = data.X_TC_0[i];
				} 
			}
		};
	}
}