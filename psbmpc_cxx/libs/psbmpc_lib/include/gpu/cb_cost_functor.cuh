/****************************************************************************************
*
*  File name : cb_cost_functor.cuh
*
*  Function  : Header file for the control behaviour cost evaluation functors. Used in
*			   the thrust framework for GPU calculations.
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

#include "cb_cost_functor_structures.cuh"

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Cuda_Obstacle;
		class Prediction_Obstacle;
		class CPE;
		class Obstacle_SBMPC;
		template <typename Parameters> class MPC_Cost;

		/****************************************************************************************
		*  Name     : CB_Cost_Functor_1
		*  Function : Functor used to predict the own-ship trajectory
		*			  following a certain control behaviour, and calculate the static obstacle
		*			  and path related costs
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		class CB_Cost_Functor_1
		{
		private: 

			//==============================================
			// Members allocated in global device memory
			//==============================================
			CB_Functor_Pars *pars;

			CB_Functor_Data *fdata;

			Ownship *ownship;

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory;

			MPC_Cost<CB_Functor_Pars> *mpc_cost;

			//==============================================
			// Pre-allocated temporaries (local to the thread stack)
			//==============================================
			float cost_cb; 

			unsigned int cb_index;
			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

			int n_samples;

		public: 
			__host__ CB_Cost_Functor_1() : 
				pars(nullptr), 
				fdata(nullptr), 
				ownship(nullptr), 
				trajectory(nullptr),
				mpc_cost(nullptr)
			{}

			__host__ CB_Cost_Functor_1(
				CB_Functor_Pars *pars, 
				CB_Functor_Data *fdata, 
				Ownship *ownship,
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory,
				MPC_Cost<CB_Functor_Pars> *mpc_cost) :
				pars(pars), fdata(fdata), ownship(ownship), trajectory(trajectory), mpc_cost(mpc_cost)
			{}

			__host__ __device__ ~CB_Cost_Functor_1() 
			{ 
				pars = nullptr; 
				fdata = nullptr; 
				ownship = nullptr;
				trajectory = nullptr; 
				mpc_cost = nullptr;
			}
			
			__device__ float operator()(const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple);

		};

		/****************************************************************************************
		*  Name     : CB_Cost_Functor_2
		*  Function : Functor used to calculate the dynamic obstacle cost, for a control behaviour
		*			  l, an obstacle i behaving as in prediction scenario ps. 
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		class CB_Cost_Functor_2
		{
		private: 

			//==============================================
			// Members allocated in global device memory
			//==============================================
			CB_Functor_Pars *pars;

			CB_Functor_Data *fdata;

			Cuda_Obstacle *obstacles;

			Prediction_Obstacle *pobstacles;

			CPE *cpe;

			Ownship *ownship;

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory;

			Obstacle_Ship *obstacle_ship;

			Obstacle_SBMPC *obstacle_sbmpc;

			MPC_Cost<CB_Functor_Pars> *mpc_cost;

			//==============================================

			//==============================================
			// Pre-allocated temporaries (local to the thread stack)
			//==============================================
			unsigned int thread_index;
			unsigned int cb_index;
			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;
			unsigned int i, ps, jp_thread_index;

			int n_samples, n_seg_samples, p_step;

			float max_cost_ps, cost_ps, P_c_i;

			float d_safe_i, chi_m;

			Intention a_i_ps_jp; bool mu_i_ps_jp;

			// Allocate predicted ownship state and predicted obstacle i state and covariance for their prediction scenarios (ps)
			// Only keeps n_seg_samples at a time, sliding window. Minimum 2
			// If cpe_method = MCSKF, then dt_seg must be equal to dt;
			// If cpe_method = CE, then only the first column in these matrices are used (only the current predicted time is considered)
			TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_p_seg; 
			TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_i_p_seg;
			TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> P_i_p_seg;

			// For the CE-method:
			TML::Vector2f p_os, p_i, v_os_prev, v_i_prev;
			TML::Matrix2f P_i_2D;

			// Joint prediction related
			TML::PDMatrix<float, MAX_N_OBST, 1> u_d_i, u_opt_last_i, chi_d_i, chi_opt_last_i;

			TML::PDMatrix<float, 7, 1> xs_os_aug_k;
			
			TML::Vector2f v_os_k, v_A, v_B, L_AB, p_A, p_B;
			TML::Vector4f xs_i_p, xs_i_p_transformed;

			float t, chi_i, psi_A, psi_B, d_AB, u_opt_i, chi_opt_i; 

			Obstacle_Data_GPU_Friendly data;

			int i_count;

			//==============================================
			

			__device__ void update_conditional_obstacle_data(const int i_caller, const int k);

			__device__ void predict_trajectories_jointly();

		public: 
			__host__ CB_Cost_Functor_2() : 
				pars(nullptr), 
				fdata(nullptr), 
				obstacles(nullptr), 
				pobstacles(nullptr), 
				cpe(nullptr), 
				ownship(nullptr), 
				trajectory(nullptr), 
				obstacle_ship(nullptr), 
				obstacle_sbmpc(nullptr), 
				mpc_cost(nullptr) 
			{}

			__host__ CB_Cost_Functor_2(
				CB_Functor_Pars *pars, 
				CB_Functor_Data *fdata, 
				Cuda_Obstacle *obstacles, 
				Prediction_Obstacle *pobstacles,
				CPE *cpe,
				Ownship *ownship,
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory,
				Obstacle_Ship *obstacle_ship,
				Obstacle_SBMPC *obstacle_sbmpc,
				MPC_Cost<CB_Functor_Pars> *mpc_cost);

			__host__ __device__ ~CB_Cost_Functor_2() 
			{ 
				pars = nullptr; 
				fdata = nullptr; 
				obstacles = nullptr; 
				pobstacles = nullptr; 
				cpe = nullptr; 
				trajectory = nullptr; 
				obstacle_ship = nullptr; 
				obstacle_sbmpc = nullptr; 
				mpc_cost = nullptr;
			}

			// Used when flattening the nested for loops over obstacles and their prediction scenario into a single loop 
			// merged with the control behaviour loop => then calulate the maximum dynamic obstacle cost of a control behaviour for the own-ship
			// considering obstacle i in prediction scenario ps. 
			// The intention and COLREGS violation indicator for the intelligent obstacle prediction (if active) is also returned from the thread.
			__device__ thrust::tuple<float, Intention, bool> operator()(const thrust::tuple<
				const unsigned int, 
				TML::PDMatrix<float, 2 * MAX_N_M, 1>, 
				const unsigned int, 
				const unsigned int, 
				const unsigned int, 
				const int> &input_tuple);
			
		};
	}
}