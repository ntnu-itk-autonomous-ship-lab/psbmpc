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
#include "tml/tml.cuh"

#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Cuda_Obstacle;
		class CPE;
		template <typename Parameters> class MPC_Cost;

		/****************************************************************************************
		*  Name     : CB_Cost_Functor_1
		*  Function : Functor used to predict the own-ship trajectory
		*			  following a certain control behaviour, and calculate the path related costs
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
			float h_path; 

			unsigned int cb_index;
			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

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
		*  Function : Functor used to calculate the partial grounding cost, dynamic obstacle cost
		*			  and COLREGS cost.
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

			Basic_Polygon *polygons;

			Cuda_Obstacle *obstacles;

			CPE *cpe;

			Ownship *ownship;

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory;

			MPC_Cost<CB_Functor_Pars> *mpc_cost;
			//==============================================

			//==============================================
			// Pre-allocated temporaries (local to the thread stack)
			//==============================================
			int thread_index, cb_index, j, i, ps, cpe_index;
			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

			int n_samples, n_seg_samples;

			float max_h_so_j, max_cost_i_ps, mu_i_ps, cost_k, P_c_i;
			bool mu_k;

			float d_safe_i, chi_m;

			thrust::tuple<float, bool> tup;

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
			//==============================================

		public: 
			__host__ CB_Cost_Functor_2() : 
				pars(nullptr), 
				fdata(nullptr), 
				polygons(nullptr),
				obstacles(nullptr), 
				cpe(nullptr), 
				ownship(nullptr), 
				trajectory(nullptr), 
				mpc_cost(nullptr) 
			{}

			__host__ CB_Cost_Functor_2(
				CB_Functor_Pars *pars, 
				CB_Functor_Data *fdata, 
				Basic_Polygon *polygons,
				Cuda_Obstacle *obstacles, 
				CPE *cpe,
				Ownship *ownship,
				TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory,
				MPC_Cost<CB_Functor_Pars> *mpc_cost):
				pars(pars), fdata(fdata), polygons(polygons), obstacles(obstacles), cpe(cpe), 
				ownship(ownship), trajectory(trajectory), mpc_cost(mpc_cost) 
				{}

			__host__ __device__ ~CB_Cost_Functor_2() 
			{ 
				pars = nullptr; 
				fdata = nullptr; 
				polygons = nullptr;
				obstacles = nullptr; 
				cpe = nullptr; 
				ownship = nullptr;
				trajectory = nullptr; 
				mpc_cost = nullptr;
			}

			__device__ thrust::tuple<float, float, float> operator()(
				const thrust::tuple<const int, TML::PDMatrix<float, 2 * MAX_N_M, 1>, const int, const int, const int, const int, const int> &input_tuple);
		};
	}
}