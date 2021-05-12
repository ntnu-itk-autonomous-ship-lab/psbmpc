/****************************************************************************************
*
*  File name : cb_cost_functor.cu
*
*  Function  : Class functions for the control behaviour cost evaluation functors. Used in
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

#include "psbmpc_defines.hpp"
#include "gpu/cb_cost_functor.cuh"
#include "gpu/cuda_obstacle.cuh"
#include "gpu/cpe_gpu.cuh"
#include "gpu/obstacle_sbmpc_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include "gpu/prediction_obstacle_gpu.cuh"

#include <cmath>
#include <iostream>

namespace PSBMPC_LIB
{
namespace GPU
{
//=======================================================================================
//  CB COST FUNCTOR 1 METHODS
//=======================================================================================

//=======================================================================================
//  Name     : operator()
//  Function : Predicts the trajectory of the ownship for a certain control behaviour,
//			   and calculates the static obstacle and path related costs
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ thrust::tuple<float, float> CB_Cost_Functor_1::operator()(
	const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple	// In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
	)
{
	h_so= 0.0f; h_path = 0.0f;
	cb_index = thrust::get<0>(cb_tuple);
	offset_sequence = thrust::get<1>(cb_tuple);

	//======================================================================================================================
	// 1.1 : Setup and own-ship trajectory prediction with the control behaviour cb_index
	n_samples = round(pars->T / pars->dt);

	ownship[cb_index].set_wp_counter(fdata->wp_c_0);

	trajectory[cb_index].resize(4, n_samples);

	ownship[cb_index].predict_trajectory(
		trajectory[cb_index], 
		fdata->ownship_state,
		offset_sequence, 
		fdata->maneuver_times, 
		fdata->u_d, fdata->chi_d, 
		fdata->waypoints, 
		pars->prediction_method, 
		pars->guidance_method, 
		pars->T, pars->dt);

	//==================================================================================================
	// 2.1 : Calculate cost due to driving the boat on land or static objects
	//h_so = mpc_cost[cb_index].calculate_grounding_cost(); 

	//==================================================================================================
	// 2.2 : Calculate cost due to deviating from the nominal path
	h_path += mpc_cost[cb_index].calculate_control_deviation_cost(offset_sequence, fdata->u_opt_last, fdata->chi_opt_last);

	//==================================================================================================
	// 2.3 : Calculate cost due to having a wobbly offset_sequence
	h_path += mpc_cost[cb_index].calculate_chattering_cost(offset_sequence, fdata->maneuver_times); 

	return thrust::tuple<float, float>(h_so, h_path);
}


//=======================================================================================
//  CB COST FUNCTOR 2 METHODS
//=======================================================================================

/****************************************************************************************
*  Name     : CB_Cost_Functor
*  Function : Class constructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ CB_Cost_Functor_2::CB_Cost_Functor_2(
	CB_Functor_Pars *pars,  										// In: Device pointer to functor parameters, one for all threads
	CB_Functor_Data *fdata,  										// In: Device pointer to functor data, one for all threads
	Cuda_Obstacle *obstacles,  										// In: Device pointer to obstacles, one for all threads
	CPE *cpe, 		 												// In: Device pointer to the collision probability estimator, one for each thread
	Ownship *ownship, 												// In: Device pointer to the ownship class, one for each thread
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory,				// In: Device pointer to the own-ship trajectory, one for each thread
	MPC_Cost<CB_Functor_Pars> *mpc_cost								// In: Device pointer to the cost function keeper class, one for each thread
	) :
	pars(pars), fdata(fdata), obstacles(obstacles), cpe(cpe), ownship(ownship), trajectory(trajectory), mpc_cost(mpc_cost)
{
}

/****************************************************************************************
*  Name     : operator()
*  Function : This is where the fun begins. Evaluates the cost of following the control
*			  behaviour (a particular avoidance maneuver) given by the input tuple.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ thrust::tuple<float, float> CB_Cost_Functor_2::operator()(const thrust::tuple<
	const unsigned int, 								// Thread ID
	TML::PDMatrix<float, 2 * MAX_N_M, 1>, 				// Control behaviour considered
	const unsigned int, 								// Control behaviour index
	const unsigned int, 								// Obstacle i considered
	const unsigned int> &input_tuple 					// Prediction scenario ps for the obstacle
	)
{
	max_cost_i_ps = 0.0f; mu_i_ps = 0.0f;

	//======================================================================================================================
	// 1.0 : Setup. Size temporaries accordingly to input data, etc..
	thread_index = thrust::get<0>(input_tuple);
	offset_sequence = thrust::get<1>(input_tuple);
	cb_index = thrust::get<2>(input_tuple);
	i = thrust::get<3>(input_tuple);
	ps = thrust::get<4>(input_tuple);

	n_samples = round(pars->T / pars->dt);

	d_safe_i = 0.0; chi_m = 0.0; cost_k = 0.0;

	// Seed collision probability estimator using the thread index
	cpe[thread_index].seed_prng(thread_index);

	// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
	n_seg_samples = std::round(cpe[thread_index].get_segment_discretization_time() / pars->dt) + 1;
	
	xs_p_seg.resize(4, n_seg_samples);
	xs_i_p_seg.resize(4, n_seg_samples);
	P_i_p_seg.resize(16, n_seg_samples);

	//======================================================================================================================
	// 2 : Max cost calculation considering own-ship control behaviour <cb_index> and prediction scenario ps for obstacle i
	d_safe_i = pars->d_safe + 0.5 * (fdata->ownship_length + obstacles[i].get_length());

	p_step = 1;
	v_os_prev.set_zero(); v_i_prev.set_zero();
	for (int k = 0; k < n_samples; k += p_step)
	{	
		//==========================================================================================
		// 2.0 : Extract states and information relevant for cost evaluation at sample k. 
		xs_p_seg.shift_columns_left();
		xs_p_seg.set_col(n_seg_samples - 1, trajectory[cb_index].get_col(k));

		P_i_p_seg.shift_columns_left();
		P_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_covariance_sample(k));

		xs_i_p_seg.shift_columns_left();
		xs_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_sample(ps, k));

		if (k == 0)
		{
			cpe[thread_index].initialize(
				xs_p_seg.get_col(n_seg_samples - 1), 
				xs_i_p_seg.get_col(n_seg_samples - 1),  
				d_safe_i);
		}

		// Determine active course modification at sample k
		for (int M = 0; M < pars->n_M; M++)
		{
			if (M < pars->n_M - 1)
			{
				if (k >= fdata->maneuver_times[M] && k < fdata->maneuver_times[M + 1])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
			else
			{
				if (k >= fdata->maneuver_times[M])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
		}

		//==========================================================================================
		// 2.1 : Estimate Collision probability at time k with obstacle i in prediction scenario ps
		/* printf("k = %d | xs_p = %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", k, xs_p_seg(0, n_seg_samples - 1), xs_p_seg(1, n_seg_samples - 1), xs_p_seg(2, n_seg_samples - 1), xs_p_seg(3, n_seg_samples - 1), xs_p_seg(4, n_seg_samples - 1), xs_p_seg(5, n_seg_samples - 1));
		printf("k = %d | xs_i_p = %.1f, %.1f, %.1f, %.1f\n", k, xs_i_p_seg(0, n_seg_samples - 1), xs_i_p_seg(1, n_seg_samples - 1), xs_i_p_seg(2, n_seg_samples - 1), xs_i_p_seg(3, n_seg_samples - 1));
		*/
		/* printf("P_i_p = %.1f, %.1f, %.1f, %.1f\n", P_i_p(0, n_seg_samples - 1), P_i_p(1, n_seg_samples - 1), P_i_p(2, n_seg_samples - 1), P_i_p(3, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(4, n_seg_samples - 1), P_i_p(5, n_seg_samples - 1), P_i_p(6, n_seg_samples - 1), P_i_p(7, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(8, n_seg_samples - 1), P_i_p(9, n_seg_samples - 1), P_i_p(10, n_seg_samples - 1), P_i_p(11, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(12, n_seg_samples - 1), P_i_p(13, n_seg_samples - 1), P_i_p(14, n_seg_samples - 1), P_i_p(15, n_seg_samples - 1)); */
		
		switch(pars->cpe_method)
		{
			case CE :	
				if (k > 0)
				{
					// Map [chi, U]^T to [Vx, Vy]
					v_os_prev(0) = xs_p_seg(3, n_seg_samples - 2) * cos(xs_p_seg(2, n_seg_samples - 2));
					v_os_prev(1) = xs_p_seg(3, n_seg_samples - 2) * sin(xs_p_seg(2, n_seg_samples - 2));
					
					v_i_prev = xs_i_p_seg.get_block<2, 1>(2, n_seg_samples - 2, 2, 1);
				}
				p_os = xs_p_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);
				p_i = xs_i_p_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);

				P_i_2D = reshape<16, 1, 4, 4>(P_i_p_seg.get_col(n_seg_samples - 1), 4, 4).get_block<2, 2>(0, 0, 2, 2);

				P_c_i = cpe[thread_index].CE_estimate(p_os, p_i, P_i_2D, v_os_prev, v_i_prev, pars->dt);
				break;
			case MCSKF4D :                
				if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
				{
					P_c_i = cpe[thread_index].MCSKF4D_estimate(xs_p_seg, xs_i_p_seg, P_i_p_seg);						
				}	
				break;
			default :
				// Throw
				break;
		}

		//==========================================================================================
		// 2.2 : Calculate and maximize dynamic obstacle cost in prediction scenario ps wrt time
		tup = mpc_cost[thread_index].calculate_dynamic_obstacle_cost(
			fdata,
			obstacles, 
			P_c_i, 
			xs_p_seg.get_col(n_seg_samples - 1), 
			xs_i_p_seg.get_col(n_seg_samples - 1), 
			i, 
			chi_m,
			fdata->ownship_length,
			k);

		cost_k = thrust::get<0>(tup);
		mu_k = thrust::get<1>(tup);

		if (max_cost_i_ps < cost_k)
		{
			max_cost_i_ps = cost_k;
		}
		if (mu_i_ps < 0.1f) // Only set the COLREGS violation indicator if it has not been true (> 0.0f) yet
		{
			if (mu_k) 	{ mu_i_ps = 1.0f; }
			else 		{ mu_i_ps = 0.0f; }
		}
		//==========================================================================================
		//printf("i = %d | ps = %d | k = %d | P_c_i = %.6f | cost_ps = %.4f | cb : %.1f, %.1f\n", i, ps, k, P_c_i, cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1));
		
		/* printf("max_cost_ps = ");
		for (int ps = 0; ps < fdata->n_ps[i]; ps++)
		{
			printf("%.4f", max_cost_ps(ps));
			if (ps < fdata->n_ps[i] - 1)
			{
				printf(", ");
			}
		}
		printf("\n"); */
		//==============================================================================================
	}
	
	//==================================================================================================
	//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | mu_i_ps : %.4f | cb : %.1f, %.1f \n", thread_index, i, ps, cb_index, max_cost_i_ps, mu_i_ps, offset_sequence(0), RAD2DEG * offset_sequence(1));
	/* printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f\n", thread_index, i, ps, cb_index, max_cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1), 
		offset_sequence(2), RAD2DEG * offset_sequence(3)); */
	//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", thread_index, i, ps, cb_index, max_cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1), 
	//	offset_sequence(2), RAD2DEG * offset_sequence(3)), offset_sequence(4), RAD2DEG * offset_sequence(5)); 

	//==================================================================================================
	// 2.7 : Return dynamic obstacle related cost and associated colregs violation indicator
	return thrust::tuple<float, float>(max_cost_i_ps, mu_i_ps);
}
 
//=======================================================================================
//	Private functions
//=======================================================================================

}
}