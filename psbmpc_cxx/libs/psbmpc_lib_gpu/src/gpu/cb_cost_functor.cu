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

#include "psbmpc_defines.h"
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
__device__ float CB_Cost_Functor_1::operator()(
	const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple	// In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
	)
{
	cost_cb = 0.0f;
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
	//cost_cb += mpc_cost[cb_index].calculate_grounding_cost(); 

	//==================================================================================================
	// 2.2 : Calculate cost due to deviating from the nominal path
	cost_cb += mpc_cost[cb_index].calculate_control_deviation_cost(offset_sequence, fdata->u_opt_last, fdata->chi_opt_last);

	//==================================================================================================
	// 2.3 : Calculate cost due to having a wobbly offset_sequence
	cost_cb += mpc_cost[cb_index].calculate_chattering_cost(offset_sequence, fdata->maneuver_times); 

	return cost_cb;
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
	Prediction_Obstacle *pobstacles,  								// In: Device pointer to prediction_obstacles, one for each thread
	CPE *cpe, 		 												// In: Device pointer to the collision probability estimator, one for each thread
	Ownship *ownship, 												// In: Device pointer to the ownship class, one for each thread
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *trajectory,				// In: Device pointer to the own-ship trajectory, one for each thread
	Obstacle_Ship *obstacle_ship,									// In: Device pointer to the obstacle ship for joint prediction, one for each thread
	Obstacle_SBMPC *obstacle_sbmpc,									// In: Device pointer to the obstacle sbmpc for joint prediction, one for each thread
	MPC_Cost<CB_Functor_Pars> *mpc_cost								// In: Device pointer to the cost function keeper class, one for each thread
	) :
	pars(pars), fdata(fdata), 
	obstacles(obstacles), 
	pobstacles(pobstacles), 
	cpe(cpe), ownship(ownship), trajectory(trajectory), 
	obstacle_ship(obstacle_ship), obstacle_sbmpc(obstacle_sbmpc),
	mpc_cost(mpc_cost)
{

}

/****************************************************************************************
*  Name     : operator()
*  Function : This is where the fun begins. Evaluates the cost of following the control
*			  behaviour (a particular avoidance maneuver) given by the input tuple.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ thrust::tuple<float, Intention, bool> CB_Cost_Functor_2::operator()(const thrust::tuple<
	const unsigned int, 
	TML::PDMatrix<float, 2 * MAX_N_M, 1>, 
	const unsigned int, 
	const unsigned int, 
	const unsigned int, 
	const int> &input_tuple	
	// In: Tuple consisting of the thread id, ownship control behaviour, the index of the obstacle and its corresponding 
	// prediction scenario index to evaluate the cost with
	)
{
	max_cost_ps = 0.0f;
	a_i_ps_jp = KCC;
	mu_i_ps_jp = false;

	//======================================================================================================================
	// 1.0 : Setup. Size temporaries accordingly to input data, etc..
	thread_index = thrust::get<0>(input_tuple);
	offset_sequence = thrust::get<1>(input_tuple);
	cb_index = thrust::get<2>(input_tuple);
	i = thrust::get<3>(input_tuple);
	ps = thrust::get<4>(input_tuple);

	n_samples = round(pars->T / pars->dt);

	d_safe_i = 0.0; chi_m = 0.0; cost_ps = 0.0;

	// Seed collision probability estimator using the thread index
	cpe[thread_index].seed_prng(thread_index);

	// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
	n_seg_samples = std::round(cpe[thread_index].get_segment_discretization_time() / pars->dt) + 1;
	
	xs_p_seg.resize(4, n_seg_samples);
	xs_i_p_seg.resize(4, n_seg_samples);
	P_i_p_seg.resize(16, n_seg_samples);

	//======================================================================================================================
	// 1.1: Joint prediction with the current control behaviour if the obstacle prediction scenario is the intelligent one
	if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)
	{
		printf("here jp1\n");
		predict_trajectories_jointly();
		a_i_ps_jp = pobstacles[i].get_intention();
		mu_i_ps_jp = pobstacles[i].get_COLREGS_breach_indicator();
	}

	//======================================================================================================================
	// 2 : Max cost calculation considering own-ship control behaviour <cb_index> and prediction scenario ps for obstacle i
	d_safe_i = pars->d_safe + 0.5 * (fdata->ownship_length + obstacles[i].get_length());

	v_os_prev.set_zero(); v_i_prev.set_zero();
	for (int k = 0; k < n_samples; k++)
	{	
		//==========================================================================================
		// 2.0 : Extract states and information relevant for cost evaluation at sample k. 
		xs_p_seg.shift_columns_left();
		xs_p_seg.set_col(n_seg_samples - 1, trajectory[cb_index].get_col(k));

		P_i_p_seg.shift_columns_left();
		P_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_covariance_sample(k));

		xs_i_p_seg.shift_columns_left();
		if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)
		{
			printf("here jp2\n");
			xs_i_p_seg.set_col(n_seg_samples - 1, pobstacles[i].get_trajectory_sample(k));
		}
		else
		{
			xs_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_sample(ps, k));
		}

		if (k == 0)
		{
			cpe[thread_index].initialize(
				xs_p_seg.get_col(n_seg_samples - 1), 
				xs_i_p_seg.get_col(n_seg_samples - 1), 
				P_i_p_seg.get_col(n_seg_samples - 1), 
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
		if (true)//fmod(k, 2) == 0)
		{
			switch(pars->cpe_method)
			{
				case CE :	
					if (k > 0)
					{
						v_os_prev = xs_p_seg.get_block<2, 1>(3, n_seg_samples - 2, 2, 1);
						v_os_prev = rotate_vector_2D(v_os_prev, xs_p_seg(2, n_seg_samples - 2));
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
		}

		//==========================================================================================
		// 2.2 : Calculate and maximize dynamic obstacle cost in prediction scenario ps wrt time
		cost_ps = mpc_cost[thread_index].calculate_dynamic_obstacle_cost(
			fdata,
			obstacles, 
			P_c_i, 
			xs_p_seg.get_col(n_seg_samples - 1), 
			xs_i_p_seg.get_col(n_seg_samples - 1), 
			i, 
			chi_m,
			fdata->ownship_length);

		if (max_cost_ps < cost_ps)
		{
			max_cost_ps = cost_ps;
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
	//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f \n", thread_index, i, ps, cb_index, max_cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1));
	/* printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f\n", thread_index, i, ps, cb_index, max_cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1), 
		offset_sequence(2), RAD2DEG * offset_sequence(3)); */
	//printf("Thread %d | i = %d | ps = %d | Cost cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", thread_index, i, ps, cb_index, max_cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1), 
	//	offset_sequence(2), RAD2DEG * offset_sequence(3)), offset_sequence(4), RAD2DEG * offset_sequence(5)); 

	//==================================================================================================
	// 2.7 : Put dynamic obstacle related cost and static + path related cost into output tuple
	//thrust::tuple<float, float> out(thrust::make_tuple(max_cost_ps, cost_cb_ch_g));

	return thrust::tuple<float, Intention, bool>(thrust::make_tuple(max_cost_ps, a_i_ps_jp, mu_i_ps_jp));
}
 
//=======================================================================================
//	Private functions
//=======================================================================================
//=======================================================================================
//  Name     : update_conditional_obstacle_data
//  Function : Updates the situation type for the calling obstacle (wrt all other 
//			   obstacles) and obstacles (wrt own-ship) and the transitional cost 
//			   indicators O, Q, X, S at the current time t0 wrt all obstacles.
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ void CB_Cost_Functor_2::update_conditional_obstacle_data(
	const int i_caller, 															// In: Index of obstacle asking for a situational awareness update
	const int k																		// In: Index of the current predicted time t_k
	)
{
	// A : Obstacle i_caller, B : Obstacle i
	p_A(0) = pobstacles[i_caller].get_trajectory_sample(k)(0);
	p_A(1) = pobstacles[i_caller].get_trajectory_sample(k)(1);
	v_A(0) = pobstacles[i_caller].get_trajectory_sample(k)(2);
	v_A(1) = pobstacles[i_caller].get_trajectory_sample(k)(3);
	psi_A = atan2(v_A(1), v_A(0));

	data.ST_0.resize(fdata->n_obst, 1);   data.ST_i_0.resize(fdata->n_obst, 1);
	
	data.AH_0.resize(fdata->n_obst, 1);   data.S_TC_0.resize(fdata->n_obst, 1); data.S_i_TC_0.resize(fdata->n_obst, 1); 
	data.O_TC_0.resize(fdata->n_obst, 1); data.Q_TC_0.resize(fdata->n_obst, 1); data.IP_0.resize(fdata->n_obst, 1); 
	data.H_TC_0.resize(fdata->n_obst, 1); data.X_TC_0.resize(fdata->n_obst, 1);
	// printf("p_A = %.2f, %.2f | v_A = %.2f, %.2f\n", p_A(0), p_A(1), v_A(0), v_A(1)); 
	i_count = 0;
	for (int i = 0; i < fdata->n_obst + 1; i++)
	{
		if (i != i_caller)
		{
			p_B(0) = pobstacles[i].get_trajectory_sample(k)(0);
			p_B(1) = pobstacles[i].get_trajectory_sample(k)(1);
			v_B(0) = pobstacles[i].get_trajectory_sample(k)(2);
			v_B(1) = pobstacles[i].get_trajectory_sample(k)(3);
			psi_B = atan2(v_B(1), v_B(0));

			L_AB = p_B - p_A;
			d_AB = L_AB.norm();

			// printf("p_B = %.2f, %.2f | v_B = %.2f, %.2f\n", p_B(0), p_B(1), v_B(0), v_B(1)); 
			// Decrease the distance between the vessels by their respective max dimension
			d_AB = d_AB - 0.5 * (pobstacles[i_caller].get_length() + pobstacles[i].get_length()); 				
			L_AB = L_AB.normalized();

			mpc_cost[thread_index].determine_situation_type(data.ST_0[i_count], data.ST_i_0[i_count], v_A, psi_A, v_B, L_AB, d_AB);
			
			//=====================================================================
			// Transitional variable update
			//=====================================================================
			data.AH_0[i_count] = v_A.dot(L_AB) > cos(pars->phi_AH) * v_A.norm();
			
			// Obstacle on starboard side
			data.S_TC_0[i_count] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			// Ownship on starboard side of obstacle
			data.S_i_TC_0[i_count] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			// Ownship overtaking the obstacle
			data.O_TC_0[i_count] = v_B.dot(v_A) > cos(pars->phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							    						&&
					v_B.norm() > 0.25															&&
					d_AB <= pars->d_close 														&&
					data.AH_0[i_count];

			// Obstacle overtaking the ownship
			data.Q_TC_0[i_count] = v_A.dot(v_B) > cos(pars->phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  							&&
					v_A.norm() > 0.25 															&&
					d_AB <= pars->d_close														&&
					!data.AH_0[i_count];

			// Determine if the obstacle is passed by
			data.IP_0[i_count] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
					!data.Q_TC_0[i_count])		 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 					&& // Obstacle's perspective	
					!data.O_TC_0[i_count]))		 											&&
					d_AB > pars->d_safe;

			// This is not mentioned in article, but also implemented here..				
			data.H_TC_0[i_count] = v_A.dot(v_B) < - cos(pars->phi_HO) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() > 0.25																&&
					v_B.norm() > 0.25																&&
					data.AH_0[i_count];

			// Crossing situation, a bit redundant with the !is_passed condition also, 
			// but better safe than sorry (could be replaced with B_is_ahead also)
			data.X_TC_0[i_count] = v_A.dot(v_B) < cos(pars->phi_CR) * v_A.norm() * v_B.norm()		&&
					!data.H_TC_0[i_count]															&& 
					!data.IP_0[i_count]																&&
					v_A.norm() > 0.25																&&
					v_B.norm() > 0.25;

			i_count += 1;
		}
	}	
}

//=======================================================================================
//  Name     : predict_trajectories_jointly
//  Function : Predicts the trajectory of the ownship and obstacles with an active COLAV
//			  system
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ void CB_Cost_Functor_2::predict_trajectories_jointly()
{
	u_d_i.resize(fdata->n_obst, 1); u_opt_last_i.resize(fdata->n_obst, 1);
	chi_d_i.resize(fdata->n_obst, 1); chi_opt_last_i.resize(fdata->n_obst, 1);
	for(int k = 0; k < n_samples; k++)
	{
		t = k * pars->dt;
		for (int i = 0; i < fdata->n_obst; i++)
		{
			xs_i_p = pobstacles[jp_thread_index + i].get_trajectory_sample(k);

			if (k == 0)
			{
				u_d_i(i) = xs_i_p.get_block<2, 1>(2, 0).norm();
				chi_d_i(i) = atan2(xs_i_p(3), xs_i_p(2));
				u_opt_last_i(i) = 1.0f; chi_opt_last_i(i) = 0.0f; 

				pobstacles[i].set_intention(KCC);
			}

			// Update obstacle data for obstacle i using all other obstacles
			update_conditional_obstacle_data(i, k);

			// Convert from X_i = [x, y, Vx, Vy] to X_i = [x, y, chi, U]
			xs_i_p_transformed.set_block<2, 1>(0, 0, xs_i_p.get_block<2, 1>(0, 0));
			xs_i_p_transformed(2) = atan2(xs_i_p(3), xs_i_p(2));
			xs_i_p_transformed(3) = xs_i_p.get_block<2, 1>(2, 0).norm();

			// Determine the intention that obstacle i`s predicted trajectory
			// corresponds to
			chi_i = xs_i_p_transformed(2);
			if (t < 30)
			{
				if (chi_i > 15 * DEG2RAD)								{ pobstacles[i].set_intention(SM); }
				else if (chi_i < -15 * DEG2RAD)							{ pobstacles[i].set_intention(PM); }
			}
			
			// printf("waypoints_i = %.1f, %.1f |  %.1f, %.1f\n", 
			//	pobstacles[i].get_waypoints()(0, 0), pobstacles[i].get_waypoints()(1, 0), pobstacles[i].get_waypoints()(0, 1), pobstacles[i].get_waypoints()(1, 1));
			obstacle_ship[cb_index].update_guidance_references(
				u_d_i(i), 
				chi_d_i(i), 
				pobstacles[i].get_waypoints(),
				xs_i_p,
				pars->dt,
				pars->guidance_method);

			if (fmod(t, 5) == 0)
			{
				obstacle_sbmpc[cb_index].calculate_optimal_offsets(
					u_opt_i, 
					chi_opt_i, 
					u_opt_last_i(i), 
					chi_opt_last_i(i), 
					u_d_i(i), 
					chi_d_i(i),
					pobstacles[i].get_waypoints(),
					xs_i_p_transformed,
					fdata->static_obstacles,
					data,
					pobstacles,
					i,
					k);

				u_opt_last_i(i) = u_opt_i;
				chi_opt_last_i(i) = chi_opt_i;
				
				//printf("u_opt_i(i) = %.1f | chi_opt_i(i) =  %.2f\n", u_opt_i, chi_opt_i);
			}

			if (k < n_samples - 1)
			{
				xs_i_p_transformed = obstacle_ship[cb_index].predict(
					xs_i_p_transformed, 
					u_d_i(i) * u_opt_i, 
					chi_d_i(i) + chi_opt_i, 
					pars->dt, 
					pars->prediction_method);
				
				// Convert from X_i = [x, y, chi, U] to X_i = [x, y, Vx, Vy]
				xs_i_p.set_block<2, 1>(0, 0, xs_i_p_transformed.get_block<2, 1>(0, 0));
				xs_i_p(2) = xs_i_p_transformed(3) * cos(xs_i_p_transformed(2));
				xs_i_p(3) = xs_i_p_transformed(3) * sin(xs_i_p_transformed(2));

				//printf("k = %d |	xs_i_p = %.1f, %.1f, %.1f, %.1f\n", k, xs_i_p(0), xs_i_p(1), xs_i_p(2), xs_i_p(3));
				pobstacles[i].set_trajectory_sample(xs_i_p, k + 1);
			}
		}
	}
}

}
}