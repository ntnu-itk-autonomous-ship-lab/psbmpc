/****************************************************************************************
*
*  File name : cb_cost_functor.cu
*
*  Function  : Class functions for the control behaviour cost evaluation functor. Used in
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
#include "cb_cost_functor.cuh"
#include "utilities.cuh"

#include <cmath>
#include <iostream>


/****************************************************************************************
*  Name     : CB_Cost_Functor
*  Function : Class constructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ CB_Cost_Functor::CB_Cost_Functor(
	CB_Functor_Pars *pars,  										// In: Device pointer to functor parameters, one for all threads
	CB_Functor_Data *fdata,  										// In: Device pointer to functor data, one for all threads
	Cuda_Obstacle *obstacles,  										// In: Device pointer to obstacles, one for all threads
	CPE *cpe, 		 												// In: Device pointer to the collision probability estimator, one for each thread
	TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory,				// In: Device pointer to the own-ship trajectory, one for each thread
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_i_colav_p,			// In: Device pointer to the intelligent obstacle trajectories, one for each thread
	const int wp_c_0												// In: Waypoint counter for PSB-MPC Ownship object, to initialize the waypoint following in the thread own-ship objects properly
	) :
	pars(pars), fdata(fdata), obstacles(obstacles), cpe(cpe), trajectory(trajectory), xs_i_colav_p(xs_i_colav_p)
{
	ownship.set_wp_counter(wp_c_0);
}

/****************************************************************************************
*  Name     : operator()
*  Function : This is where the fun begins. Evaluates the cost of following the control
*			  behaviour (a particular avoidance maneuver) given by the input tuple.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ float CB_Cost_Functor::operator()(
	const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple	// In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
	)
{
	cost_cb = 0;

	//======================================================================================================================
	// 1.0 : Setup. Size temporaries accordingly to input data, etc..
	cb_index = thrust::get<0>(cb_tuple);
	offset_sequence = thrust::get<1>(cb_tuple);

	n_samples = round(pars->T / pars->dt);

	trajectory[cb_index].resize(6, n_samples);
	trajectory[cb_index].set_col(0, fdata->ownship_state);

	d_safe_i = 0.0; chi_m = 0.0; Pr_CC_i = 0.0; cost_ps = 0.0;
	
	cost_i.resize(fdata->n_obst, 1); cost_i.set_zero();

	// Seed collision probability estimator using the cb index
	cpe[cb_index].seed_prng(cb_index);

	// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
	n_seg_samples = std::round(cpe[cb_index].get_segment_discretization_time() / pars->dt) + 1;
	
	xs_p.resize(6, n_seg_samples);
	xs_i_p.resize(4, n_seg_samples);
	P_i_p.resize(16, n_seg_samples);

	//======================================================================================================================
	// 1.1: Predict own-ship trajectory with the current control behaviour
	ownship.predict_trajectory(
		trajectory[cb_index], 
		offset_sequence, 
		fdata->maneuver_times, 
		fdata->u_d, fdata->chi_d, 
		fdata->waypoints, 
		pars->prediction_method, 
		pars->guidance_method, 
		pars->T, pars->dt);

	// 1.2: Joint prediction with the current control behaviour

	//======================================================================================================================
	// 2 : Cost calculation
	// Not entirely optimal for loop configuration, but the alternative requires alot of memory
	for (int i = 0; i < fdata->n_obst; i++)
	{	
		d_safe_i = pars->d_safe + 0.5 * (fdata->ownship.get_length() + obstacles[i].get_length());

		P_c_i.resize(fdata->n_ps[i], 1); P_c_i.set_zero();
		max_cost_ps.resize(fdata->n_ps[i], 1); max_cost_ps.set_zero();
		weights_ps.resize(fdata->n_ps[i], 1); weights_ps.set_zero();

		mu_i = obstacles[i].get_COLREGS_violation_indicator();

		cost_a.set_zero();
		Pr_a = obstacles[i].get_intention_probabilities();

		Pr_CC_i = obstacles[i].get_a_priori_CC_probability();

		for (int ps = 0; ps < fdata->n_ps[i]; ps++)
		{	
			cost_ps = 0;
			v_os_prev.set_zero(); v_i_prev.set_zero();
			for (int k = 0; k < n_samples; k++)
			{	
				//==========================================================================================
				// 2.0 : Extract states and information relevant for cost evaluation at sample k. 

				xs_p.shift_columns_left();
				xs_p.set_col(n_seg_samples - 1, trajectory[cb_index].get_col(k));

				P_i_p.shift_columns_left();
				P_i_p.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_covariance_sample(k));

				xs_i_p.shift_columns_left();
				xs_i_p.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_sample(ps, k));

				if (k == 0)
				{
					cpe[cb_index].initialize(xs_p.get_col(n_seg_samples - 1), xs_i_p.get_col(n_seg_samples - 1), P_i_p.get_col(n_seg_samples - 1), d_safe_i);
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
				

				/* printf("xs_p = %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", xs_p(0, 0), xs_p(1, 0), xs_p(2, 0), xs_p(3, 0), xs_p(4, 0), xs_p(5, 0));
				printf("xs_i_p = %.1f, %.1f, %.1f, %.1f\n", xs_i_p(0, 0), xs_i_p(1, 0), xs_i_p(2, 0), xs_i_p(3, 0)); */

				/* printf("P_i_p = %.1f, %.1f, %.1f, %.1f\n", P_i_p(0, 0), P_i_p(1, 0), P_i_p(2, 0), P_i_p(3, 0));
				printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(4, 0), P_i_p(5, 0), P_i_p(6, 0), P_i_p(7, 0));
				printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(8, 0), P_i_p(9, 0), P_i_p(10, 0), P_i_p(11, 0));
				printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(12, 0), P_i_p(13, 0), P_i_p(14, 0), P_i_p(15, 0)); */
 				
				if (true)//fmod(k, 2) == 0)
				{
					switch(pars->cpe_method)
					{
						case CE :	
							if (k > 0)
							{
								v_os_prev = xs_p.get_block<2, 1>(3, n_seg_samples - 2, 2, 1);
								v_os_prev = rotate_vector_2D(v_os_prev, xs_p(2, n_seg_samples - 2));
								v_i_prev = xs_i_p.get_block<2, 1>(2, n_seg_samples - 2, 2, 1);
							}
							p_os = xs_p.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);
							p_i = xs_i_p.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);

							P_i_2D = reshape<16, 1, 4, 4>(P_i_p.get_col(n_seg_samples - 1), 4, 4).get_block<2, 2>(0, 0, 2, 2);

							P_c_i(ps) = cpe[cb_index].CE_estimate(p_os, p_i, P_i_2D, v_os_prev, v_i_prev, pars->dt);
							break;
						case MCSKF4D :                
							if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
							{
								P_c_i(ps) = cpe[cb_index].MCSKF4D_estimate(xs_p, xs_i_p, P_i_p);						
							}	
							break;
						default :
							// Throw
							break;
					}
				}
				

				//==========================================================================================
				// 2.2 : Calculate and maximize dynamic obstacle cost in prediction scenario ps wrt time
				cost_ps = calculate_dynamic_obstacle_cost(P_c_i(ps), xs_p.get_col(n_seg_samples - 1), xs_i_p.get_col(n_seg_samples - 1), i, chi_m, cb_index);

				if (max_cost_ps(ps) < cost_ps)
				{
					max_cost_ps(ps) = cost_ps;
				}
				//==========================================================================================
				//printf("i = %d | ps = %d | k = %d | P_c_i = %.6f | cost_ps = %.4f | cb : %.1f, %.1f\n", i, ps, k, P_c_i(ps), cost_ps, offset_sequence(0), RAD2DEG * offset_sequence(1));
			}
		}
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
		
		//printf("Pr_CC = %.4f\n", Pr_CC_i);
		//==============================================================================================
		// 2.3 : Calculate a weighted obstacle cost over all prediction scenarios

		// If only 1 prediction scenario: Original PSB-MPC formulation
		if (fdata->n_ps[i] == 1)
		{
			cost_i(i) = max_cost_ps(0);
		}
		else // Three intentions to consider: KCC, SM and PM
		{
			// Weight prediction scenario cost based on if obstacle follows COLREGS or not,
			// which means that higher cost is applied if the obstacle follows COLREGS
			// to a high degree (high Pr_CC_i with no COLREGS violation from its side)
			// and the own-ship breaches COLREGS
			sum_sm_weights = 0.0f; sum_pm_weights = 0.0f;
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				weights_ps(ps) = Pr_CC_i;
				if (mu_i[ps])
				{
					//printf("Obstacle i = %d breaks COLREGS in ps = %d\n", i, ps);
					weights_ps(ps) = 1 - Pr_CC_i;
				}
				
				if (ps > 0)
				{
					// Starboard maneuvers
					if (ps < (fdata->n_ps[i] - 1) / 2 + 1)
					{
						sum_sm_weights += weights_ps(ps);
					}
					// Port maneuvers
					else
					{
						sum_pm_weights += weights_ps(ps);
					}
				}
			}

			// Normalize and assign cost associated with starboard and port intentions
			for(int ps = 1; ps < fdata->n_ps[i]; ps++)
			{
				// Starboard maneuvers
				if (ps < (fdata->n_ps[i] - 1) / 2 + 1)
				{
					if (sum_sm_weights > 0.0)
					{
						cost_a(1) += (weights_ps(ps) / sum_sm_weights) * max_cost_ps(ps);
					}
					else
					{
						cost_a(1) += weights_ps(ps) * max_cost_ps(ps);
					}
				}
				// Port maneuvers
				else
				{
					if (sum_pm_weights > 0.0)
					{
						cost_a(2) += (weights_ps(ps) / sum_pm_weights) * max_cost_ps(ps);
					}
					else
					{
						cost_a(2) += weights_ps(ps) * max_cost_ps(ps);
					}
				}
			}
			// Average the cost for the corresponding intention
			cost_a(0) = weights_ps(0) * max_cost_ps(0); 
			cost_a(1) /= (((float)fdata->n_ps[i] - 1.0f) / 2.0f);
			cost_a(2) /= (((float)fdata->n_ps[i] - 1.0f) / 2.0f);

			// Weight by the intention probabilities
			cost_i(i) = Pr_a.dot(cost_a);

			//printf("Pr_a = %.4f, %.4f, %.4f\n", Pr_a(0), Pr_a(1), Pr_a(2));
			/* printf("sum_sm_weights = %.4f\n", sum_sm_weights);
			printf("sum_pm_weights = %.4f\n", sum_pm_weights);
			printf("weights_ps normalized = %.4f, ", weights_ps(0));
			for (int ps = 1; ps < fdata->n_ps[i]; ps++)
			{
				if (ps < (fdata->n_ps[i] - 1) / 2 + 1)
				{ 
					printf("%.4f", weights_ps(ps) / sum_sm_weights);
				}
				else
				{
					printf("%.4f", weights_ps(ps) / sum_pm_weights);
				}
				printf(", ");
			}
			printf("%.1f, %.1f\n", offset_sequence(0), RAD2DEG * offset_sequence(1));
			printf("cost_a = %.4f, %.4f, %.4f | cb : %.1f, %.1f\n", cost_a(0), cost_a(1), cost_a(2), offset_sequence(0), RAD2DEG * offset_sequence(1));
			printf("cost_i(i) = %.6f | cb : %.1f, %.1f\n", cost_i(i), offset_sequence(0), RAD2DEG * offset_sequence(1));  */
		}
		//==============================================================================================
	}
	
	//==================================================================================================
	// 2.4 : Extract maximum cost wrt all obstacles
	cost_cb += cost_i.max_coeff(); 

	//==================================================================================================
	// 2.5 : Calculate cost due to driving the boat on land or static objects
	//cost += calculate_grounding_cost(); 

	//==================================================================================================
	// 2.6 : Calculate cost due to deviating from the nominal path
	cost_cb += calculate_control_deviation_cost(offset_sequence, cb_index);
	//printf("dev cost = %.4f\n", calculate_control_deviation_cost(offset_sequence, cb_index));

	//==================================================================================================
	// 2.7 : Calculate cost due to having a wobbly offset_sequence
	cost_cb += calculate_chattering_cost(offset_sequence, cb_index); 
	//printf("chat cost = %.4f\n", calculate_chattering_cost(offset_sequence, cb_index));
	//==================================================================================================
	//printf("Cost of cb_index %d : %.4f | cb : %.1f, %.1f\n", cb_index, cost_cb, offset_sequence(0), RAD2DEG * offset_sequence(1));
	return cost_cb;
}
 
//=======================================================================================
//	Private functions
//=======================================================================================

//=======================================================================================
//  Name     : predict_trajectories_jointly
//  Function : Predicts the trajectory of the ownship and obstacles with an active COLAV
//			  system
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ void CB_Cost_Functor::predict_trajectories_jointly()
{

}

//=======================================================================================
//  Name     : determine_COLREGS_violation
//  Function : Determine if vessel A violates COLREGS with respect to vessel B.
//			  
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_COLREGS_violation(
	const TML::Vector2f &v_A,												// In: (NE) Velocity vector of vessel A, row vector
	const float psi_A, 													// In: Heading of vessel A
	const TML::Vector2f &v_B, 												// In: (NE) Velocity vector of vessel B, row vector
	const TML::Vector2f &L_AB, 												// In: LOS vector pointing from vessel A to vessel B, row vector
	const float d_AB 														// In: Distance from vessel A to vessel B
	)
{
	is_ahead = v_A.dot(L_AB) > cos(pars->phi_AH) * v_A.norm();

	is_close = d_AB <= pars->d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(pars->phi_OT) * v_A.norm() * v_B.norm() 	&&
					 v_A.norm() < v_B.norm()							  		&&
					 v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(pars->phi_OT) * v_B.norm() * v_A.norm() 	&&
					 v_B.norm() < v_A.norm()							  		&&
					 v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > pars->d_safe;

	is_head_on = v_A.dot(v_B) < - cos(pars->phi_HO) * v_A.norm() * v_B.norm() 	&&
				 v_A.norm() > 0.25												&&
				 v_B.norm() > 0.25												&&
				 is_ahead;

	is_crossing = v_A.dot(v_B) < cos(pars->phi_CR) * v_A.norm() * v_B.norm()  	&&
				  v_A.norm() > 0.25												&&
				  v_B.norm() > 0.25												&&
				  !is_head_on 													&&
				  !is_passed;

	return is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken));
}

//=======================================================================================
//  Name     : determine_transitional_cost_indicator
//  Function : Determine if a transitional cost should be applied for the current
//			  control behavior, using the method in Hagen, 2018. Two overloads
// Author   : 
//  Modified :
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_transitional_cost_indicator(
	const float psi_A, 													// In: Heading of vessel A
	const float psi_B, 													// In: Heading of vessel B
	const TML::Vector2f &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const int i, 														// In: Index of obstacle
	const float chi_m, 													// In: Candidate course offset currently followed
	const unsigned int cb_index											// In: Index of control behaviour currently followed in this thread
	)
{
	// Obstacle on starboard side
	S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	// Ownship on starboard side of obstacle
	S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!fdata->S_TC_0[i]) { O_TC = fdata->O_TC_0[i] && S_TC; }
	else { O_TC = fdata->O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!fdata->S_i_TC_0[i]) { Q_TC = fdata->Q_TC_0[i] && S_i_TC; }
	else { Q_TC = fdata->Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = fdata->X_TC_0[i] && fdata->S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!fdata->S_TC_0[i]) { H_TC = fdata->H_TC_0[i] && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}

//=======================================================================================
//  Name     : calculate_collision_probabilities
//  Function : Estimates collision probabilities for the own-ship and an obstacle i in
//			   consideration. Used only with together with a specific overload of the 
//			   operator() function (unused at this moment).
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ inline void CB_Cost_Functor::calculate_collision_probabilities(
	TML::PDMatrix<float, MAX_N_PS, MAX_N_SAMPLES> &P_c_i,		// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const int i, 															// In: Index of obstacle
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	n_samples = round(pars->T / pars->dt);
	
	float d_safe_i = pars->d_safe + 0.5 * (fdata->ownship.get_length() + obstacles[i].get_length());

	TML::PDMatrix<float, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_i_p = obstacles[i].get_trajectories();
	TML::PDMatrix<float, 16, MAX_N_SAMPLES> P_i_p = obstacles[i].get_trajectory_covariance();

	// Non-optimal temporary row-vector storage solution
	TML::PDMatrix<float, 1, MAX_N_SAMPLES> P_c_i_row(1, n_samples);
	for (int ps = 0; ps < fdata->n_ps[i]; ps++)
	{
		//cpe[cb_index].estimate_over_trajectories(P_c_i_row, fdata->trajectory, xs_i_p.get_block<4, MAX_N_SAMPLES>(4 * ps, 0, 4, n_samples), P_i_p, d_safe_i, i, pars->dt);

		P_c_i.set_block(ps, 0, 1, P_c_i_row.get_cols(), P_c_i_row);
	}		
}

//=======================================================================================
//  Name     : calculate_dynamic_obstacle_cost
//  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i considering 
//			   a certain time instant for the ownship and obstacle i in pred. scen. ps.
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ inline float CB_Cost_Functor::calculate_dynamic_obstacle_cost(
	const float P_c_i,															// In: Predicted obstacle collision probabilities for obstacle in prediction scenario ps
	const TML::PDVector6f xs_p, 												// In: Predicted own-ship state at time step k
	const TML::PDVector4f xs_i_p, 												// In: Predicted obstacle state at time step k in prediction scenario ps
	const int i, 																// In: Index of obstacle
	const float chi_m,														 	// In: Course offset used by the own-ship at time step k
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	cost_do = 0.0;
	
	// l_i is the collision cost modifier depending on the obstacle track loss.
	C = 0.0; l_i = 0.0;

	psi_0_p = xs_p(2); 
	v_0_p(0) = xs_p(3); 
	v_0_p(1) = xs_p(4); 
	v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

	L_0i_p = xs_i_p.get_block<2, 1>(0, 0, 2, 1) - xs_p.get_block<2, 1>(0, 0, 2, 1);
	d_0i_p = L_0i_p.norm();

	// Decrease the distance between the vessels by their respective max dimension
	d_0i_p = d_0i_p - 0.5 * (fdata->ownship.get_length() + obstacles[i].get_length()); 
	
	L_0i_p.normalize();

	v_i_p(0) = xs_i_p(2);
	v_i_p(1) = xs_i_p(3);
	psi_i_p = atan2(v_i_p(1), v_i_p(0));

	C = calculate_collision_cost(v_0_p, v_i_p);

	mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

	trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, i, chi_m, cb_index);

	// Track loss modifier to collision cost
	if (obstacles[i].get_duration_lost() > pars->p_step)
	{
		l_i = pars->dt * pars->p_step / obstacles[i].get_duration_lost();
	} 
	else
	{
		l_i = 1;
	}

	cost_do = l_i * C * P_c_i + pars->kappa * mu  + pars->kappa_TC * trans;

	/* printf("psi_0_p = %.2f | v_0_p = %.2f, %.2f\n", psi_0_p, v_0_p(0), v_0_p(1));
	printf("psi_i_p = %.2f | v_i_p = %.2f, %.2f\n", psi_i_p, v_i_p(0), v_i_p(1));
	printf("d_0i_p = %.2f  | L_0i_p = %.2f, %.2f\n", d_0i_p, L_0i_p(0), L_0i_p(1));
	printf("C = %.4f       | mu = %d                 | trans = %d           | l_i = %.4f\n", C, mu, trans, l_i);
	printf("cost_ps = %.4f\n", cost_do); */
	return cost_do;
}

//=======================================================================================
//  Name     : calculate_ad_hoc_collision_risk
//  Function : 
//  Author   : 
//  Modified :
//=======================================================================================
__device__ float CB_Cost_Functor::calculate_ad_hoc_collision_risk(
	const float d_AB, 												// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
																	// 	   reduced by half the length of the two vessels
	const float t 													// In: Prediction time t > t0 (= 0)
	)
{
	ahcr = 0.0;
	if (d_AB <= pars->d_safe)
	{
		assert(t > 0);
		ahcr = pow(pars->d_safe / d_AB, pars->q) * (1 / pow(fabs(t), pars->p)); 
	}
	return ahcr;
}

//=======================================================================================
//  Name     : calculate_control_deviation_cost
//  Function : Determines penalty due to using offsets to guidance references ++
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ float CB_Cost_Functor::calculate_control_deviation_cost(
	const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, 				// In: Control behaviour currently followed
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	cost_cd = 0;
	for (int i = 0; i < pars->n_M; i++)
	{
		if (i == 0)
		{
			cost_cd += pars->K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], fdata->u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], fdata->chi_m_last);
		}
		else
		{
			cost_cd += pars->K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	
	/* printf("K_u (1 - u_m_0) = %.4f | Delta_u(u_m_0, u_m_last) = %.4f | K_chi(chi_0) = %.4f | Delta_chi(chi_0, chi_last) = %.4f\n", 
		pars->K_u * (1 - offset_sequence[0]), Delta_u(offset_sequence[0], fdata->u_m_last), K_chi(offset_sequence[1]), Delta_chi(offset_sequence[1], fdata->chi_m_last)); */
	return cost_cd / (float)pars->n_M;
}

//=======================================================================================
//  Name     : calculate_chattering_cost
//  Function : Determines penalty due to using wobbly (changing between positive and negative)
// 			  course modifications
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ float CB_Cost_Functor::calculate_chattering_cost(
	const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, 			// In: Control behaviour currently followed
	const unsigned int cb_index												// In: Index of control behaviour currently followed in this thread
)
{
	cost_ch = 0;
	if (pars->n_M > 1) 
	{
		delta_t = 0;
		for(int M = 0; M < pars->n_M - 1; M++)
		{
			if ((offset_sequence(2 * M + 1) > 0 && offset_sequence(2 * M + 3) < 0) ||
				(offset_sequence(2 * M + 1) < 0 && offset_sequence(2 * M + 3) > 0))
			{
				delta_t = fdata->maneuver_times(M + 1) - fdata->maneuver_times(M);
				cost_ch += pars->K_sgn * exp( - delta_t / pars->T_sgn);
			}
		}
	}
	return cost_ch;
}

//=======================================================================================
//  Name     : calculate_grounding_cost
//  Function : Determines penalty due grounding ownship on static obstacles (no-go zones)
//  Author   : Trym Tengesdal & Giorgio D. Kwame Minde Kufoalor
//  Modified :
//=======================================================================================
__device__ float CB_Cost_Functor::calculate_grounding_cost()
{
	cost_g = 0;

	return cost_g;
}

//=======================================================================================
//  Name     : find_triplet_orientation
//  Function : Find orientation of ordered triplet (p, q, r)
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal for more readability
//=======================================================================================
__device__ int CB_Cost_Functor::find_triplet_orientation(
	const TML::Vector2f &p, 
	const TML::Vector2f &q, 
	const TML::Vector2f &r
	)
{
	// Calculate z-component of cross product (q - p) x (r - q)
    int val = (q[0] - p[0]) * (r[1] - q[1]) - (q[1] - p[1]) * (r[0] - q[0]);

    if (val == 0) return 0; // colinear
    return val < 0 ? 1 : 2; // clock or counterclockwise
}

//=======================================================================================
//  Name     : determine_if_on_segment
//  Function : Determine if the point q is on the segment pr
//			  (really if q is inside the rectangle with diagonal pr...)
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_if_on_segment(
	const TML::Vector2f &p, 
	const TML::Vector2f &q, 
	const TML::Vector2f &r
	)
{
    if (q[0] <= fmaxf(p[0], r[0]) && q[0] >= fminf(p[0], r[0]) &&
        q[1] <= fmaxf(p[1], r[1]) && q[1] >= fminf(p[1], r[1]))
        return true;
    return false;
}

//=======================================================================================
//  Name     : determine_if_behind
//  Function : Check if the point p_1 is behind the line defined by v_1 and v_2
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_if_behind(
	const TML::Vector2f &p_1, 
	const TML::Vector2f &v_1, 
	const TML::Vector2f &v_2, 
	const float distance_to_line
	)
{
    TML::Vector2f v_diff, n;
    
    v_diff = v_2 - v_1;

    n(0) = -v_diff(1); n(1) = v_diff(0);
    n = n / n.norm() * distance_to_line;

    return (determine_if_on_segment(v_1 + n, p_1, v_2 + n));
}

//=======================================================================================
//  Name     : determine_if_lines_intersect
//  Function : Determine if the line segments defined by p_1, q_1 and p_2, q_2 intersects 
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_if_lines_intersect(
	const TML::Vector2f &p_1, 
	const TML::Vector2f &q_1, 
	const TML::Vector2f &p_2, 
	const TML::Vector2f &q_2
	)
{
    // Find the four orientations needed for general and
    // special cases
    int o_1 = find_triplet_orientation(p_1, q_1, p_2);
    int o_2 = find_triplet_orientation(p_1, q_1, q_2);
    int o_3 = find_triplet_orientation(p_2, q_2, p_1);
    int o_4 = find_triplet_orientation(p_2, q_2, q_1);

    // General case
    if (o_1 != o_2 && o_3 != o_4)
        return true;

    // Special Cases
    // p_1, q_1 and p_2 are colinear and p_2 lies on segment p_1q_1
    if (o_1 == 0 && determine_if_on_segment(p_1, p_2, q_1)) return true;

    // p_1, q_1 and q_2 are colinear and q_2 lies on segment p_1q_1
    if (o_2 == 0 && determine_if_on_segment(p_1, q_2, q_1)) return true;

    // p_2, q_2 and p_1 are colinear and p_1 lies on segment p_2q_2
    if (o_3 == 0 && determine_if_on_segment(p_2, p_1, q_2)) return true;

    // p_2, q_2 and q_1 are colinear and q_1 lies on segment p2q2
    if (o_4 == 0 && determine_if_on_segment(p_2, q_1, q_2)) return true;

    return false; // Doesn't fall in any of the above cases
}

//=======================================================================================
//  Name     : distance_from_point_to_line
//  Function : Calculate distance from p to the line segment defined by q_1 and q_2
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ float CB_Cost_Functor::distance_from_point_to_line(
	const TML::Vector2f &p, 
	const TML::Vector2f &q_1, 
	const TML::Vector2f &q_2
	)
{   
	TML::Vector3f a;
    TML::Vector3f b;
    a.set_block<2, 1>(0, 0, (q_1 - q_2)); 	a(2) = 0;
    b.set_block<2, 1>(0, 0, (p - q_2)); 	b(2) = 0;

    TML::Vector3f c = a.cross(b);
    if (a.norm() > 0) return c.norm() / a.norm();
    else return -1;
}

//=======================================================================================
//  Name     : distance_to_static_obstacle
//  Function : Calculate distance from p to obstacle defined by line segment {v_1, v_2}
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ float CB_Cost_Functor::distance_to_static_obstacle(
	const TML::Vector2f &p, 
	const TML::Vector2f &v_1, 
	const TML::Vector2f &v_2
	)
{
    float d2line = distance_from_point_to_line(p, v_1, v_2);

    if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
    else return fminf((v_1-p).norm(),(v_2-p).norm());
} 