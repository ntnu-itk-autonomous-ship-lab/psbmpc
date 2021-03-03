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
#include "cb_cost_functor_structures.cuh"
#include "cuda_obstacle.cuh"
#include "cpe.cuh"
#include "ownship.cuh"
#include "obstacle_sbmpc.cuh"
#include "obstacle_ship.cuh"


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
	Prediction_Obstacle *pobstacles,  								// In: Device pointer to prediction_obstacles, one for each thread
	CPE_GPU *cpe, 		 											// In: Device pointer to the collision probability estimator, one for each thread
	Ownship *ownship, 												// In: Device pointer to the ownship class, one for each thread
	TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory,				// In: Device pointer to the own-ship trajectory, one for each thread
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

	ownship[cb_index].set_wp_counter(fdata->wp_c_0);

	trajectory[cb_index].resize(6, n_samples);
	trajectory[cb_index].set_col(0, fdata->ownship_state);

	d_safe_i = 0.0; chi_m = 0.0; Pr_CC_i = 0.0; cost_ps = 0.0;
	
	cost_i.resize(fdata->n_obst, 1); cost_i.set_zero();

	// Seed collision probability estimator using the cb index
	cpe[cb_index].seed_prng(cb_index);

	// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
	n_seg_samples = std::round(cpe[cb_index].get_segment_discretization_time() / pars->dt) + 1;
	
	xs_p_seg.resize(6, n_seg_samples);
	xs_i_p_seg.resize(4, n_seg_samples);
	P_i_p_seg.resize(16, n_seg_samples);

	//======================================================================================================================
	// 1.1: Predict own-ship trajectory with the current control behaviour
	ownship[cb_index].predict_trajectory(
		trajectory[cb_index], 
		offset_sequence, 
		fdata->maneuver_times, 
		fdata->u_d, fdata->chi_d, 
		fdata->waypoints, 
		pars->prediction_method, 
		pars->guidance_method, 
		pars->T, pars->dt);

	// 1.2: Joint prediction with the current control behaviour
	if (fdata->use_joint_prediction)
	{
		printf("here jp1\n");
		predict_trajectories_jointly();
	}
	//printf("u_d = %.2f | n_obst = %d | \n", fdata->u_d, fdata->n_obst);
	//======================================================================================================================
	// 2 : Cost calculation
	// Not entirely optimal for loop configuration, but the alternative requires alot of memory
	for (int i = 0; i < fdata->n_obst; i++)
	{	
		d_safe_i = pars->d_safe + 0.5 * (ownship[cb_index].get_length() + obstacles[i].get_length());

		P_c_i.resize(fdata->n_ps[i], 1); P_c_i.set_zero();
		max_cost_ps.resize(fdata->n_ps[i], 1); max_cost_ps.set_zero();
		weights_ps.resize(fdata->n_ps[i], 1); weights_ps.set_zero();

		mu_i = obstacles[i].get_COLREGS_violation_indicator();

		cost_a.set_zero();
		Pr_a = obstacles[i].get_intention_probabilities();

		Pr_CC_i = obstacles[i].get_a_priori_CC_probability();

		ps_ordering = obstacles[i].get_ps_ordering();
		ps_intention_count = obstacles[i].get_ps_intention_count();

		for (int ps = 0; ps < fdata->n_ps[i]; ps++)
		{	
			cost_ps = 0;
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
					cpe[cb_index].initialize(
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

							P_c_i(ps) = cpe[cb_index].CE_estimate(p_os, p_i, P_i_2D, v_os_prev, v_i_prev, pars->dt);
							break;
						case MCSKF4D :                
							if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
							{
								P_c_i(ps) = cpe[cb_index].MCSKF4D_estimate(xs_p_seg, xs_i_p_seg, P_i_p_seg);						
							}	
							break;
						default :
							// Throw
							break;
					}
				}

				//==========================================================================================
				// 2.2 : Calculate and maximize dynamic obstacle cost in prediction scenario ps wrt time
				cost_ps = mpc_cost[cb_index].calculate_dynamic_obstacle_cost(
					fdata,
					obstacles, 
					P_c_i(ps), 
					xs_p_seg.get_col(n_seg_samples - 1), 
					xs_i_p_seg.get_col(n_seg_samples - 1), 
					i, 
					chi_m,
					ownship[cb_index].get_length());

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

			if (Pr_CC_i < 0.0001) // Should not be allowed to be strictly 0
			{
				Pr_CC_i = 0.0001;
			}
			
			cost_a_weight_sums.set_zero();
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				weights_ps(ps) = Pr_CC_i;

				// Last prediction scenario is the joint prediction if not pruned away
				if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)
				{
					printf("here jp3\n");
					mu_i_ps = pobstacles[i].get_COLREGS_breach_indicator();
					a_i_ps = pobstacles[i].get_intention();
					ps_intention_count(a_i_ps) += 1;
				}
				else
				{
					mu_i_ps = mu_i(ps);
					a_i_ps = ps_ordering(ps);
				}
				if (mu_i_ps)
				{
					//printf("Obstacle i = %d breaks COLREGS in ps = %d\n", i, ps);
					weights_ps(ps) = 1 - Pr_CC_i;
				}
				
				if (a_i_ps == KCC)
				{
					cost_a_weight_sums(0) += weights_ps(ps);
				}
				else if (a_i_ps == SM)
				{
					cost_a_weight_sums(1) += weights_ps(ps);
				}
				else if (a_i_ps == PM)
				{
					cost_a_weight_sums(2) += weights_ps(ps);
				}
				
			}
			
			for(int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				// Last prediction scenario is the joint prediction if not pruned away
				if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)	{ printf("here jp4\n"); a_i_ps = pobstacles[i].get_intention(); }
				else															{ a_i_ps = ps_ordering(ps); }

				if (a_i_ps == KCC)
				{
					cost_a(0) += (weights_ps(ps) / cost_a_weight_sums(0)) * max_cost_ps(ps);
				}
				else if (a_i_ps == SM)
				{
					cost_a(1) +=  (weights_ps(ps) / cost_a_weight_sums(1)) * max_cost_ps(ps);
				}
				else if (a_i_ps == PM)
				{
					cost_a(2) +=  (weights_ps(ps) / cost_a_weight_sums(2)) * max_cost_ps(ps);
				}
			}

			// Average the cost for the starboard and port maneuver type of intentions
			if (ps_intention_count(0) > 0) 	{ cost_a(0) /= ps_intention_count(0); }
			else 							{ cost_a(0) = 0.0f;}
			if (ps_intention_count(1) > 0)	{ cost_a(1) /= ps_intention_count(1); } 
			else							{ cost_a(1) = 0.0f; }
			if (ps_intention_count(2) > 0)	{ cost_a(2) /= ps_intention_count(2); } 
			else							{ cost_a(2) = 0.0f; }

			// Weight by the intention probabilities
			cost_i(i) = Pr_a.dot(cost_a);

			//=======================================================================
			// PRINTS FOR DEBUGGING PURPOSES
			//=======================================================================
			/* printf("mu_i = ");
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				// Last prediction scenario is the joint prediction if not pruned away
				if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)	{ mu_i_ps = pobstacles[i].get_COLREGS_breach_indicator(); }
				else															{ mu_i_ps = mu_i(ps); }
				printf("%d", mu_i_ps);
				printf(", ");
			}
			printf("\n");
			printf("ps_ordering = ");
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				// Last prediction scenario is the joint prediction if not pruned away
				if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)	{ a_i_ps = pobstacles[i].get_intention(); }
				else															{ a_i_ps = ps_ordering(ps); }
				printf("%u", a_i_ps);
				printf(", ");
			}
			printf("\n");
			
			printf("cost_a_weight_sums = %.2f, %.2f, %.2f\n", cost_a_weight_sums(0), cost_a_weight_sums(1), cost_a_weight_sums(2));
			printf("weights_ps = ");
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				printf("%.4f", weights_ps(ps));
				printf(", ");
			}
			printf("\n");
			printf("weights_ps normalized = ");
			for (int ps = 0; ps < fdata->n_ps[i]; ps++)
			{
				// Last prediction scenario is the joint prediction if not pruned away
				if (ps == fdata->n_ps[i] - 1 && fdata->use_joint_prediction)
				{
					a_i_ps = pobstacles[i].get_intention();
				}
				else
				{
					a_i_ps = ps_ordering(ps);
				}
				if (a_i_ps == KCC)
				{
					printf("%.4f", weights_ps(ps) / cost_a_weight_sums(0));
				}
				else if (a_i_ps == SM)
				{
					printf("%.4f", weights_ps(ps) / cost_a_weight_sums(1));
				}
				else if (a_i_ps == PM)
				{
					printf("%.4f", weights_ps(ps) / cost_a_weight_sums(2));
				}
				printf(", ");
			}
			printf("\n");
			printf("ps_intention_count = %.2f, %.2f, %.2f\n", ps_intention_count(0), ps_intention_count(1), ps_intention_count(2));
			printf("Pr_a = %.4f, %.4f, %.4f\n", Pr_a(0), Pr_a(1), Pr_a(2));
			printf("cost_a = %.4f, %.4f, %.4f | cb : %.1f, %.1f\n", cost_a(0), cost_a(1), cost_a(2), offset_sequence(0), RAD2DEG * offset_sequence(1));
			printf("cost_i(i) = %.6f | cb : %.1f, %.1f\n", cost_i(i), offset_sequence(0), RAD2DEG * offset_sequence(1));  */
			//=======================================================================
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
	cost_cb += mpc_cost[cb_index].calculate_control_deviation_cost(offset_sequence, fdata->u_opt_last, fdata->chi_opt_last);
	/* printf("dev cost = %.4f | cb : %.1f, %.1f\n", mpc_cost[cb_index].calculate_control_deviation_cost(offset_sequence, fdata->u_opt_last, fdata->chi_opt_last),  
		offset_sequence(0), RAD2DEG * offset_sequence(1)); */

	//==================================================================================================
	// 2.7 : Calculate cost due to having a wobbly offset_sequence
	cost_cb += mpc_cost[cb_index].calculate_chattering_cost(offset_sequence, fdata->maneuver_times); 
	/* printf("chat cost = %.4f | cb : %.1f, %.1f\n", mpc_cost[cb_index].calculate_chattering_cost(offset_sequence, fdata->maneuver_times),  
		offset_sequence(0), RAD2DEG * offset_sequence(1)); */
	//==================================================================================================
	printf("Cost of cb_index %d : %.4f | cb : %.1f, %.1f\n", cb_index, cost_cb, offset_sequence(0), RAD2DEG * offset_sequence(1));
	/* printf("Cost of cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f\n", cb_index, cost_cb, offset_sequence(0), RAD2DEG * offset_sequence(1), 
		offset_sequence(2), RAD2DEG * offset_sequence(3));
	printf("Cost of cb_index %d : %.4f | cb : %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", cb_index, cost_cb, offset_sequence(0), RAD2DEG * offset_sequence(1), 
		offset_sequence(2), RAD2DEG * offset_sequence(3)), offset_sequence(4), RAD2DEG * offset_sequence(5); */

	return cost_cb;
}
 
//=======================================================================================
//	Private functions
//=======================================================================================
//=======================================================================================
//  Name     : determine_situation_type
//  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ void CB_Cost_Functor::determine_situation_type(
	ST& st_A,																// In/out: Situation type of vessel A
	ST& st_B,																// In/out: Situation type of vessel B	
	const TML::Vector2f &v_A,												// In: (NE) Velocity vector of vessel A 
	const float psi_A, 														// In: Heading of vessel A
	const TML::Vector2f &v_B, 												// In: (NE) Velocity vector of vessel B
	const TML::Vector2f &L_AB, 												// In: LOS vector pointing from vessel A to vessel B
	const float d_AB 														// In: Distance from vessel A to vessel B
	)
{
	// Crash situation or outside consideration range
	if(d_AB < pars->d_safe || d_AB > pars->d_close)
	{
		st_A = A; st_B = A;
		return;
	} 
	// Inside consideration range
	else
	{
		is_ahead = v_A.dot(L_AB) > cos(pars->phi_AH) * v_A.norm();

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
					v_A.norm() > 0.25											&&
					v_B.norm() > 0.25											&&
					is_ahead;

		is_crossing = v_A.dot(v_B) < cos(pars->phi_CR) * v_A.norm() * v_B.norm()  	&&
					v_A.norm() > 0.25											&&
					v_B.norm() > 0.25											&&
					!is_head_on 												&&
					!is_passed;
		
		if (A_is_overtaken) 
		{ 
			st_A = B; st_B = D;
		} 
		else if (B_is_overtaken) 
		{ 
			st_A = D; st_B = B; 
		} 
		else if (is_head_on) 
		{ 
			st_A = E; st_B = E; 
		} 
		else if (is_crossing)
		{
			if (B_is_starboard) 
			{
				st_A = F; st_B = C;
			} else
			{
				st_A = C; st_B = F;
			}
		} 
		else 
		{
			st_A = A; st_B = A;
		}
	}
}

//=======================================================================================
//  Name     : update_conditional_obstacle_data
//  Function : Updates the situation type for the calling obstacle (wrt all other 
//			   obstacles) and obstacles (wrt own-ship) and the transitional cost 
//			   indicators O, Q, X, S at the current time t0 wrt all obstacles.
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ void CB_Cost_Functor::update_conditional_obstacle_data(
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

			determine_situation_type(data.ST_0[i_count], data.ST_i_0[i_count], v_A, psi_A, v_B, L_AB, d_AB);
			
			//=====================================================================
			// Transitional variable update
			//=====================================================================
			is_close = d_AB <= pars->d_close;

			data.AH_0[i_count] = v_A.dot(L_AB) > cos(pars->phi_AH) * v_A.norm();
			
			// Obstacle on starboard side
			data.S_TC_0[i_count] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			// Ownship on starboard side of obstacle
			data.S_i_TC_0[i_count] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			// Ownship overtaking the obstacle
			data.O_TC_0[i_count] = v_B.dot(v_A) > cos(pars->phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							    						&&
					v_B.norm() > 0.25															&&
					is_close 																	&&
					data.AH_0[i_count];

			// Obstacle overtaking the ownship
			data.Q_TC_0[i_count] = v_A.dot(v_B) > cos(pars->phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  							&&
					v_A.norm() > 0.25 															&&
					is_close 																	&&
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
__device__ void CB_Cost_Functor::predict_trajectories_jointly()
{
	u_d_i.resize(fdata->n_obst, 1); u_opt_last_i.resize(fdata->n_obst, 1);
	chi_d_i.resize(fdata->n_obst, 1); chi_opt_last_i.resize(fdata->n_obst, 1);
	for(int k = 0; k < n_samples; k++)
	{
		t = k * pars->dt;
		for (int i = 0; i < fdata->n_obst; i++)
		{
			xs_i_p = pobstacles[i].get_trajectory_sample(k);

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