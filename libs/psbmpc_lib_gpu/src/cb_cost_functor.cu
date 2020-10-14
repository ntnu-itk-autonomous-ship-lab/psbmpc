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

#include "cb_cost_functor.cuh"
#include "utilities.cuh"

#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI

/****************************************************************************************
*  Name     : CB_Cost_Functor
*  Function : Class constructor
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__host__ CB_Cost_Functor::CB_Cost_Functor(
	CB_Functor_Pars *pars,  										// In: Device pointer to functor parameters, malloc-ed in PSB-MPC constructor
	CB_Functor_Data *fdata,  										// In: Device pointer to functor data, malloc-ed in PSB-MPC
	Cuda_Obstacle *obstacles,  										// In: Device pointer to obstacles, malloc-ed in PSB-MPC
	CPE *cpe 		 												// In: Device pointer to the collision probability estimator, malloc-ed in PSB-MPC constructor
	) :
	pars(pars), fdata(fdata), obstacles(obstacles), cpe(cpe)
{

}

/****************************************************************************************
*  Name     : operator()
*  Function : This is where the fun begins
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::operator()(
	const thrust::tuple<const unsigned int, TML::PDMatrix<double, 2 * MAX_N_M, 1>> &cb_tuple	// In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
	)
{
	double cost = 0;

	//======================================================================================================================
	// 1.0 : Setup
	unsigned int cb_index = thrust::get<0>(cb_tuple);
	TML::PDMatrix<double, 2 * MAX_N_M, 1> offset_sequence = thrust::get<1>(cb_tuple);

	int n_samples = round(pars->T / pars->dt);

	double d_safe_i(0.0), chi_m(0.0), Pr_CC_i(0.0), cost_ps(0.0);

	// Allocate vectors for keeping track of max cost wrt obstacle i in predictions scenario ps
	TML::PDMatrix<double, MAX_N_OBST, 1> cost_i(fdata[cb_index].n_obst, 1); cost_i.set_zero();
	TML::PDMatrix<double, MAX_N_PS, 1> P_c_i, max_cost_ps;
	TML::PDMatrix<bool, MAX_N_PS, 1> mu_i;
	
	// Allocate vectors for the obstacle intention weighted cost, and intention probability vector
	TML::Vector3d cost_a, Pr_a;

	// Seed collision probability estimator using the cb index
	cpe->seed_prng(cb_index);

	// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
	int n_seg_samples = std::round(cpe->get_segment_discretization_time() / pars->dt) + 1;
	
	// Allocate predicted ownship state and predicted obstacle i state and covariance for their prediction scenarios (ps)
	// If cpe_method = MCSKF, then dt_seg must be equal to dt;
	// If cpe_method = CE, then only the first column in these matrices are used (only the current predicted time is considered)
	TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> xs_p(6, n_seg_samples); 
	TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> xs_i_p(4, n_seg_samples);
	TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> P_i_p(16, n_seg_samples);

	// For the CE-method:
	TML::Vector2d p_os, p_i;
    TML::Matrix2d P_i_2D;

	//======================================================================================================================
	// 1.1: Predict own-ship trajectory with the current control behaviour
	fdata[cb_index].ownship.predict_trajectory(
		fdata[cb_index].trajectory, 
		offset_sequence, 
		fdata[cb_index].maneuver_times, 
		fdata[cb_index].u_d, fdata[cb_index].chi_d, 
		fdata[cb_index].waypoints, 
		pars->prediction_method, 
		pars->guidance_method, 
		pars->T, pars->dt);

	//======================================================================================================================
	// 2 : Cost calculation
	/* printf("xs_p = %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", xs_p(0, 0), xs_p(1, 0), xs_p(2, 0), xs_p(3, 0), xs_p(4, 0), xs_p(5, 0));
	printf("xs_i_p = %.1f, %.1f, %.1f, %.1f\n", xs_i_p(0, 0), xs_i_p(1, 0), xs_i_p(2, 0), xs_i_p(3, 0));

	printf("P_i_p = %.1f, %.1f, %.1f, %.1f\n", P_i_p(0, 0), P_i_p(1, 0), P_i_p(2, 0), P_i_p(3, 0));
	printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(4, 0), P_i_p(5, 0), P_i_p(6, 0), P_i_p(7, 0));
	printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(8, 0), P_i_p(9, 0), P_i_p(10, 0), P_i_p(11, 0));
	printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(12, 0), P_i_p(13, 0), P_i_p(14, 0), P_i_p(15, 0));

	printf("xs_p rows, cols: (%ld, %ld)\n", xs_p.get_rows(), xs_p.get_cols());
	printf("xs_i_p rows, cols: (%ld, %ld)\n", xs_i_p.get_rows(), xs_i_p.get_cols());
	printf("P_i_p rows, cols: (%ld, %ld)\n", P_i_p.get_rows(), P_i_p.get_cols()); */

	// Not entirely optimal for loop configuration, but the alternative requires alot of memory, so test this first.
	for (int i = 0; i < fdata[cb_index].n_obst; i++)
	{	
		d_safe_i = pars->d_safe + 0.5 * (fdata[cb_index].ownship.get_length() + obstacles[i].get_length());

		P_c_i.resize(fdata[cb_index].n_ps[i], 1); P_c_i.set_zero();
		max_cost_ps.resize(fdata[cb_index].n_ps[i], 1); max_cost_ps.set_zero();

		mu_i = obstacles[i].get_COLREGS_violation_indicator();

		cost_a.set_zero();
		Pr_a = obstacles[i].get_intention_probabilities();

		for (int ps = 0; ps < fdata[cb_index].n_ps[i]; ps++)
		{	
			cost_ps = 0;
			for (int k = 0; k < n_samples; k++)
			{	
				//==========================================================================================
				// 2.0 : Extract states and information relevant for cost evaluation at sample k. 

				xs_p.shift_columns_right();
				xs_p.set_col(0, fdata[cb_index].trajectory.get_col(k));

				P_i_p.shift_columns_right();
				P_i_p.set_col(0, obstacles[i].get_trajectory_covariance_sample(k));

				xs_i_p.shift_columns_right();
				xs_i_p.set_col(0, obstacles[i].get_trajectory_sample(ps, k));

				if (k == 0)
				{
					cpe[cb_index].initialize(xs_p.get_col(0), xs_i_p.get_col(0), P_i_p.get_col(0), d_safe_i);
				}

				// Determine active course modification at sample k
				for (int M = 0; M < pars->n_M; M++)
				{
					if (M < pars->n_M - 1)
					{
						if (k >= fdata[cb_index].maneuver_times[M] && k < fdata[cb_index].maneuver_times[M + 1])
						{
							chi_m = offset_sequence[2 * M + 1];
						}
					}
					else
					{
						if (k >= fdata[cb_index].maneuver_times[M])
						{
							chi_m = offset_sequence[2 * M + 1];
						}
					}
				}
		
				//==========================================================================================
				// 2.1 : Estimate Collision probability at time k with obstacle i in prediction scenario ps
				/* printf("i = %d | ps = %d | k = %d\n", i, ps, k);
				switch(pars->cpe_method)
				{
					case CE :	
						p_os = xs_p.get_block<2, 1>(0, 0, 2, 1);
						p_i = xs_i_p.get_block<2, 1>(0, 0, 2, 1);

						P_i_2D = reshape<16, 1, 4, 4>(P_i_p.get_col(0), 4, 4).get_block<2, 2>(0, 0, 2, 2);

						P_c_i(ps) = cpe[cb_index].CE_estimate(p_os, p_i, P_i_2D);
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
				} */

				//==========================================================================================
				// 2.2 : Calculate and maximize dynamic obstacle cost in prediction scenario ps wrt time
				cost_ps = calculate_dynamic_obstacle_cost(P_c_i(ps), xs_p.get_col(0), xs_i_p.get_col(0),  i, chi_m, cb_index);

				if (max_cost_ps(ps) < cost_ps)
				{
					max_cost_ps(ps) = cost_ps;
				}
				//==========================================================================================
				printf("P_c_i = %.1f | cost_ps = %.1f\n", P_c_i(ps), cost_ps);
			}
		}

		//==============================================================================================
		// 2.3 : Calculate a weighted obstacle cost over all prediction scenarios

		// If only 1 prediction scenario: Original PSB-MPC formulation
		if (fdata[cb_index].n_ps[i] == 1)
		{
			cost_i(i) = max_cost_ps(0);
		}
		else // Three intentions to consider: KCC, SM and PM
		{
			// Weight prediction scenario cost based on if obstacle follows COLREGS or not,
			// which means that higher cost is applied if the obstacle follows COLREGS
			// to a high degree (high Pr_CC_i with no COLREGS violation from its side)
			// and the own-ship breaches COLREGS
			for (int ps = 0; ps < fdata[cb_index].n_ps[i]; ps++)
			{
				if (mu_i[ps])
				{
					max_cost_ps(ps) = (1 - Pr_CC_i) * max_cost_ps(ps);
				}
				else
				{
					max_cost_ps(ps) = Pr_CC_i * max_cost_ps(ps);
				}
			}
			
			cost_a(0) = max_cost_ps(0); 
			for(int ps = 1; ps < fdata[cb_index].n_ps[i]; ps++)
			{
				// Starboard maneuvers
				if (ps < (fdata[cb_index].n_ps[i] - 1) / 2 + 1)
				{
					cost_a(1) += max_cost_ps(ps);
				}
				// Port maneuvers
				else
				{
					cost_a(2) += max_cost_ps(ps);
				}
			}
			// Average the cost for the corresponding intention
			cost_a(1) = cost_a(1) / ((fdata[cb_index].n_ps[i] - 1) / 2);
			cost_a(2) = cost_a(2) / ((fdata[cb_index].n_ps[i] - 1) / 2);

			// Weight by the intention probabilities
			cost_i(i) = Pr_a.dot(cost_a);
		}
		//==============================================================================================
	}
	
	//==================================================================================================
	// 2.4 : Extract maximum cost wrt all obstacles
	cost += cost_i.max_coeff(); 

	//==================================================================================================
	// 2.5 : Calculate grounding cost
	//cost += calculate_grounding_cost(); 

	//==================================================================================================
	// 2.6 : Calculate cost due to deviating from the nominal path
	cost += calculate_control_deviation_cost(offset_sequence, cb_index);

	//==================================================================================================
	// 2.7 : Calculate cost due to having a wobbly offset_sequence
	cost += calculate_chattering_cost(offset_sequence, cb_index); 
	//==================================================================================================

	return cost;
}

/* __device__ double CB_Cost_Functor::operator()(
	const thrust::tuple<const unsigned int, TML::PDMatrix<double, 20, 1>> &cb_tuple		// In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
	)
{
	double cost = 0;

	unsigned int cb_index = thrust::get<0>(cb_tuple);
	TML::PDMatrix<double, 20, 1> offset_sequence = thrust::get<1>(cb_tuple);

	int n_samples = round(pars->T / pars->dt);

	TML::PDMatrix<double, MAX_N_PS, MAX_N_SAMPLES> P_c_i;
	TML::PDMatrix<double, MAX_N_OBST, 1> cost_i(fdata[cb_index].n_obst, 1);

	printf("here1 \n");
 	ownship.predict_trajectory(
		fdata[cb_index].trajectory, 
		offset_sequence, 
		fdata[cb_index].maneuver_times, 
		fdata[cb_index].u_d, 
		fdata[cb_index].chi_d, 
		fdata[cb_index].waypoints, 
		pars->prediction_method, 
		pars->guidance_method, 
		pars->T, 
		pars->dt);
	printf("here2 \n");

	printf("n_obst = %d \n", fdata[cb_index].n_obst);
	printf("dt = %f \n", pars->dt);

	//CPE cpe(pars->cpe_method, fdata[cb_index].n_obst, pars->dt);

	//cpe.seed_prng(cb_index);

	printf("here3 \n");
	
	 for (int i = 0; i < fdata.n_obst; i++)
	{
		if (pars->obstacle_colav_on) { predict_trajectories_jointly(); }

		P_c_i.resize(fdata.n_ps[i], n_samples);
		calculate_collision_probabilities(P_c_i, i); 

		cost_i(i) = calculate_dynamic_obstacle_cost(P_c_i, i, offset_sequence);
	}

	cost += cost_i.max_coeff();

	cost += calculate_grounding_cost();

	cost += calculate_control_deviation_cost(offset_sequence);

	cost += calculate_chattering_cost(offset_sequence); 
	
	

	return cost;
} */

 
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
	const TML::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A, row vector
	const double psi_A, 													// In: Heading of vessel A
	const TML::Vector2d &v_B, 												// In: (NE) Velocity vector of vessel B, row vector
	const TML::Vector2d &L_AB, 												// In: LOS vector pointing from vessel A to vessel B, row vector
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	bool B_is_starboard, A_is_overtaken, B_is_overtaken;
	bool is_ahead, is_close, is_passed, is_head_on, is_crossing;

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

	return (is_close && B_is_starboard && is_head_on) || (is_close && B_is_starboard && is_crossing && !A_is_overtaken);
}

//=======================================================================================
//  Name     : determine_transitional_cost_indicator
//  Function : Determine if a transitional cost should be applied for the current
//			  control behavior, using the method in Hagen, 2018. Two overloads
// Author   : 
//  Modified :
//=======================================================================================
__device__ bool CB_Cost_Functor::determine_transitional_cost_indicator(
	const double psi_A, 													// In: Heading of vessel A
	const double psi_B, 													// In: Heading of vessel B
	const TML::Vector2d &L_AB, 												// In: LOS vector pointing from vessel A to vessel B
	const double chi_m, 													// In: Candidate course offset currently followed
	const int i, 															// In: Index of obstacle
	const unsigned int cb_index												// In: Index of control behaviour currently followed in this thread
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;

	// Obstacle on starboard side
	S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	// Ownship on starboard side of obstacle
	S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!fdata[cb_index].S_TC_0[i]) { O_TC = fdata[cb_index].O_TC_0[i] && S_TC; }
	else { O_TC = fdata[cb_index].O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!fdata[cb_index].S_i_TC_0[i]) { Q_TC = fdata[cb_index].Q_TC_0[i] && S_i_TC; }
	else { Q_TC = fdata[cb_index].Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = fdata[cb_index].X_TC_0[i] && fdata[cb_index].S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!fdata[cb_index].S_TC_0[i]) { H_TC = fdata[cb_index].H_TC_0[i] && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}

//=======================================================================================
//  Name     : calculate_collision_probabilities
//  Function : Estimates collision probabilities for the own-ship and an obstacle i in
//			  consideration
// Author   : Trym Tengesdal
// Modified :
//=======================================================================================
__device__ inline void CB_Cost_Functor::calculate_collision_probabilities(
	TML::PDMatrix<double, MAX_N_PS, MAX_N_SAMPLES> &P_c_i,		// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const int i, 															// In: Index of obstacle
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	int n_samples = round(pars->T / pars->dt);
	
	double d_safe_i = pars->d_safe + 0.5 * (fdata[cb_index].ownship.get_length() + obstacles[i].get_length());

	TML::PDMatrix<double, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_i_p = obstacles[i].get_trajectories();
	TML::PDMatrix<double, 16, MAX_N_SAMPLES> P_i_p = obstacles[i].get_trajectory_covariance();

	// Non-optimal temporary row-vector storage solution
	TML::PDMatrix<double, 1, MAX_N_SAMPLES> P_c_i_row(1, n_samples);
	for (int ps = 0; ps < fdata[cb_index].n_ps[i]; ps++)
	{
		//cpe[cb_index].estimate_over_trajectories(P_c_i_row, fdata[cb_index].trajectory, xs_i_p.get_block<4, MAX_N_SAMPLES>(4 * ps, 0, 4, n_samples), P_i_p, d_safe_i, i, pars->dt);

		P_c_i.set_block(ps, 0, 1, P_c_i_row.get_cols(), P_c_i_row);
	}		
}

//=======================================================================================
//  Name     : calculate_dynamic_obstacle_cost
//  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i.
//			   Two overloads: One considering whole trajectories, and one considering
//			   a certain time instant for the ownship and obstacle i in pred. scen. ps.
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ inline double CB_Cost_Functor::calculate_dynamic_obstacle_cost(
	const TML::PDMatrix<double, MAX_N_PS, MAX_N_SAMPLES> &P_c_i,	// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i]+1 x n_samples
	const int i, 													// In: Index of obstacle
	const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence, 	// In: Control behaviour currently followed
	const unsigned int cb_index										// In: Index of control behaviour currently followed in this thread
	)
{
	// l_i is the collision cost modifier depending on the obstacle track loss.
	double cost(0.0), cost_ps(0.0), C(0.0), l_i(0.0);
	TML::PDMatrix<double, MAX_N_PS, 1> max_cost_ps(fdata[cb_index].n_ps[i], 1); max_cost_ps.set_zero();

	int n_samples = round(pars->T / pars->dt);
	

	double Pr_CC_i = obstacles[i].get_a_priori_CC_probability();
	TML::PDMatrix<double, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_i_p = obstacles[i].get_trajectories();
	TML::PDMatrix<bool, MAX_N_PS, 1> mu_i = obstacles[i].get_COLREGS_violation_indicator();
	
	TML::Vector2d v_0_p, v_i_p, L_0i_p;
	double psi_0_p, psi_i_p, d_0i_p, chi_m;
	bool mu, trans;
	for(int k = 0; k < n_samples; k++)
	{
		psi_0_p = fdata[cb_index].trajectory(2, k); 
		v_0_p(0) = fdata[cb_index].trajectory(3, k); 
		v_0_p(1) = fdata[cb_index].trajectory(4, k); 
		v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

		// Determine active course modification at sample k
		for (int M = 0; M < pars->n_M; M++)
		{
			if (M < pars->n_M - 1)
			{
				if (k >= fdata[cb_index].maneuver_times[M] && k < fdata[cb_index].maneuver_times[M + 1])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
			else
			{
				if (k >= fdata[cb_index].maneuver_times[M])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
		}
		
		for(int ps = 0; ps < fdata[cb_index].n_ps[i]; ps++)
		{
			L_0i_p = xs_i_p.get_block<2, 1>(4 * ps, k, 2, 1) - fdata[cb_index].trajectory.get_block<2, 1>(0, k, 2, 1);
			d_0i_p = L_0i_p.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_0i_p = d_0i_p - 0.5 * (fdata[cb_index].ownship.get_length() + obstacles[i].get_length()); 
			
			L_0i_p = L_0i_p.normalized();

			v_i_p(0) = xs_i_p(4 * ps + 2, k);
			v_i_p(1) = xs_i_p(4 * ps + 3, k);
			psi_i_p = atan2(v_i_p(1), v_i_p(0));

			C = calculate_collision_cost(v_0_p, v_i_p);

			mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

			trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, i, chi_m, cb_index);

			// Track loss modifier to collision cost
			if (obstacles[i].get_duration_lost() > pars->p_step)
			{
				l_i = pars->dt * pars->p_step / obstacles[i].get_duration_lost(); // Why the 2 Giorgio?
			} else
			{
				l_i = 1;
			}

			cost_ps = l_i * C * P_c_i(ps, k) + pars->kappa * mu  + 0 * pars->kappa_TC * trans;

			// Maximize wrt time
			if (cost_ps > max_cost_ps(ps))
			{
				max_cost_ps(ps) = cost_ps;
			}
		}
	}

	// If only 1 prediction scenario
	// => Original PSB-MPC formulation
	if (fdata[cb_index].n_ps[i] == 1)
	{
		cost = max_cost_ps(0);
		return cost;
	}
	// Weight prediction scenario cost based on if obstacle follows COLREGS or not,
	// which means that higher cost is applied if the obstacle follows COLREGS
	// to a high degree (high Pr_CC_i with no COLREGS violation from its side)
	// and the own-ship breaches COLREGS
	for (int ps = 0; ps < fdata[cb_index].n_ps[i]; ps++)
	{
		if (mu_i[ps])
		{
			max_cost_ps(ps) = (1 - Pr_CC_i) * max_cost_ps(ps);
		}
		else
		{
			max_cost_ps(ps) = Pr_CC_i * max_cost_ps(ps);
		}
	}

	TML::Vector3d cost_a; cost_a.set_zero();
	TML::Vector3d Pr_a = obstacles[i].get_intention_probabilities();
	cost_a(0) = max_cost_ps(0); 
	for(int ps = 1; ps < fdata[cb_index].n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (fdata[cb_index].n_ps[i] - 1) / 2 + 1)
		{
			cost_a(1) += max_cost_ps(ps);
		}
		// Port maneuvers
		else
		{
			cost_a(2) += max_cost_ps(ps);
		}
	}
	// Average the cost for the corresponding intention
	cost_a(1) = cost_a(1) / ((fdata[cb_index].n_ps[i] - 1) / 2);
	cost_a(2) = cost_a(2) / ((fdata[cb_index].n_ps[i] - 1) / 2);

	// Weight by the intention probabilities
	cost = Pr_a.dot(cost_a);

	return cost;
}

__device__ inline double CB_Cost_Functor::calculate_dynamic_obstacle_cost(
	const double P_c_i,															// In: Predicted obstacle collision probabilities for obstacle in prediction scenario ps
	const TML::PDVector6d xs_p, 												// In: Predicted own-ship state at time step k
	const TML::PDVector4d xs_i_p, 												// In: Predicted obstacle state at time step k in prediction scenario ps
	const int i, 																// In: Index of obstacle
	const double chi_m,														 	// In: Course offset used by the own-ship at time step k
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	// l_i is the collision cost modifier depending on the obstacle track loss.
	double cost(0.0), C(0.0), l_i(0.0);

	double Pr_CC_i = obstacles[i].get_a_priori_CC_probability();
	
	TML::Vector2d v_0_p, v_i_p, L_0i_p;
	double psi_0_p, psi_i_p, d_0i_p;
	bool mu, trans;

	psi_0_p = xs_p(2); 
	v_0_p(0) = xs_p(3); 
	v_0_p(1) = xs_p(4); 
	v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

	L_0i_p = xs_i_p.get_block<2, 1>(0, 0, 2, 1) - xs_p.get_block<2, 1>(0, 0, 2, 1);
	d_0i_p = L_0i_p.norm();

	// Decrease the distance between the vessels by their respective max dimension
	d_0i_p = d_0i_p - 0.5 * (fdata[cb_index].ownship.get_length() + obstacles[i].get_length()); 
	
	L_0i_p = L_0i_p.normalized();

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

	cost = l_i * C * P_c_i + pars->kappa * mu  + 0 * pars->kappa_TC * trans;

	return cost;
}

//=======================================================================================
//  Name     : calculate_ad_hoc_collision_risk
//  Function : 
//  Author   : 
//  Modified :
//=======================================================================================
__device__ double CB_Cost_Functor::calculate_ad_hoc_collision_risk(
	const double d_AB, 												// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
																	// 	   reduced by half the length of the two vessels
	const double t 													// In: Prediction time t > t0 (= 0)
	)
{
	double R = 0;
	if (d_AB <= pars->d_safe)
	{
		assert(t > 0);
		R = pow(pars->d_safe / d_AB, pars->q) * (1 / pow(fabs(t), pars->p)); 
	}
	return R;
}

//=======================================================================================
//  Name     : calculate_control_deviation_cost
//  Function : Determines penalty due to using offsets to guidance references ++
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ double CB_Cost_Functor::calculate_control_deviation_cost(
	const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence, 				// In: Control behaviour currently followed
	const unsigned int cb_index													// In: Index of control behaviour currently followed in this thread
	)
{
	double cost = 0;
	for (int i = 0; i < pars->n_M; i++)
	{
		if (i == 0)
		{
			cost += pars->K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], fdata[cb_index].u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], fdata[cb_index].chi_m_last);
		}
		else
		{
			cost += pars->K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	return cost / (double)pars->n_M;
}

//=======================================================================================
//  Name     : calculate_chattering_cost
//  Function : Determines penalty due to using wobbly (changing between positive and negative)
// 			  course modifications
//  Author   : Trym Tengesdal
//  Modified :
//=======================================================================================
__device__ double CB_Cost_Functor::calculate_chattering_cost(
	const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence, 			// In: Control behaviour currently followed
	const unsigned int cb_index												// In: Index of control behaviour currently followed in this thread
)
{
	double cost = 0;

	if (pars->n_M > 1) 
	{
		double delta_t = 0;
		for(int M = 0; M < pars->n_M - 1; M++)
		{
			if ((offset_sequence(2 * M + 1) > 0 && offset_sequence(2 * M + 3) < 0) ||
				(offset_sequence(2 * M + 1) < 0 && offset_sequence(2 * M + 3) > 0))
			{
				delta_t = fdata[cb_index].maneuver_times(M + 1) - fdata[cb_index].maneuver_times(M);
				cost += pars->K_sgn * exp( - delta_t / pars->T_sgn);
			}
		}
	}
	return cost;
}

//=======================================================================================
//  Name     : calculate_grounding_cost
//  Function : Determines penalty due grounding ownship on static obstacles (no-go zones)
//  Author   : Trym Tengesdal & Giorgio D. Kwame Minde Kufoalor
//  Modified :
//=======================================================================================
__device__ double CB_Cost_Functor::calculate_grounding_cost()
{
	double cost = 0;

	return cost;
}

//=======================================================================================
//  Name     : find_triplet_orientation
//  Function : Find orientation of ordered triplet (p, q, r)
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal for more readability
//=======================================================================================
__device__ int CB_Cost_Functor::find_triplet_orientation(
	const TML::Vector2d &p, 
	const TML::Vector2d &q, 
	const TML::Vector2d &r
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
	const TML::Vector2d &p, 
	const TML::Vector2d &q, 
	const TML::Vector2d &r
	)
{
    if (q[0] <= (double)fmax(p[0], r[0]) && q[0] >= (double)fmin(p[0], r[0]) &&
        q[1] <= (double)fmax(p[1], r[1]) && q[1] >= (double)fmin(p[1], r[1]))
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
	const TML::Vector2d &p_1, 
	const TML::Vector2d &v_1, 
	const TML::Vector2d &v_2, 
	const double distance_to_line
	)
{
    TML::Vector2d v_diff, n;
    
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
	const TML::Vector2d &p_1, 
	const TML::Vector2d &q_1, 
	const TML::Vector2d &p_2, 
	const TML::Vector2d &q_2
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
__device__ double CB_Cost_Functor::distance_from_point_to_line(
	const TML::Vector2d &p, 
	const TML::Vector2d &q_1, 
	const TML::Vector2d &q_2
	)
{   
	TML::Vector3d a;
    TML::Vector3d b;
    a.set_block<2, 1>(0, 0, (q_1 - q_2)); 	a(2) = 0;
    b.set_block<2, 1>(0, 0, (p - q_2)); 	b(2) = 0;

    TML::Vector3d c = a.cross(b);
    if (a.norm() > 0) return c.norm() / a.norm();
    else return -1;
}

//=======================================================================================
//  Name     : distance_to_static_obstacle
//  Function : Calculate distance from p to obstacle defined by line segment {v_1, v_2}
//  Author   : Giorgio D. Kwame Minde Kufoalor
//  Modified : By Trym Tengesdal
//=======================================================================================
__device__ double CB_Cost_Functor::distance_to_static_obstacle(
	const TML::Vector2d &p, 
	const TML::Vector2d &v_1, 
	const TML::Vector2d &v_2
	)
{
    double d2line = distance_from_point_to_line(p, v_1, v_2);

    if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
    else return (double)fmin((v_1-p).norm(),(v_2-p).norm());
} 