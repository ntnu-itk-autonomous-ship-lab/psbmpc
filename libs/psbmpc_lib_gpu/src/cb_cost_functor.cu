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

#include <thrust/device_vector.h>
#include "cml.cuh"
#include "utilities.cuh"
#include "psbmpc.h"
#include "cb_cost_functor.cuh"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI


/****************************************************************************************
*  Name     : CB_Cost_Functor
*  Function : Class constructors, initializes parameters and variables.
*  Author   : 
*  Modified :
****************************************************************************************/
__host__ CB_Cost_Functor::CB_Cost_Functor(
	const PSBMPC &master, 												// In: Parent class
	const double u_d,  													// In: Surge reference
	const double chi_d, 												// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints, 						// In: Waypoints to follow
	const Eigen::Matrix<double, 4, -1> &static_obstacles,				// In: Static obstacle information
	const Obstacle_Data &odata											// In: Dynamic obstacle information
	) :
	pars(master.pars), cpe(master.cpe)
{	
	fdata.n_obst = odata.new_obstacles.size();

	fdata.u_d = u_d;
	fdata.chi_d = chi_d;

	fdata.u_m_last = master.u_m_last;
	fdata.chi_m_last = master.chi_m_last;

	// Assign Eigen objects to CML objects
	CML::assign_eigen_object(fdata.maneuver_times, master.maneuver_times);
	CML::assign_eigen_object(fdata.control_behaviours, master.control_behaviours);

	CML::assign_eigen_object(fdata.trajectory, master.trajectory);

	CML::assign_eigen_object(fdata.waypoints, waypoints);

	CML::assign_eigen_object(fdata.static_obstacles, static_obstacles);

	fdata.n_ps.resize(fdata.n_obst, 1);

	fdata.AH_0.resize(fdata.n_obst, 1); 	fdata.S_TC_0.resize(fdata.n_obst, 1); fdata.S_i_TC_0.resize(fdata.n_obst, 1);
	fdata.O_TC_0.resize(fdata.n_obst, 1); 	fdata.Q_TC_0.resize(fdata.n_obst, 1); fdata.IP_0.resize(fdata.n_obst, 1);
	fdata.H_TC_0.resize(fdata.n_obst, 1); 	fdata.X_TC_0.resize(fdata.n_obst, 1);


	fdata.obstacles = new Cuda_Obstacle[fdata.n_obst];
	for (int i = 0; i < fdata.n_obst; i++)
	{
		fdata.n_ps[i] = master.n_ps[i];

		fdata.AH_0[i] = odata.AH_0[i]; 
		fdata.S_TC_0[i] = odata.S_TC_0[i];
		fdata.S_i_TC_0[i] = odata.S_i_TC_0[i]; 
		fdata.O_TC_0[i] = odata.O_TC_0[i];
		fdata.Q_TC_0[i] = odata.Q_TC_0[i]; 
		fdata.IP_0[i] = odata.IP_0[i];
		fdata.H_TC_0[i] = odata.H_TC_0[i]; 
		fdata.X_TC_0[i] = odata.X_TC_0[i];

		fdata.obstacles[i] = odata.new_obstacles[i];
	}

	// Two different own-ship versions, as the psbmpc (CPU part) uses the version with
	// Eigen, whereas the GPU part on the cost functor uses the CML. Only thing that
	// needs to be transferred is the active waypoint segment. 
	ownship = Ownship();
	ownship.set_wp_counter(master.ownship.get_wp_counter());
}

/****************************************************************************************
*  Name     : ~CB_Cost_Functor
*  Function : Destructor
*  Author   : 
*  Modified :
****************************************************************************************/
__host__ __device__ CB_Cost_Functor::~CB_Cost_Functor() = default;

/****************************************************************************************
*  Name     : operator()
*  Function : This is where the fun begins
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::operator()(
	const unsigned int cb_index 									// In: Index of control behaviour evaluated in this kernel
	)
{
	int n_samples = fdata.trajectory.get_cols();
	double cost = 0;
	CML::MatrixXd P_c_i;
	CML::MatrixXd cost_i(fdata.n_obst, 1), offset_sequence = fdata.control_behaviours.get_col(cb_index);

	ownship.predict_trajectory(
		fdata.trajectory, 
		offset_sequence, 
		fdata.maneuver_times, 
		fdata.u_d, 
		fdata.chi_d, 
		fdata.waypoints, 
		pars.prediction_method, 
		pars.guidance_method, 
		pars.T, 
		pars.dt);
	
	cpe.seed_prng(cb_index);
	
	for (int i = 0; i < fdata.n_obst; i++)
	{
		if (pars.obstacle_colav_on) { predict_trajectories_jointly(); }

		P_c_i.resize(fdata.n_ps[i], n_samples);
		calculate_collision_probabilities(P_c_i, i); 

		cost_i(i) = calculate_dynamic_obstacle_cost(P_c_i, i, offset_sequence);
	}

	cost += cost_i.max_coeff();

	cost += calculate_grounding_cost();

	cost += calculate_control_deviation_cost(offset_sequence);

	cost += calculate_chattering_cost(offset_sequence);
	
	return cost;
}


/****************************************************************************************
	Private functions
****************************************************************************************/

/****************************************************************************************
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the ownship and obstacles with an active COLAV
*			  system
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CB_Cost_Functor::predict_trajectories_jointly()
{

}

/****************************************************************************************
*  Name     : determine_COLREGS_violation
*  Function : Determine if vessel A violates COLREGS with respect to vessel B.
*			  
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ bool CB_Cost_Functor::determine_COLREGS_violation(
	const CML::MatrixXd &v_A,												// In: (NE) Velocity vector of vessel A, row vector
	const double psi_A, 													// In: Heading of vessel A
	const CML::MatrixXd &v_B, 											// In: (NE) Velocity vector of vessel B, row vector
	const CML::MatrixXd &L_AB, 											// In: LOS vector pointing from vessel A to vessel B, row vector
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	bool B_is_starboard, A_is_overtaken, B_is_overtaken;
	bool is_ahead, is_close, is_passed, is_head_on, is_crossing;

	is_ahead = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

	is_close = d_AB <= pars.d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
					 v_A.norm() < v_B.norm()							  		&&
					 v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
					 v_B.norm() < v_A.norm()							  		&&
					 v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > pars.d_safe;

	is_head_on = v_A.dot(v_B) < - cos(pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
				 v_A.norm() > 0.25												&&
				 v_B.norm() > 0.25												&&
				 is_ahead;

	is_crossing = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
				  v_A.norm() > 0.25												&&
				  v_B.norm() > 0.25												&&
				  !is_head_on 													&&
				  !is_passed;

	return (is_close && B_is_starboard && is_head_on) || (is_close && B_is_starboard && is_crossing && !A_is_overtaken);
}



/****************************************************************************************
*  Name     : determine_transitional_cost_indicator
*  Function : Determine if a transitional cost should be applied for the current
*			  control behavior, using the method in Hagen, 2018. Two overloads
*  Author   : 
*  Modified :
*****************************************************************************************/
__device__ bool CB_Cost_Functor::determine_transitional_cost_indicator(
	const double psi_A, 													// In: Heading of vessel A
	const double psi_B, 													// In: Heading of vessel B
	const CML::MatrixXd &L_AB, 												// In: LOS vector pointing from vessel A to vessel B
	const double chi_m, 													// In: Candidate course offset currently followed
	const int i 															// In: Index of obstacle
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;

	// Obstacle on starboard side
	S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	// Ownship on starboard side of obstacle
	S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!fdata.S_TC_0[i]) { O_TC = fdata.O_TC_0[i] && S_TC; }
	else { O_TC = fdata.O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!fdata.S_i_TC_0[i]) { Q_TC = fdata.Q_TC_0[i] && S_i_TC; }
	else { Q_TC = fdata.Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = fdata.X_TC_0[i] && fdata.S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!fdata.S_TC_0[i]) { H_TC = fdata.H_TC_0[i] && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}

/****************************************************************************************
*  Name     : calculate_collision_probabilities
*  Function : Estimates collision probabilities for the own-ship and an obstacle i in
*			  consideration
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ void CB_Cost_Functor::calculate_collision_probabilities(
	CML::MatrixXd &P_c_i,									// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const int i 											// In: Index of obstacle
	)
{
	int n_samples = fdata.trajectory.get_cols();
	CML::MatrixXd P_i_p = fdata.obstacles[i].get_trajectory_covariance();
	double d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + fdata.obstacles[i].get_length());

	CML::MatrixXd* xs_i_p = new CML::MatrixXd[fdata.n_ps[i]];
	*xs_i_p = *fdata.obstacles[i].get_trajectories();

	// Non-optimal temporary row-vector storage solution
	CML::MatrixXd P_c_i_row(1, P_i_p.get_cols());
	for (int ps = 0; ps < fdata.n_ps[i]; ps++)
	{
		cpe.estimate_over_trajectories(P_c_i_row, fdata.trajectory, xs_i_p[ps], P_i_p, d_safe_i, i, pars.dt);

		P_c_i.set_block(ps, 0, 1, P_c_i_row.get_cols(), P_c_i_row);
	}		
}

/****************************************************************************************
*  Name     : calculate_dynamic_obstacle_cost
*  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::calculate_dynamic_obstacle_cost(
	const CML::MatrixXd &P_c_i,									// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i]+1 x n_samples
	const int i, 													// In: Index of obstacle
	const CML::MatrixXd &offset_sequence 							// In: Control behaviour currently followed
	)
{
	// l_i is the collision cost modifier depending on the obstacle track loss.
	double cost(0.0), cost_ps(0.0), C(0.0), l_i(0.0);
	CML::MatrixXd max_cost_ps(fdata.n_ps[i]);

	CML::MatrixXd* xs_i_p = new CML::MatrixXd[fdata.n_ps[i]];
	CML::MatrixXb mu_i = fdata.obstacles[i].get_COLREGS_violation_indicator();

	for (int ps = 0; ps < fdata.n_ps[i]; ps++)
	{
		xs_i_p[ps] = fdata.obstacles[i].get_trajectories()[ps];
		max_cost_ps(ps) = 0;
	}

	int n_samples = fdata.trajectory.get_cols();
	CML::MatrixXd P_i_p = fdata.obstacles[i].get_trajectory_covariance();
	double Pr_CC_i = fdata.obstacles[i].get_a_priori_CC_probability();

	CML::MatrixXd v_0_p(2, 1), v_i_p(2, 1), L_0i_p(2, 1);
	double psi_0_p, psi_i_p, d_0i_p, chi_m;
	bool mu, trans;
	for(int k = 0; k < n_samples; k++)
	{
		psi_0_p = fdata.trajectory(2, k); 
		v_0_p(0) = fdata.trajectory(3, k); 
		v_0_p(1) = fdata.trajectory(4, k); 
		v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

		// Determine active course modification at sample k
		for (int M = 0; M < pars.n_M; M++)
		{
			if (M < pars.n_M - 1)
			{
				if (k >= fdata.maneuver_times[M] && k < fdata.maneuver_times[M + 1])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
			else
			{
				if (k >= fdata.maneuver_times[M])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
		}
		
		for(int ps = 0; ps < fdata.n_ps[i]; ps++)
		{
			L_0i_p = xs_i_p[ps].get_block(0, k, 2, 1) - fdata.trajectory.get_block(0, k, 2, 1);
			d_0i_p = L_0i_p.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_0i_p = d_0i_p - 0.5 * (ownship.get_length() + fdata.obstacles[i].get_length()); 
			
			L_0i_p = L_0i_p.normalized();

			v_i_p(0) = xs_i_p[ps](2, k);
			v_i_p(1) = xs_i_p[ps](3, k);
			psi_i_p = atan2(v_i_p(1), v_i_p(0));

			C = calculate_collision_cost(v_0_p, v_i_p);

			mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

			trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, i, chi_m);

			// Track loss modifier to collision cost
			if (fdata.obstacles[i].get_duration_lost() > pars.p_step)
			{
				l_i = pars.dt * pars.p_step / fdata.obstacles[i].get_duration_lost(); // Why the 2 Giorgio?
			} else
			{
				l_i = 1;
			}

			cost_ps = l_i * C * P_c_i(ps, k) + pars.kappa * mu  + 0 * pars.kappa_TC * trans;

			// Maximize wrt time
			if (cost_ps > max_cost_ps(ps))
			{
				max_cost_ps(ps) = cost_ps;
			}
		}
	}

	// If only 1 prediction scenario
	// => Original PSB-MPC formulation
	if (fdata.n_ps[i] == 1)
	{
		delete[] xs_i_p;
		cost = max_cost_ps(0);
		return cost;
	}
	// Weight prediction scenario cost based on if obstacle follows COLREGS or not,
	// which means that higher cost is applied if the obstacle follows COLREGS
	// to a high degree (high Pr_CC_i with no COLREGS violation from its side)
	// and the own-ship breaches COLREGS
	for (int ps = 0; ps < fdata.n_ps[i]; ps++)
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

	CML::MatrixXd cost_a; cost_a.set_zero();
	CML::MatrixXd Pr_a = fdata.obstacles[i].get_intention_probabilities();
	cost_a(0) = max_cost_ps(0); 
	for(int ps = 1; ps < fdata.n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (fdata.n_ps[i] - 1) / 2 + 1)
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
	cost_a(1) = cost_a(1) / ((fdata.n_ps[i] - 1) / 2);
	cost_a(2) = cost_a(2) / ((fdata.n_ps[i] - 1) / 2);

	// Weight by the intention probabilities
	cost = Pr_a.dot(cost_a);

	delete[] xs_i_p;
	return cost;
}

/****************************************************************************************
*  Name     : calculate_ad_hoc_collision_risk
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::calculate_ad_hoc_collision_risk(
	const double d_AB, 												// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
																	// 	   reduced by half the length of the two vessels
	const double t 													// In: Prediction time t > t0 (= 0)
	)
{
	double R = 0;
	if (d_AB <= pars.d_safe)
	{
		assert(t > 0);
		R = pow(pars.d_safe / d_AB, pars.q) * (1 / pow(fabs(t), pars.p)); 
	}
	return R;
}

/****************************************************************************************
*  Name     : calculate_control_deviation_cost
*  Function : Determines penalty due to using offsets to guidance references ++
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::calculate_control_deviation_cost(
	const CML::MatrixXd &offset_sequence 								// In: Control behaviour currently followed
	)
{
	double cost = 0;
	for (int i = 0; i < pars.n_M; i++)
	{
		if (i == 0)
		{
			cost += pars.K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], fdata.u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], fdata.chi_m_last);
		}
		else
		{
			cost += pars.K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	return cost / pars.n_M;
}

/****************************************************************************************
*  Name     : calculate_chattering_cost
*  Function : Determines penalty due to using wobbly (changing between positive and negative)
* 			  course modifications
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::calculate_chattering_cost(
	const CML::MatrixXd &offset_sequence 								// In: Control behaviour currently followed
)
{
	double cost = 0;

	if (pars.n_M > 1) 
	{
		double delta_t = 0;
		for(int M = 0; M < pars.n_M - 1; M++)
		{
			if ((offset_sequence(2 * M + 1) > 0 && offset_sequence(2 * M + 3) < 0) ||
				(offset_sequence(2 * M + 1) < 0 && offset_sequence(2 * M + 3) > 0))
			{
				delta_t = fdata.maneuver_times(M + 1) - fdata.maneuver_times(M);
				cost += pars.K_sgn * exp( - delta_t / pars.T_sgn);
			}
		}
	}
	return cost;
}

/****************************************************************************************
*  Name     : calculate_grounding_cost
*  Function : Determines penalty due grounding ownship on static obstacles (no-go zones)
*  Author   : 
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::calculate_grounding_cost()
{
	double cost = 0;

	return cost;
}

/****************************************************************************************
*  Name     : find_triplet_orientation
*  Function : Find orientation of ordered triplet (p, q, r)
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ int CB_Cost_Functor::find_triplet_orientation(
	const CML::MatrixXd &p, 
	const CML::MatrixXd &q, 
	const CML::MatrixXd &r
	)
{
	// Calculate z-component of cross product (q - p) x (r - q)
    int val = (q[0] - p[0]) * (r[1] - q[1]) - (q[1] - p[1]) * (r[0] - q[0]);

    if (val == 0) return 0; // colinear
    return val < 0 ? 1 : 2; // clock or counterclockwise
}

/****************************************************************************************
*  Name     : determine_if_on_segment
*  Function : Determine if the point q is on the segment pr
*			  (really if q is inside the rectangle with diagonal pr...)
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ bool CB_Cost_Functor::determine_if_on_segment(
	const CML::MatrixXd &p, 
	const CML::MatrixXd &q, 
	const CML::MatrixXd &r
	)
{
    if (q[0] <= (double)fmax(p[0], r[0]) && q[0] >= (double)fmin(p[0], r[0]) &&
        q[1] <= (double)fmax(p[1], r[1]) && q[1] >= (double)fmin(p[1], r[1]))
        return true;
    return false;
}

/****************************************************************************************
*  Name     : determine_if_behind
*  Function : Check if the point p_1 is behind the line defined by v_1 and v_2
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ bool CB_Cost_Functor::determine_if_behind(
	const CML::MatrixXd &p_1, 
	const CML::MatrixXd &v_1, 
	const CML::MatrixXd &v_2, 
	const double distance_to_line
	)
{
    CML::MatrixXd v_diff(2, 1), n(2, 1);
    
    v_diff = v_2 - v_1;

    n(0) = -v_diff(1); n(1) = v_diff(0);
    n = n / n.norm() * distance_to_line;

    return (determine_if_on_segment(v_1 + n, p_1, v_2 + n));
}

/****************************************************************************************
*  Name     : determine_if_lines_intersect
*  Function : Determine if the line segments defined by p_1, q_1 and p_2, q_2 intersects 
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ bool CB_Cost_Functor::determine_if_lines_intersect(
	const CML::MatrixXd &p_1, 
	const CML::MatrixXd &q_1, 
	const CML::MatrixXd &p_2, 
	const CML::MatrixXd &q_2
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

/****************************************************************************************
*  Name     : distance_from_point_to_line
*  Function : Calculate distance from p to the line segment defined by q_1 and q_2
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::distance_from_point_to_line(
	const CML::MatrixXd &p, 
	const CML::MatrixXd &q_1, 
	const CML::MatrixXd &q_2
	)
{   
	CML::MatrixXd a(3, 1);
    CML::MatrixXd b(3, 1);
    a.set_block(0, 0, 2, 1, (q_1 - q_2)); 	a(2) = 0;
    b.set_block(0, 0, 2, 1, (p - q_2)); 	b(2) = 0;

    CML::MatrixXd c = a.cross(b);
    if (a.norm() > 0) return c.norm() / a.norm();
    else return -1;
}

/****************************************************************************************
*  Name     : distance_to_static_obstacle
*  Function : Calculate distance from p to obstacle defined by line segment {v_1, v_2}
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
__device__ double CB_Cost_Functor::distance_to_static_obstacle(
	const CML::MatrixXd &p, 
	const CML::MatrixXd &v_1, 
	const CML::MatrixXd &v_2
	)
{
    double d2line = distance_from_point_to_line(p, v_1, v_2);

    if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
    else return (double)fmin((v_1-p).norm(),(v_2-p).norm());
}