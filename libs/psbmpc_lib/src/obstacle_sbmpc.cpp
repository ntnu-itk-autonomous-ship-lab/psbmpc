/****************************************************************************************
*
*  File name : obstacle_sbmpc.cpp
*
*  Function  : Class functions for Probabilistic Scenario-based Model Predictive Control
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

#include "utilities.h"
#include "obstacle_sbmpc.h"
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI


/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC::Obstacle_SBMPC() :
	ownship(new Obstacle_Ship())
{
	// Initialize parameters before parameter limits, as some limits depend on the
	// parameter values set.
	initialize_pars();
	
	initialize_par_limits();
}

/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC::Obstacle_SBMPC(
	const Obstacle_SBMPC &o_sbmpc 									// In: Obstacle SBMPC to copy
	)
{
	this->n_cbs = o_sbmpc.n_cbs;
	this->n_M = o_sbmpc.n_M;

	this->u_offsets = o_sbmpc.u_offsets;
	this->chi_offsets = o_sbmpc.chi_offsets;

	this->offset_sequence_counter = o_sbmpc.offset_sequence_counter;
	this->offset_sequence = o_sbmpc.offset_sequence;
	this->maneuver_times = o_sbmpc.maneuver_times;

	this->u_m_last = o_sbmpc.u_m_last;
	this->chi_m_last = o_sbmpc.chi_m_last;

	this->min_cost = o_sbmpc.min_cost;

	this->dpar_low = o_sbmpc.dpar_low;
	this->dpar_high = o_sbmpc.dpar_high;
	this->ipar_low = o_sbmpc.ipar_low;
	this->ipar_high = o_sbmpc.ipar_high;

	this->prediction_method = o_sbmpc.prediction_method;
	this->guidance_method = o_sbmpc.guidance_method;

	this->T = o_sbmpc.T; this->T_static = o_sbmpc.T_static;
	this->dt = o_sbmpc.dt; 
	this->p_step = o_sbmpc.p_step;
	this->t_ts = o_sbmpc.t_ts;
	
	this->d_safe = o_sbmpc.d_safe; this->d_close = o_sbmpc.d_close; this->d_init = o_sbmpc.d_init;
	
	this->K_coll = o_sbmpc.K_coll;

	this->phi_AH = o_sbmpc.phi_AH; this->phi_OT = o_sbmpc.phi_OT; this->phi_HO = o_sbmpc.phi_HO; this->phi_CR = o_sbmpc.phi_CR;

	this->kappa = o_sbmpc.kappa; this->kappa_TC = o_sbmpc.kappa_TC;

	this->K_u = o_sbmpc.K_u; this->K_du = o_sbmpc.K_du;

	this->K_chi_strb = o_sbmpc.K_chi_strb; this->K_dchi_strb = o_sbmpc.K_dchi_strb;
	this->K_chi_port = o_sbmpc.K_chi_port; this->K_dchi_port = o_sbmpc.K_dchi_port;

	this->K_sgn = o_sbmpc.K_sgn; this->T_sgn = o_sbmpc.T_sgn;
	
	this->G = o_sbmpc.G;

	this->q = o_sbmpc.q; this->p = o_sbmpc.p;

	this->obstacle_colav_on = o_sbmpc.obstacle_colav_on;

	this->ownship.reset(new Obstacle_Ship(*(o_sbmpc.ownship)));

	this->trajectory = o_sbmpc.trajectory;

	this->AH_0 = o_sbmpc.AH_0; this->S_TC_0 = o_sbmpc.S_TC_0; this->S_i_TC_0 = o_sbmpc.S_i_TC_0;
	this->O_TC_0 = o_sbmpc.O_TC_0; this->Q_TC_0 = o_sbmpc.Q_TC_0; this->IP_0 = o_sbmpc.IP_0;
	this->H_TC_0 = o_sbmpc.H_TC_0; this->X_TC_0 = o_sbmpc.X_TC_0;

	old_obstacles.resize(old_obstacles.size());
	for (int i = 0; i < old_obstacles.size(); i++)
	{
		old_obstacles[i].reset(new Prediction_Obstacle(*(old_obstacles[i])));
	}
	new_obstacles.resize(new_obstacles.size());
	for (int i = 0; i < new_obstacles.size(); i++)
	{
		new_obstacles[i].reset(new Prediction_Obstacle(*(new_obstacles[i])));
	}
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC& Obstacle_SBMPC::operator=(
	const Obstacle_SBMPC &o_sbmpc 									// In: Rhs Obstacle SBMPC to assign
	)
{
	if (this == &o_sbmpc)
	{
		return *this;
	}

	return *this = Obstacle_SBMPC(o_sbmpc);
}

/****************************************************************************************
*  Name     : determine_COLREGS_violation
*  Function : Determine if vessel A violates COLREGS with respect to vessel B. 
*			  Two overloads.
*			  
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool Obstacle_SBMPC::determine_COLREGS_violation(
	const Eigen::Vector2d& v_A,												// In: (NE) Velocity vector of vessel A
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d& v_B, 											// In: (NE) Velocity vector of vessel B
	const Eigen::Vector2d& L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	bool B_is_starboard, A_is_overtaken, B_is_overtaken;
	bool is_ahead, is_close, is_passed, is_head_on, is_crossing;

	is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	is_close = d_AB <= d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
					 v_A.norm() < v_B.norm()							  	&&
					 v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
					 v_B.norm() < v_A.norm()							  	&&
					 v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > d_safe;

	is_head_on = v_A.dot(v_B) < - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				 v_A.norm() > 0.25											&&
				 v_B.norm() > 0.25											&&
				 is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  	&&
				  v_A.norm() > 0.25											&&
				  v_B.norm() > 0.25											&&
				  !is_head_on 												&&
				  !is_passed;

	return (is_close && B_is_starboard && is_head_on) || (is_close && B_is_starboard && is_crossing && !A_is_overtaken);
}

bool Obstacle_SBMPC::determine_COLREGS_violation(
	const Eigen::VectorXd &xs_A,											// In: State vector of vessel A 
	const Eigen::VectorXd &xs_B 											// In: State vector of vessel B
	)
{
	bool B_is_starboard, A_is_overtaken, B_is_overtaken;
	bool is_ahead, is_close, is_passed, is_head_on, is_crossing;

	Eigen::Vector2d v_A, v_B, L_AB;
	double psi_A, psi_B;
	if (xs_A.size() == 6) { psi_A = xs_A(2); v_A(0) = xs_A(3); v_A(1) = xs_A(4); v_A = rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B(2); v_B(0) = xs_B(3); v_B(1) = xs_B(4); v_B = rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	L_AB(0) = xs_B(0) - xs_A(0);
	L_AB(1) = xs_B(1) - xs_A(1);
	double d_AB = L_AB.norm();
	L_AB = L_AB / L_AB.norm();

	is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	is_close = d_AB <= d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  		&&
					v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							  		&&
					v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;
	
	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > d_safe;

	is_head_on = v_A.dot(v_B) < - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25											&&
				is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  	&&
				v_A.dot(L_AB) > cos(90 * DEG2RAD) * v_A.norm()				&& 	// Its not a crossing situation if A's velocity vector points away from vessel B  
				v_A.norm() > 0.25											&&	// or does not make a crossing occur. This is not mentioned in Johansen.
				v_B.norm() > 0.25											&&
				!is_head_on 												&&
				!is_passed;

	bool mu = (is_close && B_is_starboard && is_head_on) || (is_close && B_is_starboard && is_crossing && !A_is_overtaken);
	if (mu)
	{
		std::cout << mu << std::endl;
	}
	return mu;
}

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : Calculate optimal surge and course offsets for PSB-MPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 2, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	Eigen::Matrix<double,-1,-1> &obstacle_status,							// In/out: Various information on obstacles
	Eigen::Matrix<double,-1, 1> &colav_status,								// In/out: status on the COLAV system
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::Vector4d &ownship_state, 									// In: Current ship state
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 					// In: Dynamic obstacle states 
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 				// In: Obstacle covariance information
	const Eigen::Matrix<double, 4, -1> &static_obstacles					// In: Static obstacle information
	)
{
	int n_samples = std::round(T / dt);

	trajectory.resize(4, n_samples);
	trajectory.col(0) = ownship_state;

	update_obstacles(obstacle_states, obstacle_covariances);
	int n_obst = new_obstacles.size();
	int n_static_obst = static_obstacles.cols();

	Eigen::VectorXd HL_0(n_obst); 

	bool colav_active = determine_colav_active(n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		return;
	}
	
	update_transitional_variables();

	initialize_prediction();

	if (!obstacle_colav_on)
	{
		for (int i = 0; i < n_obst; i++)
		{
			new_obstacles[i]->predict_independent_trajectory(T, dt);
		}
	}

	double cost;
	Eigen::VectorXd opt_offset_sequence(2 * n_M), cost_i(n_obst);

	reset_control_behavior();
	for (int cb = 0; cb < n_cbs; cb++)
	{
		cost = 0;
		//std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
		//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
		ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, prediction_method, guidance_method, T, dt);

		for (int i = 0; i < n_obst; i++)
		{
			cost_i(i) = calculate_dynamic_obstacle_cost(i);
		}

		cost += cost_i.maxCoeff();

		cost += calculate_grounding_cost(static_obstacles);

		cost += calculate_control_deviation_cost();

		cost += calculate_chattering_cost();
		
		if (cost < min_cost) 
		{
			min_cost = cost;
			opt_offset_sequence = offset_sequence;

			// Set current optimal x-y position trajectory, downsample if linear prediction was not used
			if (prediction_method > Linear)
			{
				int count = 0;
				predicted_trajectory.resize(2, n_samples / p_step);
				for (int k = 0; k < n_samples; k+=p_step)
				{
					predicted_trajectory.col(count) = trajectory.block<2, 1>(0, k);
					if (count < std::round(n_samples / p_step) - 1) count++;					
				}
			} 
			else
			{
				predicted_trajectory = trajectory.block(2, n_samples, 0, 0);
			}

			// Assign current optimal hazard level for each obstacle
			for (int i = 0; i < n_obst; i++)
			{
				HL_0(i) = cost_i(i) / cost_i.sum();
			}	
		}
		increment_control_behavior();
	}

	update_obstacle_status(obstacle_status, HL_0);

	u_opt = opt_offset_sequence(0); 	u_m_last = u_opt;
	chi_opt = opt_offset_sequence(1); 	chi_m_last = chi_opt;

	double CF_0 = u_opt * (1 - fabs(chi_opt * RAD2DEG) / 15.0);
	colav_status.resize(2,1);
	colav_status << CF_0, min_cost;
}

/****************************************************************************************
	Private functions
****************************************************************************************/

/****************************************************************************************
*  Name     : initialize_par_limits
*  Function : Sets initial low and high limits on tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::initialize_par_limits()
{
	ipar_low.resize(N_IPAR); ipar_high.resize(N_IPAR);
	for (int i = 0; i < N_IPAR; i++)
	{
		ipar_low[i] = 0.0;
		ipar_high[i] = 1e12;
	}
	ipar_low[i_ipar_n_M] = 1; ipar_high[i_ipar_n_M] = 5; 

	//std::cout << "i_par_low = " << ipar_low.transpose() << std::endl;
	//std::cout << "i_par_high = " << ipar_high.transpose() << std::endl;

	dpar_low.resize(N_DPAR); dpar_high.resize(N_DPAR);
	for (int i = 0; i < N_DPAR; i++)
	{
		dpar_low[i] = 0.0;
		dpar_high[i] = 1e12;
	}
	dpar_low[i_dpar_T] = 60.0;
	dpar_low[i_dpar_T_static] = 10.0;
	dpar_low[i_dpar_dt] = 0.001;
	dpar_low[i_dpar_p_step] = 0.001;

	dpar_low[i_dpar_d_safe] = 20.0;
	dpar_low[i_dpar_d_close] = d_safe; 			
	dpar_low[i_dpar_d_init] = d_safe; 			

	dpar_high[i_dpar_K_dchi_strb] = 3.0;
	dpar_high[i_dpar_K_dchi_port] = 3.0;

	dpar_low[i_dpar_phi_AH] = -180.0; 		dpar_high[i_dpar_phi_AH] = 180.0;
	dpar_low[i_dpar_phi_OT] = -180.0;		dpar_high[i_dpar_phi_OT] = 180.0;
	dpar_low[i_dpar_phi_HO] = -180.0; 		dpar_high[i_dpar_phi_HO] = 180.0;
	dpar_low[i_dpar_phi_CR] = -180.0; 		dpar_high[i_dpar_phi_CR] = 180.0;

	//std::cout << "d_par_low = " << dpar_low.transpose() << std::endl;
	//std::cout << "d_par_high = " << dpar_high.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : initialize_pars
*  Function : Sets initial values for Obstacle_SBMPC tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::initialize_pars()
{
	n_cbs = 1;
	n_M = 1;

	offset_sequence_counter.resize(2 * n_M);
	offset_sequence.resize(2 * n_M);

	chi_offsets.resize(n_M);
	u_offsets.resize(n_M);
	for (int M = 0; M < n_M; M++)
	{
		if (M == 0)
		{
			u_offsets[M].resize(1);
			u_offsets[M] << 1.0; //, 0.5, 0.0;

			chi_offsets[M].resize(7);
			chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		} 
		else
		{
			u_offsets[M].resize(2);
			u_offsets[M] << 1.0, 0.5;

			chi_offsets[M].resize(7);
			chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		}
		n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
	}
	reset_control_behavior();

	u_m_last = 1;
	chi_m_last = 0;

	prediction_method = ERK1;
	guidance_method = LOS;

	T = 100.0; 	      // 400.0, 300.0, 240 (sim/Euler)
	dt = 5.0;		      // 5.0, 0.5 (sim/Euler)
  	T_static = 60.0;		  // (50.0)

	p_step = 1;
	if (prediction_method == ERK1)
	{ 
		dt = 0.5; 
		p_step = 10;
	}
	t_ts = 25;

	d_init = 1500;							//1852.0;	  // should be >= D_CLOSE 300.0 600.0 500.0 700.0 800 1852
	d_close = 1000;							//1000.0;	// 200.0 300.0 400.0 500.0 600 1000
	d_safe = 50; 							//185.2; 	  // 40.0, 50.0, 70.0, 80.0, 100, 200, 185.2
	K_coll = 1.0;		  					
	phi_AH = 68.5 * DEG2RAD;		 	
	phi_OT = 68.5 * DEG2RAD;		 		 
	phi_HO = 22.5 * DEG2RAD;		 		
	phi_CR = 68.5 * DEG2RAD;	     		
	kappa = 3.0;		  					
	kappa_TC = 100.0;						 
	K_u = 3;		   						 
	K_du = 2.5;		    					
	K_chi_strb = 1.3;	  					
	K_chi_port =  1.6;	  					
	K_dchi_strb = 0.9;	 			
	K_dchi_port = 1.2;	  					
	G = 0;		         					 // 1.0e3


	min_cost = 1e10;
}

/****************************************************************************************
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::initialize_prediction()
{
	int n_obst = new_obstacles.size();

	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************

	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa;
	for (int i = 0; i < n_obst; i++)
	{
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), new_obstacles[i]->get_state());
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(n_M);
	// First avoidance maneuver is always at t0
	maneuver_times(0) = 0;
	double d_cpa_min = 1e10, t_cpa_min;
	for (int M = 1; M < n_M; M++)
	{
		for (int i = 0; i < n_obst; i++)
		{
			// For the current avoidance maneuver, determine which obstacle that should be
			// considered, i.e. the closest obstacle that is not already passed (which means
			// that the previous avoidance maneuver happened before CPA with this obstacle)
			if (maneuver_times(M - 1) * dt < t_cpa(i))
			{	
				if (d_cpa(i) < d_cpa_min)
				{
					d_cpa_min = d_cpa(i);
					t_cpa_min = t_cpa(i);
				}
			}
		}
		// If no predicted collision,  avoidance maneuver M with the closest
		// obstacle (that is not passed) is taken at t_cpa_min
		if (d_cpa_min > d_safe)
		{
			maneuver_times(M) = std::round(t_cpa_min / dt);
		}
		// If a predicted collision occurs with the closest obstacle, avoidance maneuver 
		// M is taken right after the obstacle maneuvers (which will be at t_0 + M * t_ts)
		// Given that t_cpa > t_ts. If t_cpa < t_ts, the subsequent maneuver is taken
		// at t_0 + M * t_ts + 1 anyways (simplification)
		else
		{
			maneuver_times(M) = maneuver_times(M - 1) + std::round((t_ts + 1) / dt);
		}
	}
	std::cout << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branches of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::reset_control_behavior()
{
	offset_sequence_counter.setZero();
	for (int M = 0; M < n_M; M++)
	{
		offset_sequence(2 * M) = u_offsets[M](0);
		offset_sequence(2 * M + 1) = chi_offsets[M](0);
	}
}

/****************************************************************************************
*  Name     : increment_control_behavior
*  Function : Increments the control behavior counter and changes the offset sequence 
*			  accordingly. Backpropagation is used for the incrementation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::increment_control_behavior()
{
	for (int M = n_M - 1; M > -1; M--)
	{
		// Only increment counter for "leaf node offsets" on each iteration, which are the
		// course offsets in the last maneuver
		if (M == n_M - 1)
		{
			offset_sequence_counter(2 * M + 1) += 1;
		}

		// If one reaches the end of maneuver M's course offsets, reset corresponding
		// counter and increment surge offset counter above
		if (offset_sequence_counter(2 * M + 1) == chi_offsets[M].size())
		{
			offset_sequence_counter(2 * M + 1) = 0;
			offset_sequence_counter(2 * M) += 1;
		}
		offset_sequence(2 * M + 1) = chi_offsets[M](offset_sequence_counter(2 * M + 1));

		// If one reaches the end of maneuver M's surge offsets, reset corresponding
		// counter and increment course offset counter above (if any)
		if (offset_sequence_counter(2 * M) == u_offsets[M].size())
		{
			offset_sequence_counter(2 * M) = 0;
			if (M > 0)
			{
				offset_sequence_counter(2 * M - 1) += 1;
			}
		}
		offset_sequence(2 * M) = u_offsets[M](offset_sequence_counter(2 * M));
	}
}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the freshly updated new_obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool Obstacle_SBMPC::determine_colav_active(
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Vector4d xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (int i = 0; i < new_obstacles.size(); i++)
	{
		d_0i(0) = new_obstacles[i]->get_state()(0) - xs(0);
		d_0i(1) = new_obstacles[i]->get_state()(1) - xs(1);
		if (d_0i.norm() < d_init) colav_active = true;
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
}

/****************************************************************************************
*  Name     : determine_transitional_cost_indicator
*  Function : Determine if a transitional cost should be applied for the current
*			  control behavior, using the method in Hagen, 2018. Two overloads
*  Author   : 
*  Modified :
*****************************************************************************************/
bool Obstacle_SBMPC::determine_transitional_cost_indicator(
	const double psi_A, 													// In: Heading of vessel A
	const double psi_B, 													// In: Heading of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const int i, 															// In: Index of obstacle
	const double chi_m 														// In: Candidate course offset currently followed
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;

	// Obstacle on starboard side
	S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	// Ownship on starboard side of obstacle
	S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!S_TC_0[i]) { O_TC = O_TC_0[i] && S_TC; }
	else { O_TC = O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!S_i_TC_0[i]) { Q_TC = Q_TC_0[i] && S_i_TC; }
	else { Q_TC = Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = X_TC_0[i] && S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!S_TC_0[i]) { H_TC = H_TC_0[i] && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}

bool Obstacle_SBMPC::determine_transitional_cost_indicator(
	const Eigen::VectorXd& xs_A,											// In: State vector of vessel A (the ownship)
	const Eigen::VectorXd& xs_B, 											// In: State vector of vessel B (the obstacle)
	const int i, 															// In: Index of obstacle
	const double chi_m 														// In: Candidate course offset currently followed
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;
	double psi_A, psi_B;
	Eigen::Vector2d L_AB;
	if (xs_A.size() == 6) { psi_A = xs_A[2]; }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); }
	
	if (xs_B.size() == 6) { psi_B = xs_B[2]; }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); }

	L_AB(0) = xs_B(0) - xs_A(0);
	L_AB(1) = xs_B(1) - xs_A(1);
	L_AB = L_AB / L_AB.norm();

	// Obstacle on starboard side
	S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

	// Ownship on starboard side of obstacle
	S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!S_TC_0[i]) { O_TC = O_TC_0[i] && S_TC; }
	else { O_TC = O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!S_i_TC_0[i]) { Q_TC = Q_TC_0[i] && S_i_TC; }
	else { Q_TC = Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = X_TC_0[i] && S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!S_TC_0[i]) { H_TC = H_TC_0[i] && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}


/****************************************************************************************
*  Name     : calculate_dynamic_obstacle_cost
*  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_dynamic_obstacle_cost(
	const int i 													// In: Index of obstacle
	)
{
	double cost = 0;
	return cost;
}

/****************************************************************************************
*  Name     : calculate_collision_cost
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_collision_cost(
	const Eigen::Vector2d &v_1, 												// In: Velocity v_1
	const Eigen::Vector2d &v_2 												// In: Velocity v_2
	)
{
	return K_coll * (v_1 - v_2).norm();
}

/****************************************************************************************
*  Name     : calculate_ad_hoc_collision_risk
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_ad_hoc_collision_risk(
	const double d_AB, 														// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
	const double t 															// In: Prediction time t > t0 (= 0)
	)
{
	double R = 0;
	if (d_AB <= d_safe)
	{
		assert(t > 0);
		R = pow(d_safe / d_AB, q) * (1 / pow(fabs(t), p)); 
	}
	return R;
}

/****************************************************************************************
*  Name     : calculate_control_deviation_cost
*  Function : Determines penalty due to using offsets to guidance references ++
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_control_deviation_cost()
{
	double cost = 0;
	for (int i = 0; i < n_M; i++)
	{
		if (i == 0)
		{
			cost += K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], chi_m_last);
		}
		else
		{
			cost += K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	return cost / n_M;
}

/****************************************************************************************
*  Name     : calculate_chattering_cost
*  Function : Determines penalty due to using wobly (changing between positive and negative)
* 			  course modifications
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_chattering_cost()
{
	double cost = 0;

	if (n_M > 1) 
	{
		double delta_t = 0;
		for(int M = 0; M < n_M; M++)
		{
			if (M < n_M - 1)
			{
				if ((offset_sequence(M + 1) >= 0 && offset_sequence(2 * M + 1) < 0) ||
					(offset_sequence(M + 1) < 0 && offset_sequence(2 * M + 1) >= 0))
				{
					delta_t = maneuver_times(M+1) - maneuver_times(M);
					cost += K_sgn * exp( - delta_t / T_sgn);
				}
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
double Obstacle_SBMPC::calculate_grounding_cost(
	const Eigen::Matrix<double, 4, -1>& static_obstacles						// In: Static obstacle information
	)
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
int Obstacle_SBMPC::find_triplet_orientation(
	const Eigen::Vector2d &p, 
	const Eigen::Vector2d &q, 
	const Eigen::Vector2d &r
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
bool Obstacle_SBMPC::determine_if_on_segment(
	const Eigen::Vector2d &p, 
	const Eigen::Vector2d &q, 
	const Eigen::Vector2d &r
	)
{
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
        q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
        return true;
    return false;
}

/****************************************************************************************
*  Name     : determine_if_behind
*  Function : Check if the point p_1 is behind the line defined by v_1 and v_2
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
bool Obstacle_SBMPC::determine_if_behind(
	const Eigen::Vector2d &p_1, 
	const Eigen::Vector2d &v_1, 
	const Eigen::Vector2d &v_2, 
	const double distance_to_line
	)
{
    Eigen::Vector2d v_diff, n;
    
    v_diff = v_2 - v_1;

    n << -v_diff[1], v_diff[0];
    n = n / n.norm() * distance_to_line;

    return (determine_if_on_segment(v_1 + n, p_1, v_2 + n));
}

/****************************************************************************************
*  Name     : determine_if_lines_intersect
*  Function : Determine if the line segments defined by p_1, q_1 and p_2, q_2 intersects 
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
bool Obstacle_SBMPC::determine_if_lines_intersect(
	const Eigen::Vector2d &p_1, 
	const Eigen::Vector2d &q_1, 
	const Eigen::Vector2d &p_2, 
	const Eigen::Vector2d &q_2
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
double Obstacle_SBMPC::distance_from_point_to_line(
	const Eigen::Vector2d &p, 
	const Eigen::Vector2d &q_1, 
	const Eigen::Vector2d &q_2
	)
{   
	Eigen::Vector3d a;
    Eigen::Vector3d b;
    a << (q_1 - q_2), 0;
    b << (p - q_2), 0;

    Eigen::Vector3d c = a.cross(b);
    if (a.norm() > 0) return c.norm() / a.norm();
    else return -1;
}

/****************************************************************************************
*  Name     : distance_to_static_obstacle
*  Function : Calculate distance from p to obstacle defined by line segment {v_1, v_2}
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::distance_to_static_obstacle(
	const Eigen::Vector2d &p, 
	const Eigen::Vector2d &v_1, 
	const Eigen::Vector2d &v_2
	)
{
    double d2line = distance_from_point_to_line(p, v_1, v_2);

    if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
    else return std::min((v_1-p).norm(),(v_2-p).norm());
}

/****************************************************************************************
*  Name     : assign_optimal_trajectory
*  Function : Set the optimal trajectory to the current predicted trajectory
*  Author   :
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::assign_optimal_trajectory(
	Eigen::Matrix<double, 2, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = round(T / dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(2, n_samples / p_step);
		for (int k = 0; k < n_samples; k+=p_step)
		{
			optimal_trajectory.col(count) = trajectory.block<2, 1>(0, k);
			if (count < round(n_samples / p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(2, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 2, n_samples);
	}
}

/****************************************************************************************
*  Name     : update_obstacles
*  Function : Takes in new obstacle information and updates the obstacle data structures
*  Author   :
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::update_obstacles(
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic predicted obstacle states
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances 							// In: Obstacle covariance information
	) 			
{
	// Clear "old" new obstacle vector before the update
	new_obstacles.clear();

	int n_obst_old = old_obstacles.size();
	int n_obst_new = obstacle_states.cols();

	bool obstacle_exist;
	for (int i = 0; i < n_obst_new; i++)
	{
		obstacle_exist = false;
		for (int j = 0; j < n_obst_old; j++)
		{
			if ((double)old_obstacles[j]->get_ID() == obstacle_states(8, i))
			{
				old_obstacles[j]->update(obstacle_states.block<4, 1>(0, i));

				new_obstacles.push_back(std::move(old_obstacles[j]));

				obstacle_exist = true;

				break;
			}
		}
		if (!obstacle_exist)
		{
			new_obstacles.push_back(std::move(std::unique_ptr<Prediction_Obstacle>(new Prediction_Obstacle(
				obstacle_states.col(i), obstacle_covariances.col(i), 
				obstacle_colav_on, 
				T, 
				dt))));
		}
	}

	// Clear old obstacle vector, which includes transferred obstacles and terminated obstacles
	// Then set equal to the new obstacle vector
	old_obstacles.resize(new_obstacles.size());
	for (int i = 0; i < new_obstacles.size(); i++)
	{
		old_obstacles[i].reset(new Prediction_Obstacle(*(new_obstacles[i])));
	}
}

/****************************************************************************************
*  Name     : update_obstacle_status
*  Function : Updates various information on each obstacle at the current time
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::update_obstacle_status(
	Eigen::Matrix<double,-1,-1> &obstacle_status,							// In/out: Various information on obstacles
	const Eigen::VectorXd &HL_0 											// In: relative (to total hazard) hazard level of each obstacle
	)
{
	int n_obst = new_obstacles.size();
	obstacle_status.resize(13, n_obst);
	double ID_0, RB_0, COG_0, SOG_0; 
	Eigen::Vector2d d_0i;
	Eigen::Vector4d xs_i;
	for(int i = 0; i < n_obst; i++)
	{
		xs_i = new_obstacles[i]->get_state();

		ID_0 = new_obstacles[i]->get_ID();
		
		d_0i = (xs_i.block<2, 1>(0, 0) - trajectory.block<2, 1>(0, 0));

		COG_0 = atan2(xs_i(3), xs_i(2));

		SOG_0 = xs_i.block<2, 1>(2, 0).norm();

		RB_0 = angle_difference_pmpi(atan2(d_0i(1), d_0i(0)), trajectory(2, 0));

		obstacle_status.col(i) << ID_0, 											// Obstacle ID
								  SOG_0, 											// Speed over ground of obstacle
								  wrap_angle_to_02pi(COG_0) * RAD2DEG, 				// Course over ground of obstacle
								  RB_0 * RAD2DEG, 									// Relative bearing
								  d_0i.norm(),										// Range
								  HL_0[i], 											// Hazard level of obstacle at optimum
								  IP_0[i], 											// If obstacle is passed by or not 
								  AH_0[i], 											// If obstacle is ahead or not
								  S_TC_0[i], 										// If obstacle is starboard or not
								  H_TC_0[i],										// If obstacle is head on or not
								  X_TC_0[i],										// If crossing situation or not
								  Q_TC_0[i],										// If obstacle overtakes ownship or not
								  O_TC_0[i];										// If ownship overtakes obstacle or not
	}
}

/****************************************************************************************
*  Name     : update_transitional_variables
*  Function : Updates the transitional cost indicators O, Q, X, S 
*			  at the current time t0 wrt all obstacles.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::update_transitional_variables()
{
	Eigen::Vector4d xs = trajectory.col(0);
	bool is_close;

	// A : Own-ship, B : Obstacle i
	Eigen::Vector2d v_A, v_B, L_AB;
	double psi_A, psi_B, d_AB;
	psi_A = xs[2]; // Crab angle neglected, thus chi = psi
	v_A(0) = xs(3) * cos(psi_A);
	v_A(1) = xs(3) * sin(psi_A);
	
	int n_obst = new_obstacles.size();
	
	AH_0.resize(n_obst);   S_TC_0.resize(n_obst); S_i_TC_0.resize(n_obst); 
	O_TC_0.resize(n_obst); Q_TC_0.resize(n_obst); IP_0.resize(n_obst); 
	H_TC_0.resize(n_obst); X_TC_0.resize(n_obst);

	for (int i = 0; i < n_obst; i++)
	{
		v_B(0) = new_obstacles[i]->get_state()(2);
		v_B(1) = new_obstacles[i]->get_state()(3);
		psi_B = atan2(v_B(1), v_B(0));

		L_AB(0) = new_obstacles[i]->get_state()(0) - xs(0);
		L_AB(1) = new_obstacles[i]->get_state()(1) - xs(1);
		d_AB = L_AB.norm();
		L_AB = L_AB / L_AB.norm();

		/*********************************************************************
		* Transitional variable update
		*********************************************************************/
		is_close = d_AB <= d_close;

		AH_0[i] = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

		//std::cout << "Obst i = " << i << " ahead at t0 ? " << AH_0[i] << std::endl;
		
		// Obstacle on starboard side
		S_TC_0[i] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

		//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << S_TC_0[i] << std::endl;

		// Ownship on starboard side of obstacle
		S_i_TC_0[i] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

		//std::cout << "Own-ship on starboard side of obst i = " << i << " at t0 ? " << S_i_TC_0[i] << std::endl;

		// Ownship overtaking the obstacle
		O_TC_0[i] = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
			  	v_B.norm() < v_B.norm()							    		&&
				v_B.norm() > 0.25											&&
				is_close 													&&
				AH_0[i];

		//std::cout << "Own-ship overtaking obst i = " << i << " at t0 ? " << O_TC_0[i] << std::endl;

		// Obstacle overtaking the ownship
		Q_TC_0[i] = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() < v_B.norm()							  			&&
				v_A.norm() > 0.25 											&&
				is_close 													&&
				!AH_0[i];

		//std::cout << "Obst i = " << i << " overtaking the ownship at t0 ? " << Q_TC_0[i] << std::endl;

		// Determine if the obstacle is passed by
		IP_0[i] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
				!Q_TC_0[i])		 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Obstacle's perspective	
				!O_TC_0[i]))		 										&&
				d_AB > d_safe;
		
		//std::cout << "Obst i = " << i << " passed by at t0 ? " << IP_0[i] << std::endl;

		// This is not mentioned in article, but also implemented here..				
		H_TC_0[i] = v_A.dot(v_B) < - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25											&&
				AH_0[i];
		
		//std::cout << "Head-on at t0 wrt obst i = " << i << " ? " << H_TC_0[i] << std::endl;

		// Crossing situation, a bit redundant with the !is_passed condition also, 
		// but better safe than sorry (could be replaced with B_is_ahead also)
		X_TC_0[i] = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()	&&
				!H_TC_0[i]													&&
				!O_TC_0[i] 													&&
				!Q_TC_0[i] 	 												&&
				!IP_0[i]													&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25;

		//std::cout << "Crossing at t0 wrt obst i = " << i << " ? " << X_TC_0[i] << std::endl;
	}
}