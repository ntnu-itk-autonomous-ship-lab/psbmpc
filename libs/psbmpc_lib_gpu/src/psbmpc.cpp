/****************************************************************************************
*
*  File name : psbmpc.cpp
*
*  Function  : C++ Class functions for Probabilistic Scenario-based Model Predictive Control
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

#include "utilities.cuh"
#include "cb_cost_functor.cuh"
#include "psbmpc.h"
#include "iostream"
#include "engine.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI


/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC() 
	: 
	ownship(new Ownship())
{
	// Initialize parameters before parameter limits, as some limits depend on the
	// parameter values set.
	initialize_pars();
	
	initialize_par_limits();

	cpe.reset(new CPE(cpe_method, 1000, 100, 0, dt));
}

/****************************************************************************************
*  Name     : get_<type>par
*  Function : Returns parameter with index <index>, "overloaded" for different data types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
int PSBMPC::get_ipar(
	const int index															// In: Index of parameter to return (Must be of int type)
	) const
{
	switch(index){
		case i_ipar_n_M 				: return n_M; 
		case i_ipar_n_a					: return n_a;

		default : { std::cout << "Wrong index given" << std::endl; return 0;}
	}
}
	
double PSBMPC::get_dpar(
	const int index															// In: Index of parameter to return (Must be of double type)
	) const
{
	switch(index){
		case i_dpar_T 					: return T;
		case i_dpar_T_static 			: return T_static;
		case i_dpar_dt 					: return dt;
		case i_dpar_p_step				: return p_step;
		case i_dpar_t_ts 				: return t_ts;
		case i_dpar_d_safe 				: return d_safe;
		case i_dpar_d_close 			: return d_close;
		case i_dpar_K_coll 				: return K_coll;
		case i_dpar_phi_AH 				: return phi_AH;
		case i_dpar_phi_OT 				: return phi_OT;
		case i_dpar_phi_HO 				: return phi_HO;
		case i_dpar_phi_CR 				: return phi_CR;
		case i_dpar_kappa 				: return kappa;
		case i_dpar_kappa_TC 			: return kappa_TC;
		case i_dpar_K_u 				: return K_u;
		case i_dpar_K_du 				: return K_du;
		case i_dpar_K_chi_strb 			: return K_chi_strb;
		case i_dpar_K_dchi_strb 		: return K_dchi_strb;
		case i_dpar_K_chi_port 			: return K_chi_port;
		case i_dpar_K_dchi_port 		: return K_dchi_port;
		case i_dpar_K_sgn 				: return K_sgn;
		case i_dpar_T_sgn 				: return T_sgn;
		case i_dpar_G					: return G;
		case i_dpar_q					: return q;
		case i_dpar_p					: return p;
		case i_dpar_T_lost_limit		: return T_lost_limit;
		case i_dpar_T_tracked_limit		: return T_tracked_limit;

		default : { std::cout << "Wrong index given" << std::endl; return 0; }
	}
}

std::vector<Eigen::VectorXd> PSBMPC::get_mpar(
	const int index															// In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
{
	switch (index){
		case i_mpar_u_offsets			: return u_offsets;
		case i_mpar_chi_offsets 		: return chi_offsets;

		default : 
		{ 
			std::cout << "Wrong index given" << std::endl; 
			std::vector<Eigen::VectorXd> bs;
			return bs; 
		}
	}
}

/****************************************************************************************
*  Name     : set_par
*  Function : Sets parameter with index <index> to value <value>, given that it is inside
*			  valid limits. Overloaded for different data types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_par(
	const int index, 														// In: Index of parameter to set
	const int value 														// In: Value to set for parameter
	)
{
	if (value >= ipar_low[index] && value <= ipar_high[index])
	{	
		switch(index)
		{
			case i_ipar_n_M 				: n_M = value; break;
			case i_ipar_n_a 				: n_a = value; break;

			default : std::cout << "Wrong index given" << std::endl; break;
		}
	}
	else
	{
		std::cout << "Non-valid parameter value!" << std::endl;
	}
	
}

void PSBMPC::set_par(
	const int index, 														// In: Index of parameter to set
	const double value 														// In: Value to set for parameter
	)
{
	if (value >= dpar_low[index] && value <= dpar_high[index])
	{
		switch(index){
			case i_dpar_T 					: T = value; break;
			case i_dpar_T_static 			: T_static = value; break;
			case i_dpar_dt 					: dt = value; break;
			case i_dpar_p_step 				: p_step = value; break;
			case i_dpar_t_ts 				: t_ts = value; break;
			case i_dpar_d_safe :
				// Limits on d_close and d_init depend on d_safe
				d_safe = value; 
				dpar_low[i_dpar_d_close] = d_safe;
				dpar_low[i_dpar_d_init] = d_safe;
				break;
			case i_dpar_d_close 			: d_close = value; break;
			case i_dpar_d_init 				: d_init = value; break;
			case i_dpar_K_coll 				: K_coll = value; break;
			case i_dpar_phi_AH 				: phi_AH = value; break;
			case i_dpar_phi_OT 				: phi_OT = value; break;
			case i_dpar_phi_HO 				: phi_HO = value; break;
			case i_dpar_phi_CR 				: phi_CR = value; break;
			case i_dpar_kappa 				: kappa = value; break;
			case i_dpar_kappa_TC 			: kappa_TC = value; break;
			case i_dpar_K_u 				: K_u = value; break;
			case i_dpar_K_du 				: K_du = value; break;
			case i_dpar_K_chi_strb 			: K_chi_strb = value; break;
			case i_dpar_K_dchi_strb 		: K_dchi_strb = value; break;
			case i_dpar_K_chi_port 			: K_chi_port = value; break;
			case i_dpar_K_dchi_port 		: K_dchi_port = value; break;
			case i_dpar_K_sgn 				: K_sgn = value; break;
			case i_dpar_T_sgn 				: T_sgn = value; break;
			case i_dpar_G 					: G = value; break;
			case i_dpar_q 					: q = value; break;
			case i_dpar_p 					: p = value; break;
			case i_dpar_T_lost_limit		: T_lost_limit = value; break;
			case i_dpar_T_tracked_limit		: T_tracked_limit = value; break;

			default : std::cout << "Index invalid but makes it past limit checks? Update the index file or the parameters in the PSBMPC class.." << std::endl; break;
		}
	}
	else
	{
		std::cout << "Non-valid parameter value!" << std::endl;
	}
	
}

void PSBMPC::set_par(
	const int index,														// In: Index of parameter to set
	const std::vector<Eigen::VectorXd> &value 								// In: Value to set for parameter
	)
{
	if (value.size() == n_M)
	{
		switch (index){
			case i_mpar_u_offsets : 
				for (int j = 0; j < n_M; j++){
					if (value[j].size() > 0)
					{
						u_offsets[j] = value[j];
					}
				}
				break;
			case i_mpar_chi_offsets : 
				for (int j = 0; j < n_M; j++)
				{
					if (value[j].size() > 0)
					{
						chi_offsets[j] = value[j];
					}
				}
				break;
			default : std::cout << "Index invalid but makes it past limit checks? Update the index file or the parameters in the PSBMPC class.." << std::endl; break; 
		}
	}
	else
	{
		std::cout << "Update n_M first.." << std::endl;
	}
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : map_offset_sequences
*  Function : Maps the currently set surge and course modifications into a matrix of 
*			  offset sequences or control behaviours
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::map_offset_sequences()
{
	Eigen::VectorXd offset_sequence_counter(2 * n_M), offset_sequence(2 * n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);

	control_behaviours.resize(2 * n_M, n_cbs);
	for (int cb = 0; cb < n_cbs; cb++)
	{
		control_behaviours.col(cb) = offset_sequence;

		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
	std::cout << "Control behaviours: " << std::endl;
	std::cout << control_behaviours << std::endl;
}

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branch of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::reset_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
)
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
void PSBMPC::increment_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
	)
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
*  Name     : initialize_par_limits
*  Function : Sets initial low and high limits on tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_par_limits()
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

	dpar_low[i_dpar_phi_AH] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_AH] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_OT] = -180.0 * DEG2RAD;			dpar_high[i_dpar_phi_OT] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_HO] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_HO] = 180.0 * DEG2RAD;
	dpar_low[i_dpar_phi_CR] = -180.0 * DEG2RAD; 		dpar_high[i_dpar_phi_CR] = 180.0 * DEG2RAD;

	//std::cout << "d_par_low = " << dpar_low.transpose() << std::endl;
	//std::cout << "d_par_high = " << dpar_high.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : initialize_pars
*  Function : Sets initial values for PSBMPC tuning parameters
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_pars()
{
	n_cbs = 1;
	n_M = 2;
	n_a = 1; // (original PSB-MPC/SB-MPC) or = 3 if intentions KCC, SM, PM are considered (PSB-MPC fusion article)
	n_ps.resize(1); // Determined by initialize_prediction();

	chi_offsets.resize(n_M);
	u_offsets.resize(n_M);
	for (int M = 0; M < n_M; M++)
	{
		if (M == 0)
		{
			u_offsets[M].resize(3);
			//u_offsets[M] << 1.0;
			u_offsets[M] << 1.0, 0.5, 0.0;

			chi_offsets[M].resize(13);
			//chi_offsets[M] << 0.0;
			//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		} 
		else
		{
			u_offsets[M].resize(2);
			u_offsets[M] << 1.0, 0.5;

			chi_offsets[M].resize(7);
			//chi_offsets[M] << 0.0;
			//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
			chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		}
		n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
	}
	
	map_offset_sequences();

	u_m_last = 1;
	chi_m_last = 0;

	obstacle_course_changes.resize(1);
	obstacle_course_changes << 30 * DEG2RAD; //60 * DEG2RAD, 90 * DEG2RAD;

	cpe_method = CE;
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
	t_ts = 50;

	d_init = 1500;								 
	d_close = 500;
	d_safe = 50; 							
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
	K_sgn = 5;
	T_sgn = 4 * t_ts;	  					
	G = 1e3;		         					 
	q = 4.0;
	p = 1.0;

	obstacle_filter_on = false;
	obstacle_colav_on.resize(1);
	obstacle_colav_on[0] = false;
	
	T_lost_limit = 15.0; 	// 15.0 s obstacle no longer relevant after this time
	T_tracked_limit = 15.0; // 15.0 s obstacle still relevant if tracked for so long, choice depends on survival rate

	min_cost = 1e10;

}

/****************************************************************************************
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_prediction()
{
	int n_obst = new_obstacles.size();
	cpe->set_number_of_obstacles(n_obst);
	n_ps.resize(n_obst);
	
	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	int n_turns;
	std::vector<Intention> ps_ordering_i;
	Eigen::VectorXd ps_course_changes_i;
	Eigen::VectorXd ps_weights_i;
	Eigen::VectorXd ps_maneuver_times_i;

	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa;
	for (int i = 0; i < n_obst; i++)
	{
		//Typically three intentions: KCC, SM, PM
		//std::cout << trajectory.col(0).transpose() << std::endl;
		//std::cout << new_obstacles[i]->kf->get_state() << std::endl;
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), new_obstacles[i]->kf->get_state());
		//std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
		//std::cout << "t_cpa = " << t_cpa(i) << std::endl;
		//std::cout << "d_cpa = " << d_cpa(i) << std::endl;
		if (n_a == 1)
		{
			n_ps[i] = 1;
			ps_ordering_i.resize(1);
			if (!obstacle_colav_on[i])	{ ps_ordering_i[0] = KCC; } // One intention: KCC for independent obstacle prediction
			else						{ ps_ordering_i[0] = SM;  } // and CC starboard maneuver for dependent obstacle prediction
			
			ps_course_changes_i.resize(1);
			ps_course_changes_i[0] = 0;
			ps_weights_i.resize(1);
			ps_weights_i(0)= 1;
			ps_maneuver_times_i.resize(1);
			ps_maneuver_times_i(0) = 0;
		}
		else
		{
			if (!obstacle_colav_on[i])
			{
				// Space obstacle maneuvers evenly throughout horizon, depending on CPA configuration
				if (d_cpa(i) > d_safe || (d_cpa(i) <= d_safe && t_cpa(i) > T)) // No predicted collision inside time horizon
				{
					n_turns = std::floor(T / t_ts);
				} 
				else  // Safety zone violation at CPA inside prediction horizon, as d_cpa <= d_safe				
				{
					if (t_cpa(i) > t_ts)	{ n_turns = std::floor(t_cpa(i) / t_ts); }
					else					{ n_turns = 1; }	
				}

				n_ps[i] = 1 + 2 * obstacle_course_changes.size() * n_turns;
				set_up_independent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, i, n_turns);
			}
			else // Set up dependent obstacle prediction scenarios
			{
				n_ps[i] = 3;
				set_up_dependent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, i);
			}	
		}
		new_obstacles[i]->initialize_prediction(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i);		
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(n_M);
	// First avoidance maneuver is always at t0
	maneuver_times(0) = 0;
	double d_cpa_min = 1e10, t_cpa_min = t_cpa(0);
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
	std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : set_up_independent_obstacle_prediction_variables
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_independent_obstacle_prediction_variables(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_weights_i, 											// In/out: Cost weights of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const int i, 															// In: Index of obstacle in consideration
	const int n_turns 														// In: number of predicted turns for the obstacle 
	)
{
	double Pr_CC_i, t_obst_passed;
	int turn_count, course_change_count;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC;
	ps_maneuver_times_i.resize(n_ps[i]);
	ps_maneuver_times_i[0] = 0;
	ps_course_changes_i.resize(n_ps[i]);
	ps_course_changes_i[0] = 0;
	turn_count = 0;
	course_change_count = 0;
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (n_ps[i] - 1) / 2 + 1)
		{
			ps_ordering_i[ps] = SM;

			ps_maneuver_times_i[ps] = turn_count * std::floor(t_ts / dt);

			ps_course_changes_i(ps) = obstacle_course_changes(course_change_count);
			if (++course_change_count == obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}
		// Port maneuvers
		else
		{
			ps_ordering_i[ps] = PM;

			ps_maneuver_times_i[ps] = turn_count * std::floor(t_ts / dt);

			ps_course_changes_i(ps) = - obstacle_course_changes(course_change_count);
			if (++course_change_count == obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}	
	}
	//std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
	std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl;
	// Determine prediction scenario cost weights based on situation type and correct behavior (COLREGS)
	ps_weights_i.resize(n_ps[i]);
	Pr_CC_i = new_obstacles[i]->get_a_priori_CC_probability();
	switch(ST_i_0[i])
	{
		case A : // Outside CC consideration zone
			ps_weights_i(0) = 1;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1;	
			}
			break;
		case B : // OT, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case C : // CR, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case D : // OT, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = Pr_CC_i;
			}
			break;
		case E : // HO, GW
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	

				// Starboard maneuvers, which are CC
				if (ps > 0 && ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;	
				}
			}
			break;
		case F : // CR, GW
			t_obst_passed = find_time_of_passing(i);
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					// Crossing obstacle prediction scenario gets COLREGS compliant weight Pr_CC_i
					// if the course change is COLREGS compliant and happens in time before
					// the own-ship is passed
					if (ps_maneuver_times_i(ps) * dt < t_obst_passed - t_ts)
					{
						ps_weights_i(ps) = Pr_CC_i;
					}							
				} 
			}
			break;
		default :
			// Throw
			break;
	}
	ps_weights_i = ps_weights_i / ps_weights_i.sum();
}

/****************************************************************************************
*  Name     : set_up_independent_obstacle_prediction_variables
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_dependent_obstacle_prediction_variables(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_weights_i, 											// In/out: Cost weights of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const int i 															// In: Index of obstacle in consideration
	)
{
	double Pr_CC_i;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC; 
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard and port maneuvers, respectively
		if (ps < (n_ps[i] - 1) / 2 + 1)		{ ps_ordering_i[ps] = SM; }
		else 								{ ps_ordering_i[ps] = PM; }	
	}
	ps_maneuver_times_i.resize(0);
	ps_course_changes_i.resize(0);

	ps_weights_i.resize(n_ps[i]);
	Pr_CC_i = new_obstacles[i]->get_a_priori_CC_probability();
	switch(ST_i_0[i])
	{
		case A : // Outside CC consideration zone
			ps_weights_i(0) = 1;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1;	
			}
			break;
		case B : // OT, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case C : // CR, SO
			ps_weights_i(0) = Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
			}
			break;
		case D : // OT, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = Pr_CC_i;
			}
			break;
		case E : // HO, GW
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;	
				// Starboard maneuvers, which are CC
				if (ps > 0 && ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;	
				}
			}
			break;
		case F : // CR, GW
			ps_weights_i(0) = 1 - Pr_CC_i;
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				ps_weights_i(ps) = 1 - Pr_CC_i;
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_weights_i(ps) = Pr_CC_i;						
				} 
			}
			break;
		default :
			// Throw
			break;
	}
	ps_weights_i = ps_weights_i / ps_weights_i.sum();
}


/****************************************************************************************
*  Name     : find_time_of_passing
*  Function : Finds the time when an obstacle is passed by the own-ship, assuming both 
*			  vessels keeps their current course
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double PSBMPC::find_time_of_passing(
	const int i 														// In: Index of relevant obstacle
	)
{
	double t_obst_passed(1e12), t, psi_A, d_AB;
	Eigen::VectorXd xs_A = trajectory.col(0);
	Eigen::VectorXd xs_B = new_obstacles[i]->kf->get_state();
	Eigen::Vector2d p_A, p_B, v_A, v_B, L_AB;
	p_A(0) = xs_A(0); p_A(1) = xs_A(1); psi_A = xs_A(2);
	v_A(0) = xs_A(3); v_A(1) = xs_A(4); 
	v_A = rotate_vector_2D(v_A, psi_A);
	p_B(0) = xs_B(0); p_B(1) = xs_B(1);
	v_B(0) = xs_B(2); v_B(1) = xs_B(3); 

	bool A_is_overtaken, B_is_overtaken, is_passed;

	int n_samples = T / dt;
	for (int k = 0; k < n_samples; k++)
	{
		t = k * dt;
		p_A = p_A + v_A * t;
		p_B = p_B + v_B * t;

		L_AB = p_B - p_A;
		d_AB = L_AB.norm();
		L_AB = L_AB.normalized();

		A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  		&&
						v_A.norm() > 0.25;

		B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							  		&&
						v_B.norm() > 0.25;

		is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
					!A_is_overtaken) 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
					!B_is_overtaken)) 											&&
					d_AB > d_safe;
		
		if (is_passed) 
		{
			t_obst_passed = t; 
			break;
		}
	}
	return t_obst_passed;
}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the freshly updated new_obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_colav_active(
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < new_obstacles.size(); i++)
	{
		d_0i(0) = new_obstacles[i]->kf->get_state()(0) - xs(0);
		d_0i(1) = new_obstacles[i]->kf->get_state()(1) - xs(1);
		if (d_0i.norm() < d_init) colav_active = true;

		// If all obstacles are passed, even though inside colav range,
		// then no need for colav
		if (IP_0[i]) 	{ colav_active = false; }
		else 			{ colav_active = true; }
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
}

/****************************************************************************************
*  Name     : determine_situation_type
*  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::determine_situation_type(
	ST& st_A,																// In/out: Situation type of vessel A
	ST& st_B,																// In/out: Situation type of vessel B
	const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A 
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d &v_B, 											// In: (NE) Velocity vector of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	// Crash situation, assume reactive maneuvers like in an overtaking scenario
	if (d_AB < d_safe) 
	{
		st_A = D; st_B = D; 
		return;
	} 
	// Outside consideration range
	else if(d_AB > d_close)
	{
		st_A = A; st_B = A;
		return;
	} 
	// Inside consideration range
	else
	{
		bool B_is_starboard, A_is_overtaken, B_is_overtaken;
		bool is_ahead, is_passed, is_head_on, is_crossing;

		is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

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

/****************************************************************************************
*  Name     : assign_optimal_trajectory
*  Function : Set the optimal trajectory to the current predicted trajectory
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::assign_optimal_trajectory(
	Eigen::Matrix<double, 2, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = std::round(T / dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(2, n_samples / p_step);
		for (int k = 0; k < n_samples; k+=p_step)
		{
			optimal_trajectory.col(count) = trajectory.block<2, 1>(0, k);
			if (count < std::round(n_samples / p_step) - 1) count++;					
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
void PSBMPC::update_obstacles(
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
	const Eigen::MatrixXd &obstacle_intention_probabilities, 							// In: Obstacle intention probability information
	const Eigen::VectorXd &obstacle_a_priori_CC_probabilities 							// In: Obstacle a priori COLREGS compliance probabilities
	) 			
{
	// Clear "old" new obstacles before the update
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
				old_obstacles[j]->reset_duration_lost();

				old_obstacles[j]->update(
					obstacle_states.col(i), 
					obstacle_covariances.col(i), 
					obstacle_intention_probabilities.col(i),
					obstacle_a_priori_CC_probabilities(i),
					obstacle_filter_on,
					dt);

				new_obstacles.push_back(std::move(old_obstacles[j]));

				obstacle_exist = true;

				break;
			}
		}
		if (!obstacle_exist)
		{
			new_obstacles.push_back(std::move(std::unique_ptr<Tracked_Obstacle>(new Tracked_Obstacle(
				obstacle_states.col(i), 
				obstacle_covariances.col(i),
				obstacle_intention_probabilities.col(i), 
				obstacle_a_priori_CC_probabilities(i),
				obstacle_filter_on, 
				false, 
				T, 
				dt))));
		}
	}
	// Keep terminated obstacles that may still be relevant, and compute duration lost as input to the cost of collision risk
	// Obstacle track may be lost due to sensor/detection failure, or the obstacle may go out of COLAV-target range
	// Detection failure will lead to the start (creation) of a new track (obstacle), typically after a short duration,
	// whereas an obstacle that is out of COLAV-target range may re-enter range with the same id.
	if (obstacle_filter_on)
	{
		for (size_t j = 0; j < old_obstacles.size(); j++)
		{
			old_obstacles[j]->increment_duration_lost(dt * p_step);

			if (	old_obstacles[j]->get_duration_tracked() >= T_tracked_limit 	&&
					(old_obstacles[j]->get_duration_lost() < T_lost_limit || old_obstacles[j]->kf->get_covariance()(0,0) <= 5.0))
			{
				old_obstacles[j]->update(obstacle_filter_on, dt);

				new_obstacles.push_back(std::move(old_obstacles[j]));
			}
		}
	}
	// Clear old obstacle vector, which consist of transferred (nullptr) and terminated obstacles
	// Then set equal to the new obstacle vector
	old_obstacles.resize(new_obstacles.size());
	for (size_t i = 0; i < new_obstacles.size(); i++)
	{
		old_obstacles[i].reset(new Tracked_Obstacle(*(new_obstacles[i])));
	}
}

/****************************************************************************************
*  Name     : update_obstacle_status
*  Function : Updates various information on each obstacle at the current time
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC::update_obstacle_status(
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
		xs_i = new_obstacles[i]->kf->get_state();

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
								  O_TC_0[i],										// If ownship overtakes obstacle or not
								  Q_TC_0[i];										// If obstacle overtakes ownship or not
	}
}

/****************************************************************************************
*  Name     : update_situation_type_and_transitional_variables
*  Function : Updates the situation type for the own-ship (wrt all obstacles) and
*			  obstacles (wrt own-ship) and the transitional cost indicators O, Q, X, S 
*			  at the current time t0 wrt all obstacles.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::update_situation_type_and_transitional_variables()
{
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool is_close;

	// A : Own-ship, B : Obstacle i
	Eigen::Vector2d v_A, v_B, L_AB;
	double psi_A, psi_B, d_AB;
	v_A(0) = xs(3);
	v_A(1) = xs(4);
	psi_A = wrap_angle_to_pmpi(xs[2]);
	v_A = rotate_vector_2D(v_A, psi_A);

	int n_obst = new_obstacles.size();
	ST_0.resize(n_obst);   ST_i_0.resize(n_obst);
	
	AH_0.resize(n_obst);   S_TC_0.resize(n_obst); S_i_TC_0.resize(n_obst); 
	O_TC_0.resize(n_obst); Q_TC_0.resize(n_obst); IP_0.resize(n_obst); 
	H_TC_0.resize(n_obst); X_TC_0.resize(n_obst);

	//std::cout << "Situation types:: 0 : (ST = Ø), 1 : (ST = OT, SO), 2 : (ST = CR, SO), 3 : (ST = OT, GW), 4 : (ST = HO, GW), 5 : (ST = CR, GW)" << std::endl;
	//std::cout << A << std::endl;
	for (int i = 0; i < n_obst; i++)
	{
		v_B(0) = new_obstacles[i]->kf->get_state()(2);
		v_B(1) = new_obstacles[i]->kf->get_state()(3);
		psi_B = atan2(v_B(1), v_B(0));

		L_AB(0) = new_obstacles[i]->kf->get_state()(0) - xs(0);
		L_AB(1) = new_obstacles[i]->kf->get_state()(1) - xs(1);
		d_AB = L_AB.norm();

		// Decrease the distance between the vessels by their respective max dimension
		d_AB = d_AB - 0.5 * (ownship->get_length() + std::max(new_obstacles[i]->get_length(), new_obstacles[i]->get_width())); 

		L_AB = L_AB.normalized();

		determine_situation_type(ST_0[i], ST_i_0[i], v_A, psi_A, v_B, L_AB, d_AB);
		
		//std::cout << "Own-ship situation type wrt obst i = " << i << " ? " << ST_0[i] << std::endl;
		//std::cout << "Obst i = " << i << " situation type wrt ownship ? " << ST_i_0[i] << std::endl;

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