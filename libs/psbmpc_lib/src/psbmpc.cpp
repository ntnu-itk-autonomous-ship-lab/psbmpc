/****************************************************************************************
*
*  File name : psb_mpc.h
*
*  Function  : Class functions for Probabilistic Scneario-based Model Predictive Control
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
#include "psbmpc.h"
#include "iostream"

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
PSBMPC::PSBMPC(){

	// Initialize parameters before parameter limits, as some limits depend on the
	// parameter values set.

	initialize_pars();
	
	initialize_par_limits();

	ownship = new Ownship();

}

/****************************************************************************************
*  Name     : PSBMPC~
*  Function : Class destructor
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::~PSBMPC(){
	delete ownship;
}

/****************************************************************************************
*  Name     : get_<type>par
*  Function : Returns parameter with index <index>, overloaded for different data types
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
int PSBMPC::get_ipar(
	const int index															// In: Index of parameter to return (Must be of int type)
	) const
{
	switch(index){
		case i_ipar_n_M 				: return n_M; 

		default : std::cout << "Wrong index given" << std::endl;
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
		case i_dpar_t_ts 				: return t_ts;
		case i_dpar_d_safe 				: return d_safe;
		case i_dpar_d_close 			: return d_close;
		case i_dpar_K_coll 				: return K_coll;
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
		case i_dpar_phi_AH 				: return phi_AH;
		case i_dpar_phi_OT 				: return phi_OT;
		case i_dpar_phi_HO 				: return phi_HO;
		case i_dpar_phi_CR 				: return phi_CR;
		case i_dpar_T_lost_limit		: return T_lost_limit;
		case i_dpar_T_tracked_limit		: return T_tracked_limit;

		default : std::cout << "Wrong index given" << std::endl;
	}
}

std::vector<Eigen::VectorXd> PSBMPC::get_mpar(
	const int index															// In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
{
	switch (index){
		case i_mpar_u_offsets			: return u_offsets; 
		case i_mpar_chi_offsets 		: return chi_offsets;

		default : std::cout << "Wrong index given" << std::endl;  
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
			case i_ipar_n_M : n_M = value; 

			default : std::cout << "Wrong index given" << std::endl;
		}
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
			case i_dpar_T 					: T = value;
			case i_dpar_T_static 			: T_static = value;
			case i_dpar_dt 					: dt = value;
			case i_dpar_p_step 				: p_step = value; 
			case i_dpar_t_ts 				: t_ts = value;
			case i_dpar_d_safe :
			{
				// Limits on d_close and d_init depend on d_safe
				d_safe = value; 
				dpar_low[i_dpar_d_close] = d_safe;
				dpar_low[i_dpar_d_init] = d_safe;
			} 
			case i_dpar_d_close 			: d_close = value; 
			case i_dpar_d_init 				: d_init = value;
			case i_dpar_K_coll 				: K_coll = value;
			case i_dpar_kappa 				: kappa = value;
			case i_dpar_kappa_TC 			: kappa_TC = value;
			case i_dpar_K_u 				: K_u = value;
			case i_dpar_K_du 				: K_du = value;
			case i_dpar_K_chi_strb 			: K_chi_strb = value;
			case i_dpar_K_dchi_strb 		: K_dchi_strb = value;
			case i_dpar_K_chi_port 			: K_chi_port = value;
			case i_dpar_K_dchi_port 		: K_dchi_port = value;
			case i_dpar_G 					: G = value;
			case i_dpar_K_sgn 				: K_sgn = value;
			case i_dpar_T_sgn 				: T_sgn = value;
			case i_dpar_phi_AH 				: phi_AH = value;
			case i_dpar_phi_OT 				: phi_OT = value;
			case i_dpar_phi_HO 				: phi_HO = value;
			case i_dpar_phi_CR 				: phi_CR = value;
			case i_dpar_T_lost_limit		: T_lost_limit = value;
			case i_dpar_T_tracked_limit		: T_tracked_limit = value;

			default : std::cout << "Index invalid but makes it past limit checks? Update the index file or the parameters in the PSBMPC class.." << std::endl;
		}
	}
}

void PSBMPC::set_par(
	const int index,														// In: Index of parameter to set
	const std::vector<Eigen::VectorXd> value 								// In: Value to set for parameter
	)
{
	if (value.size() == n_M)
	{
		switch (index){
			case i_mpar_u_offsets : 
			{
				for (int j = 0; j < n_M; j++){
					if (value[j].size() > 0)
					{
						u_offsets[j] = value[j];
					}
				}
			}
			case i_mpar_chi_offsets : 
			{
				for (int j = 0; j < n_M; j++){
					if (value[j].size() > 0)
					{
						chi_offsets[j] = value[j];
					}
				}
			}
			default : std::cout << "Index invalid but makes it past limit checks? Update the index file or the parameters in the PSBMPC class.." << std::endl;  
		}
	}
	else{
		std::cout << "Update n_M first.." << std::endl;
	}
}

/****************************************************************************************
*  Name     : get_optimal_offsets
*  Function : Calculate optimal surge and course offsets for PSB-MPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// Out: Optimal surge offset
	double &chi_opt, 														// Out: Optimal course offset
	Eigen::Matrix<double, 2, -1> &predicted_trajectory,						// Out: Predicted optimal ownship trajectory
	Eigen::Matrix<double,-1,-1> &obstacle_status,							// Out: Status on obstacles
	Eigen::Matrix<double,-1, 1> &colav_status,								// Out: status on the COLAV system
	const double u_d, 														// In: Surge reference
	const double psi_d, 													// In: Heading reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::VectorXd &ownship_state, 									// In: Current ship state
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 					// In: Dynamic obstacle states 
	const std::vector<Eigen::Matrix<double, 4, 4>> &obstacle_covariances, 	// In: Dynamic obstacle covariances
	const Eigen::Matrix<double, 4, -1> &static_obstacles					// In: Static obstacle information
	)
{
	Eigen::VectorXd id_0, rb_0, d_0i, COG_0, SOG_0, CF_0, HL_0;


	update_obstacles(obstacle_states, obstacle_covariances);
	int n_obst = new_obstacles.size();
	int n_static_obst = static_obstacles.cols();

	bool colav_active = determine_colav_active(ownship_state, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		return;
	}
	
	update_transitional_variables(ownship_state);

	initialize_predictions();

	for (int i = 0; i < n_obst; i++)
	{
		new_obstacles[i]->predict_independent_trajectories(T, dt);
	}

	double cost, min_cost;
	Eigen::VectorXd opt_offset_sequence;
	min_cost = 1e10;
	reset_control_behavior();
	for (int cb = 0; cb < n_cbs; cb++)
	{
		// Predict own-ship trajectory jointly with dependent obstacle trajectories

		// calculate total cost with this control behavior
		//cost = calculate_total_cost();
		if (cost < min_cost) {
			min_cost = cost;
			opt_offset_sequence = offset_sequence;
		}

		increment_control_behavior();
	}

	obstacle_status.resize(13, n_obst);
	for(int i=0; i < n_obst; i++){
		obstacle_status.col(i) << id_0(i), SOG_0(i), COG_0(i) * RAD2DEG, rb_0(i) * RAD2DEG, d_0i(i), HL_0(i), IP_0(i), AH_0(i), S_TC_0(i), H_TC_0(i), X_TC_0(i), Q_TC_0(i), O_TC_0(i);
	}

	colav_status.resize(2,1);
	colav_status << CF_0, cost;
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
void PSBMPC::initialize_par_limits()
{
	ipar_low.resize(N_IPAR); ipar_high.resize(N_IPAR);
	for (int i = 0; i < N_IPAR; i++)
	{
		ipar_low[i] = 0.0;
		ipar_high[i] = 1e12;
	}
	ipar_low[i_ipar_n_M] = 1; ipar_high[i_ipar_n_M] = 5; 

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

	dpar_high[i_dpar_K_dchi_strb] = 1.0;
	dpar_high[i_dpar_K_dchi_port] = 1.0;

	dpar_low[i_dpar_phi_AH] = -180.0; 		dpar_high[i_dpar_phi_AH] = 180.0;
	dpar_low[i_dpar_phi_OT] = -180.0;		dpar_high[i_dpar_phi_AH] = 180.0;
	dpar_low[i_dpar_phi_HO] = -180.0; 		dpar_high[i_dpar_phi_AH] = 180.0;
	dpar_low[i_dpar_phi_CR] = -180.0; 		dpar_high[i_dpar_phi_AH] = 180.0;
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

	offset_sequence_counter.resize(2 * n_M);
	offset_sequence.resize(2 * n_M);

	reset_control_behavior();
	chi_offsets.resize(n_M);
	u_offsets.resize(n_M);
	for (int M = 0; M < n_M; M++)
	{
		if (M == 0)
		{
			u_offsets[M].resize(3);
			u_offsets[M] << 1.0, 0.5, 0.0;

			chi_offsets[M].resize(13);
			chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		} 
		else
		{
			u_offsets[M].resize(2);
			u_offsets[M] << 1.0, 0.5;

			chi_offsets[M].resize(6);
			chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
			chi_offsets[M] *= DEG2RAD;
		}
		n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
	}
	u_m_last = 1;
	chi_m_last = 0;

	cpe_method = CE;
	prediction_method = ERK1;
	guidance_method = LOS;

	T = 300.0; 	      // 400.0, 300.0, 240 (sim/Euler)
	dt = 5.0;		      // 5.0, 0.5 (sim/Euler)
  	T_static = 60.0;		  // (50.0)

	p_step = 1;
	if (prediction_method == ERK1)
	{ 
		dt = 0.5; 
		p_step = 10;
	}
	d_init = 700;							//1852.0;	  // should be >= D_CLOSE 300.0 600.0 500.0 700.0 800 1852
	d_close = 500;							//1000.0;	// 200.0 300.0 400.0 500.0 600 1000
	d_safe = 300; 							//185.2; 	  // 40.0, 50.0, 70.0, 80.0, 100, 200, 185.2
	K_coll = 1.0;		  					// 0.5, (1.0), (0.1), 0.5, (10.0), 100.0 ;  need 0.1 when K_P_=10.5!
	phi_AH = 68.5 * DEG2RAD;		 	
	phi_OT = 68.5 * DEG2RAD;		 		 
	phi_HO = 22.5 * DEG2RAD;		 		
	phi_CR = 68.5 * DEG2RAD;	     		
	kappa = 3.0;		  					
	kappa_TC = 100.0;						// (10.0) 100.0 
	K_u = 10.5;		   						 // (1.5), 2.5, (10.5) 100.5
	K_du = 0.5;		    					// 2.0, (0.5) (cost requires <1)
	K_chi_strb = 1.5;	  					// 1.5
	K_chi_port =  10.5;	  					// 100.5, (10.5), 5.5, 2.5, 1.5 (>1.5 red. flexibility to port)
	K_dchi_strb = 0.5;	 					 // 0.9, (0.5) 0.1 (cost requires <1)
	K_dchi_port = 0.9;	  					// 1.2, 0.9 (cost requires <1)
	G = 0;		         					 // 1.0e3

	obstacle_filter_on = false;
	T_lost_limit = 15.0; 	// 15.0 s obstacle no longer relevant after this time
	T_tracked_limit = 15.0; // 15.0 s obstacle still relevant if tracked for so long, choice depends on survival rate

	cost = 1e12;
}

/****************************************************************************************
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branches of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::reset_control_behavior()
{
	offset_sequence_counter.setZero();
	for (int M = 0; M < n_M - 1; M++)
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
void PSBMPC::increment_control_behavior()
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
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the ownship and obstacles with an active COLAV
*			  system
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::predict_trajectories_jointly()
{

}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the freshly updated new_obstacles vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_colav_active(
	const Eigen::Matrix<double, 6, 1> xs, 									// In: Ownship state
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (int i = 0; i < new_obstacles.size(); i++)
	{
		d_0i(0) = new_obstacles[i]->get_xs()(0) - xs(0);
		d_0i(1) = new_obstacles[i]->get_xs()(1) - xs(1);
		if (d_0i.norm() < d_init) colav_active = true;
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
}

/****************************************************************************************
*  Name     : determine_situation_type
*  Function : Determines the situation type for vessel A  \in {A, B, C, D, E, F}
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
ST PSBMPC::determine_situation_type(
	const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A (the ownship)
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d &v_B, 												// In: (NE) Velocity vector of vessel B (the obstacle)
	const double psi_B, 													// In: Heading of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	ST st = A;
}

/****************************************************************************************
*  Name     : determine_COLREGS_violation
*  Function : Determine if vessel A violates COLREGS with respect to vessel B.
*			  Two overloads
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_COLREGS_violation(
	const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A (the ownship)
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d &v_B, 												// In: (NE) Velocity vector of vessel B (the obstacle)
	const double psi_B, 													// In: Heading of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	bool B_is_ahead;
	bool A_is_starboard, B_is_starboard;
	bool A_is_overtaken, B_is_overtaken;
	bool is_close, is_passed, is_head_on, is_crossing;

	B_is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() &&
					 v_A.norm() < v_B.norm()							  &&
					 v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() &&
					 v_B.norm() < v_A.norm()							  &&
					 v_B.norm() > 0.25;

	B_is_starboard = atan2(L_AB(1), L_AB(0)) > psi_A;

	is_close = d_AB <= d_close;

	is_passed = (v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()	&& // Vessel A's perspective	
				!A_is_overtaken) 									||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() && // Vessel B's perspective	
				!B_is_overtaken) 									&&
				d_AB > d_safe;

	is_head_on = v_A.dot(v_B) > - cos(phi_HO) * v_A.norm() * v_B.norm() &&
				 v_A.norm() > 0.25										&&
				 v_B.norm() > 0.25										&&
				 B_is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  &&
				  v_A.norm() > 0.25										&&
				  v_B.norm() > 0.25										&&
				  !is_passed;

	return (is_close && B_is_starboard && is_head_on) || (is_close && B_is_starboard && is_crossing && !A_is_overtaken);
}

bool PSBMPC::determine_COLREGS_violation(
	const Eigen::VectorXd& xs_A,											// In: State vector of vessel A (most often the ownship)
	const Eigen::VectorXd& xs_B 											// In: State vector of vessel B (most often an obstacle)
	)
{
	bool B_is_ahead;
	bool A_is_starboard, B_is_starboard;
	bool A_is_overtaken, B_is_overtaken;
	bool is_close, is_passed, is_head_on, is_crossing;

	Eigen::Vector2d v_A, v_B, d_AB, L_AB;
	double psi_A, psi_B;
	if (xs_A.size() == 6) { psi_A = xs_A[2]; v_A(0) = xs_A(3); v_A(1) = xs_A(4); Utilities::rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B[2]; v_B(1) = xs_B(4); v_B(1) = xs_B(4); Utilities::rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	d_AB(0) = xs_B(0) - xs_A(0);
	d_AB(1) = xs_B(1) - xs_A(1);
	L_AB = d_AB / d_AB.norm();

	B_is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() &&
					 v_A.norm() < v_B.norm()							  &&
					 v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() &&
					 v_B.norm() < v_A.norm()							  &&
					 v_B.norm() > 0.25;

	B_is_starboard = atan2(L_AB(1), L_AB(0)) > psi_A;

	is_close = d_AB.norm() <= d_close;

	is_passed = (v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()	&& // Vessel A's perspective	
				!A_is_overtaken) 									||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() && // Vessel B's perspective	
				!B_is_overtaken) 									&&
				d_AB.norm() > d_safe;

	is_head_on = v_A.dot(v_B) > - cos(phi_HO) * v_A.norm() * v_B.norm() &&
				 v_A.norm() > 0.25										&&
				 v_B.norm() > 0.25										&&
				 B_is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  &&
				  v_A.norm() > 0.25										&&
				  v_B.norm() > 0.25										&&
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
bool PSBMPC::determine_transitional_cost_indicator(
	const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A (the ownship)
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d &v_B, 											// In: (NE) Velocity vector of vessel B (the obstacle)
	const double psi_B, 													// In: Heading of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const int i, 															// In: Index of obstacle
	const double chi_m 														// In: Candidate course offset currently followed
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;

	// Obstacle on starboard side
	S_TC = atan2(L_AB(1), L_AB(0)) > psi_A;

	// Ownship on starboard side of obstacle
	S_i_TC = atan2(-L_AB(1), -L_AB(0)) > psi_B;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!S_TC_0(i)) { O_TC = O_TC_0(i) && S_TC; }
	else { O_TC = O_TC_0(i) && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!S_i_TC_0(i)) { Q_TC = Q_TC_0(i) && S_i_TC; }
	else { Q_TC = Q_TC_0(i) && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = X_TC_0(i) && S_TC_0(i) && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!S_TC_0(i)) { H_TC = H_TC_0(i) && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}

bool PSBMPC::determine_transitional_cost_indicator(
	const Eigen::VectorXd& xs_A,											// In: State vector of vessel A (the ownship)
	const Eigen::VectorXd& xs_B, 											// In: State vector of vessel B (the obstacle)
	const int i, 															// In: Index of obstacle
	const double chi_m 														// In: Candidate course offset currently followed
	)
{
	bool S_TC, S_i_TC, O_TC, Q_TC, X_TC, H_TC;
	double psi_A, psi_B;
	Eigen::Vector2d v_A, v_B, d_AB, L_AB;
	if (xs_A.size() == 6) { psi_A = xs_A[2]; v_A(0) = xs_A(3); v_A(1) = xs_A(4); Utilities::rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B[2]; v_B(1) = xs_B(4); v_B(1) = xs_B(4); Utilities::rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	d_AB(0) = xs_B(0) - xs_A(0);
	d_AB(1) = xs_B(1) - xs_A(1);
	L_AB = d_AB / d_AB.norm();

	// Obstacle on starboard side
	S_TC = atan2(L_AB(1), L_AB(0)) > psi_A;

	// Ownship on starboard side of obstacle
	S_i_TC = atan2(-L_AB(1), -L_AB(0)) > psi_B;

	// For ownship overtaking the obstacle: Check if obstacle is on opposite side of 
	// ownship to what was observed at t0
	if (!S_TC_0(i)) { O_TC = O_TC_0(i) && S_TC; }
	else { O_TC = O_TC_0(i) && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!S_i_TC_0(i)) { Q_TC = Q_TC_0(i) && S_i_TC; }
	else { Q_TC = Q_TC_0(i) && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = X_TC_0(i) && S_TC_0(i) && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!S_TC_0(i)) { H_TC = H_TC_0(i) && S_TC; }
	else { H_TC = false; }
	H_TC = H_TC && !X_TC;

	return O_TC || Q_TC || X_TC || H_TC;
}


/****************************************************************************************
*  Name     : update_transitional_variables
*  Function : Updates the transitional cost indicators O, Q, X, S at the current time t0
*			  wrt obstacle i. Two overloads
*  Author   : 
*  Modified :
*****************************************************************************************/
void PSBMPC::update_transitional_variables(
	const Eigen::Matrix<double, 6, 1>& xs 									// In: Ownship state
	)
{
	bool is_close;

	Eigen::Vector2d v_A, v_B, d_AB, L_AB;
	double psi_A, psi_B;
	v_A(0) = xs(3);
	v_A(1) = xs(4);
	double psi_A = Utilities::wrap_angle_to_pmpi(xs[2]);
	Utilities::rotate_vector_2D(v_A, psi_A);

	int n_obst = new_obstacles.size();
	AH_0.resize(n_obst);   S_TC_0.resize(n_obst); S_i_TC_0.resize(n_obst); 
	O_TC_0.resize(n_obst); Q_TC_0.resize(n_obst); IP_0.resize(n_obst); 
	H_TC_0.resize(n_obst); X_TC_0.resize(n_obst);

	for (int i = 0; i < n_obst; i++)
	{
		v_B(0) = new_obstacles[i]->get_xs()(2);
		v_B(1) = new_obstacles[i]->get_xs()(3);
		psi_B = atan2(v_B(1), v_B(0));

		d_AB(0) = new_obstacles[i]->get_xs()(0) - xs(0);
		d_AB(1) = new_obstacles[i]->get_xs()(1) - xs(1);
		L_AB = d_AB / d_AB.norm();

		is_close = d_AB.norm() <= d_close;

		AH_0(i) = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

		// Obstacle on starboard side
		S_TC_0(i) = atan2(L_AB(1), L_AB(0)) > psi_A;

		// Ownship on starboard side of obstacle
		S_i_TC_0(i) = atan2(-L_AB(1), -L_AB(0)) > psi_B;

		// Ownship overtaking the obstacle
		O_TC_0(i) = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
			  	v_B.norm() < v_B.norm()							    		&&
				v_B.norm() > 0.25											&&
				is_close 													&&
				AH_0(i);

		// Obstacle overtaking the ownship
		Q_TC_0(i) = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() < v_B.norm()							  			&&
				v_A.norm() > 0.25 											&&
				is_close 													&&
				!AH_0(i);

		// Determine if the obstacle is passed by
		IP_0(i) = (v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
				!Q_TC_0(i))		 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Obstacle's perspective	
				!O_TC_0(i))		 											&&
				d_AB.norm() > d_safe;
		
		// This is not mentioned in article, but also implemented here..				
		H_TC_0(i) = v_A.dot(v_B) > - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25											&&
				AH_0(i);
		
		// Crossing situation, a bit redundant with the !is_passed condition also, 
		// but better safe than sorry (could be replaced with B_is_ahead also)
		X_TC_0(i) = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()	&&
				!O_TC_0(i) 													&&
				!Q_TC_0(i) 													&&
				!IP_0(i);
	}
}

/****************************************************************************************
*  Name     : calculate_total_cost
*  Function : Calculates total hazard, i.e. cost function H(t_0)
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double PSBMPC::calculate_total_cost(
	const Eigen::VectorXd offset_sequence, 
	const Eigen::VectorXd maneuver_times, 
	const Eigen::Matrix<double, 6, -1> os_trajectory,
	const std::vector<Eigen::MatrixXd> obstacle_trajectories,

	const int k 
	)
{
	double cost = 0;

	cost += calculate_control_deviation_cost(offset_sequence);
	cost += calculate_grounding_cost();

	return cost;
}

/****************************************************************************************
*  Name     : calculate_collision_cost
*  Function : Overloaded for 2 x 1 and 2 x n_samples inputs
*  Author   : 
*  Modified :
*****************************************************************************************/
double PSBMPC::calculate_collision_cost(
	const Eigen::Vector2d v_1, 												// In: Velocity v_1
	const Eigen::Vector2d v_2 												// In: Velocity v_2
	)
{
	Eigen::Vector2d v_diff = v_1 - v_2;
	return K_coll * v_diff.norm();
}

void PSBMPC::calculate_collision_cost(
	Eigen::VectorXd& cost,													// Out: Collision cost
	const Eigen::Matrix<double, 2, -1>& v_1, 										// In: Velocity v_1
	const Eigen::Matrix<double, 2, -1>& v_2 										// In: Velocity v_2
	)
{
	int n_samples = v_1.cols();
	cost.resize(n_samples);
	Eigen::MatrixXd v_diff = v_1 - v_2;
	for (int i = 0; i < n_samples; i++)
	{
		cost[i] = K_coll * v_diff.block<2, 1>(0, i).norm();
	}
}

/****************************************************************************************
*  Name     : calculate_control_deviation_cost
*  Function : Determines penalty due to using offsets to guidance references ++
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
double PSBMPC::calculate_control_deviation_cost(
	const Eigen::VectorXd &offset_sequence 									// In: Candidate control behavior [(u_m1, chi_m1), ..., (u_mn_M, chi_mn_M)]
	)
{
	double cost = 0;
	for (int i = 0; i < n_M; i++)
	{
		if (i == 0)
		{
			cost += K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], chi_m_last);
		}else{
			cost += K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	return cost / n_M;
}

/****************************************************************************************
*  Name     : calculate_grounding_cost
*  Function : Determines penalty due grounding ownship on static obstacles (no-go zones)
*  Author   : 
*  Modified :
*****************************************************************************************/
double PSBMPC::calculate_grounding_cost(
	const Eigen::Matrix<double, 2, -1> trajectory 									// In: Predicted ownship trajectory with control behavior k
	)
{
	double cost;

	return cost;
}

/****************************************************************************************
*  Name     : find_triplet_orientation
*  Function : Find orientation of ordered triplet (p, q, r)
*  Author   : Giorgio D. Kwame Minde Kufoalor
*  Modified :
*****************************************************************************************/
int PSBMPC::find_triplet_orientation(
	const Eigen::Vector2d p, 
	const Eigen::Vector2d q, 
	const Eigen::Vector2d r
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
bool PSBMPC::determine_if_on_segment(
	const Eigen::Vector2d p, 
	const Eigen::Vector2d q, 
	const Eigen::Vector2d r
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
bool PSBMPC::determine_if_behind(
	const Eigen::Vector2d p_1, 
	const Eigen::Vector2d v_1, 
	const Eigen::Vector2d v_2, 
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
bool PSBMPC::determine_if_lines_intersect(
	const Eigen::Vector2d p_1, 
	const Eigen::Vector2d q_1, 
	const Eigen::Vector2d p_2, 
	const Eigen::Vector2d q_2
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
double PSBMPC::distance_from_point_to_line(
	const Eigen::Vector2d p, 
	const Eigen::Vector2d q_1, 
	const Eigen::Vector2d q_2
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
double PSBMPC::distance_to_static_obstacle(
	const Eigen::Vector2d p, 
	const Eigen::Vector2d v_1, 
	const Eigen::Vector2d v_2
	)
{
    double d2line = distance_from_point_to_line(p, v_1, v_2);

    if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
    else return std::min((v_1-p).norm(),(v_2-p).norm());
}


/****************************************************************************************
*  Name     : update_obstacles
*  Function : Takes in new obstacle information and updates the obstacle data structures
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::update_obstacles(
	const Eigen::Matrix<double,-1,9>& obstacle_states, 								// In: Dynamic obstacle states 
	const std::vector<Eigen::Matrix<double, 4, 4>> &obstacle_covariances 			// In: Dynamic obstacle covariances
	) 			
{

	int n_obst_old = old_obstacles.size();
	int n_obst_new = obstacle_states.rows();

	bool obstacle_exist;
	for (int i = 0; i < n_obst_new; i++)
	{
		obstacle_exist = false;
		for (int j = 0; j < n_obst_old; j++)
		{
			if ((double)old_obstacles[j]->get_id() == obstacle_states(8, i))
			{
				old_obstacles[j]->reset_duration_lost();

				new_obstacles.push_back(old_obstacles[j]);

				obstacle_exist = true;
			}
		}
		if (!obstacle_exist)
		{
			Obstacle *obstacle = new Obstacle(obstacle_states.col(i), obstacle_covariances[i], obstacle_filter_on, T, dt);
			new_obstacles.push_back(obstacle);
		}
	}
	// Keep terminated obstacles that may still be relevant, and compute duration lost as input to the cost of collision risk
	// Obstacle track may be lost due to sensor/detection failure, or the obstacle may go out of COLAV-target range
	// Detection failure will lead to the start (creation) of a new track (obstacle), typically after a short duration,
	// whereas an obstacle that is out of COLAV-target range may re-enter range with the same id.
	if (obstacle_filter_on)
	{
		for (int j = 0; j < old_obstacles.size(); j++)
		{
			old_obstacles[j]->increment_duration_lost(dt * p_step);

			if (old_obstacles[j]->get_duration_tracked() >= T_tracked_limit &&
				(old_obstacles[j]->get_duration_lost() < T_lost_limit || old_obstacles[j]->get_P()(0,0) <= 5.0))
			{
				new_obstacles.push_back(old_obstacles[j]);
			}
		}
	}
	// Clear old obstacle vector, which includes transferred obstacles and terminated obstacles
	for (int i = 0; i < n_obst_old; i++)
	{
		delete old_obstacles[i];
	}
	old_obstacles.clear();
}