/****************************************************************************************
*
*  File name : obstacle_sbmpc.cpp
*
*  Function  : Class functions for the SB-MPC used by obstacles in the PSB-MPC predictions
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


/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC::Obstacle_SBMPC() :
	pars(SBMPC_Parameters(true))
{
	offset_sequence_counter.resize(2 * pars.n_M);
	offset_sequence.resize(2 * pars.n_M);

	u_m_last = 1.0; chi_m_last = 0.0;

	min_cost = 1e12;
	
	obstacle_colav_on = false;
}

Obstacle_SBMPC::~Obstacle_SBMPC() = default;

/****************************************************************************************
*  Name     : Obstacle_SBMPC
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC::Obstacle_SBMPC(
	const Obstacle_SBMPC &o_sbmpc 									// In: Obstacle SBMPC to copy
	) :
	pars(SBMPC_Parameters(true))
{
	assign_data(o_sbmpc);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Obstacle_SBMPC& Obstacle_SBMPC::operator=(
	const Obstacle_SBMPC &rhs 									// In: Rhs Obstacle SBMPC to assign
	)
{
	if (this == &rhs)
	{
		return *this;
	}

	assign_data(rhs);

	return *this = Obstacle_SBMPC(rhs);
}

/****************************************************************************************
*  Name     : determine_COLREGS_violation
*  Function : Determine if vessel A violates COLREGS with respect to vessel B. 
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

	// Extra condition that the COLREGS violation is only considered in an annulus; i.e. within d_close but outside d_safe.
	// The logic behind having to be outside d_safe is that typically a collision happens here, and thus COLREGS should be disregarded
	// in order to make a safe reactive avoidance maneuver, if possible.  
	return is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken)) && (d_AB > pars.d_safe);
}

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 4, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::Vector4d &ownship_state, 									// In: Current ship state
	const Eigen::Matrix<double, 4, -1> &static_obstacles,					// In: Static obstacle information
	Obstacle_Data<Prediction_Obstacle> &data,								// In/Out: Dynamic obstacle information
	const int k_0 															// In: Index of the current (joint prediction) time t_k0
	)
{
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(4, n_samples);
	trajectory.col(0) = ownship_state;

	int n_obst = data.obstacles.size();
	int n_static_obst = static_obstacles.cols();

	Eigen::VectorXd opt_offset_sequence(2 * pars.n_M), cost_i(n_obst);
	data.HL_0.resize(n_obst); data.HL_0.setZero();

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1; 		u_m_last = u_opt;
		chi_opt = 0; 	chi_m_last = chi_opt;
		
		for (int M = 0; M < pars.n_M; M++)
		{
			opt_offset_sequence(2 * M) = 1.0; opt_offset_sequence(2 * M + 1) = 0.0;
		}
		maneuver_times.setZero();
		ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
		
		assign_optimal_trajectory(predicted_trajectory);

		return;
	}

	initialize_prediction(data, k_0);

	double cost;
	min_cost = 1e12;
	reset_control_behavior();
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cost = 0;

		//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
		ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

		for (int i = 0; i < n_obst; i++)
		{
			cost_i(i) = calculate_dynamic_obstacle_cost(data, i);
		}

		cost += cost_i.maxCoeff();
		//std::cout << "cost = " << cost << std::endl;
		//cost += calculate_grounding_cost(static_obstacles);

		cost += calculate_control_deviation_cost();

		//cost += calculate_chattering_cost();
		//std::cout << "cost = " << cost << std::endl;

		if (cost < min_cost) 
		{
			min_cost = cost;
			opt_offset_sequence = offset_sequence;

			assign_optimal_trajectory(predicted_trajectory);

			// Assign current optimal hazard level for each obstacle
			for (int i = 0; i < n_obst; i++)
			{
				data.HL_0(i) = cost_i(i) / cost_i.sum();
			}	
		}
		increment_control_behavior();
	}

	u_opt = opt_offset_sequence(0); 	u_m_last = u_opt;
	chi_opt = opt_offset_sequence(1); 	chi_m_last = chi_opt;

	if(u_opt == 0)
	{
		chi_opt = 0; 	chi_m_last = chi_opt;
	} 

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : assign_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::assign_data(
	const Obstacle_SBMPC &o_sbmpc 							// In: Obstacle_SBMPC whose data to assign to this
	)
{
	this->offset_sequence_counter = o_sbmpc.offset_sequence_counter;
	this->offset_sequence = o_sbmpc.offset_sequence;
	this->maneuver_times = o_sbmpc.maneuver_times;

	this->u_m_last = o_sbmpc.u_m_last;
	this->chi_m_last = o_sbmpc.chi_m_last;

	this->min_cost = o_sbmpc.min_cost;

	this->obstacle_colav_on = o_sbmpc.obstacle_colav_on;

	this->ownship = o_sbmpc.ownship;

	this->trajectory = o_sbmpc.trajectory;
}

/****************************************************************************************
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_SBMPC::initialize_prediction(
	Obstacle_Data<Prediction_Obstacle> &data,							// In: Dynamic obstacle information
	const int k_0														// In: Index of the current (joint prediction) time t_k0	 
	)
{
	int n_obst = data.obstacles.size();

	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa;
	for (int i = 0; i < n_obst; i++)
	{
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), data.obstacles[i].get_state(k_0));

		if (!obstacle_colav_on)
		{
			data.obstacles[i].predict_independent_trajectory(pars.T, pars.dt, k_0);
		}
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(pars.n_M);
	// First avoidance maneuver is always at t0
	maneuver_times(0) = 0;
	
	//std::cout << maneuver_times.transpose() << std::endl;
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
	for (int M = 0; M < pars.n_M; M++)
	{
		offset_sequence(2 * M) = pars.u_offsets[M](0);
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](0);
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
	for (int M = pars.n_M - 1; M > -1; M--)
	{
		// Only increment counter for "leaf node offsets" on each iteration, which are the
		// course offsets in the last maneuver
		if (M == pars.n_M - 1)
		{
			offset_sequence_counter(2 * M + 1) += 1;
		}

		// If one reaches the end of maneuver M's course offsets, reset corresponding
		// counter and increment surge offset counter above
		if (offset_sequence_counter(2 * M + 1) == pars.chi_offsets[M].size())
		{
			offset_sequence_counter(2 * M + 1) = 0;
			offset_sequence_counter(2 * M) += 1;
		}
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](offset_sequence_counter(2 * M + 1));

		// If one reaches the end of maneuver M's surge offsets, reset corresponding
		// counter and increment course offset counter above (if any)
		if (offset_sequence_counter(2 * M) == pars.u_offsets[M].size())
		{
			offset_sequence_counter(2 * M) = 0;
			if (M > 0)
			{
				offset_sequence_counter(2 * M - 1) += 1;
			}
		}
		offset_sequence(2 * M) = pars.u_offsets[M](offset_sequence_counter(2 * M));
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
	const Obstacle_Data<Prediction_Obstacle> &data,							// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Vector4d xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].get_initial_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].get_initial_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;
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
	const double chi_m, 													// In: Candidate course offset currently followed
	const Obstacle_Data<Prediction_Obstacle> &data,							// In: Dynamic obstacle information
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
	if (!data.S_TC_0[i]) { O_TC = data.O_TC_0[i] && S_TC; }
	else { O_TC = data.O_TC_0[i] && !S_TC; };

	// For obstacle overtaking the ownship: Check if ownship is on opposite side of 
	// obstacle to what was observed at t0
	if (!data.S_i_TC_0[i]) { Q_TC = data.Q_TC_0[i] && S_i_TC; }
	else { Q_TC = data.Q_TC_0[i] && !S_i_TC; };

	// For crossing: Check if obstacle is on opposite side of ownship to what was
	// observed at t0
	X_TC = data.X_TC_0[i] && data.S_TC_0[i] && S_TC && (chi_m < 0);

	// This is not mentioned in article, but also implemented here..
	// Transitional cost only valid by going from having obstacle on port side at
	// t0, to starboard side at time t
	if (!data.S_TC_0[i]) { H_TC = data.H_TC_0[i] && S_TC; }
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
	const Obstacle_Data<Prediction_Obstacle> &data,					// In: Dynamic obstacle information
	const int i 													// In: Index of obstacle
	)
{
	double cost(0.0), max_cost(0.0), C(0.0), R(0.0);

	int n_samples = trajectory.cols();

	Eigen::MatrixXd xs_i_p = data.obstacles[i].get_predicted_trajectory();

	Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
	double psi_0_p(0.0), psi_i_p(0.0), d_0i_p(0.0), chi_m(0.0);
	bool mu, trans;
	for(int k = 0; k < n_samples; k++)
	{
		psi_0_p = trajectory(2, k); 
		v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k)); 
		v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));

		// Determine active course modification at sample k
		for (int M = 0; M < pars.n_M; M++)
		{
			if (M < pars.n_M - 1)
			{
				if (k >= maneuver_times[M] && k < maneuver_times[M + 1])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
			else
			{
				if (k >= maneuver_times[M])
				{
					chi_m = offset_sequence[2 * M + 1];
				}
			}
		}

		L_0i_p = xs_i_p.block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
		d_0i_p = L_0i_p.norm();

		// Decrease the distance between the vessels by their respective max dimension
		d_0i_p = abs(d_0i_p - 0.5 * (ownship.get_length() + data.obstacles[i].get_length())); 

		L_0i_p = L_0i_p.normalized();

		v_i_p(0) = xs_i_p(2, k);
		v_i_p(1) = xs_i_p(3, k);
		psi_i_p = atan2(v_i_p(1), v_i_p(0));

		C = calculate_collision_cost(v_0_p, v_i_p);

		mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

		trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, chi_m, data, i);

		R = calculate_ad_hoc_collision_risk(d_0i_p, (k + 1) * pars.dt);

		// SB-MPC formulation with ad-hoc collision risk
		cost = C * R + pars.kappa * mu  + pars.kappa_TC * trans;

		if (cost > max_cost)
		{
			max_cost = cost;
		}
		
		/* if (cost > 5000)
		{
			std::cout << "v_0_p = " << v_0_p.transpose() << std::endl;
			std::cout << "v_i_p = " << v_i_p.transpose() << std::endl;

			std::cout << "d_0i_p = " << d_0i_p << std::endl;
			std::cout << "psi_0_p = " << psi_0_p << std::endl;
			std::cout << "psi_i_p = " << psi_i_p << std::endl;

			std::cout << "C = " << C << std::endl;
			std::cout << "mu = " << mu << std::endl;
			std::cout << "trans = " << trans << std::endl;
			std::cout << "R = " << R << std::endl;
			std::cout << "cost = " << cost << std::endl;
			std::cout << "..." << std::endl;
		}	 */	
	}
	return max_cost;
}

/****************************************************************************************
*  Name     : calculate_ad_hoc_collision_risk
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
double Obstacle_SBMPC::calculate_ad_hoc_collision_risk(
	const double d_AB, 												// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
																	// 	   reduced by half the length of the two vessels
	const double t 													// In: Prediction time t > t0 (= 0)
	)
{
	double R = 0;
	if (d_AB <= pars.d_safe)
	{
		assert(t > 0);
		R = pow(pars.d_safe / d_AB, pars.q) * (1 / pow(abs(t), pars.p)); 
		assert(t > 0);
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
	for (int i = 0; i < pars.n_M; i++)
	{
		if (i == 0)
		{
			cost += pars.K_u * (1 - offset_sequence[0]) + Delta_u(offset_sequence[0], u_m_last) +
				    K_chi(offset_sequence[1])      + Delta_chi(offset_sequence[1], chi_m_last);
		}
		else
		{
			cost += pars.K_u * (1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
				    K_chi(offset_sequence[2 * i + 1])  + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
		}
	}
	return cost / (double)pars.n_M;
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

	if (pars.n_M > 1) 
	{
		double delta_t = 0;
		for(int M = 0; M < pars.n_M; M++)
		{
			if (M < pars.n_M - 1)
			{
				if ((offset_sequence(M + 1) >= 0 && offset_sequence(2 * M + 1) < 0) ||
					(offset_sequence(M + 1) < 0 && offset_sequence(2 * M + 1) >= 0))
				{
					delta_t = maneuver_times(M + 1) - maneuver_times(M);
					cost += pars.K_sgn * exp( - delta_t / pars.T_sgn);
				}
			}
		}
	}
	return cost / (double)(pars.n_M - 1);
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
	Eigen::Matrix<double, 4, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = round(pars.T / pars.dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (false) //pars.prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(4, n_samples / pars.p_step);
		for (int k = 0; k < n_samples; k += pars.p_step)
		{
			optimal_trajectory.col(count) = trajectory.col(k);
			if (count < round(n_samples / pars.p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(4, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 4, n_samples);
	}
}