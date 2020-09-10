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

#include "utilities.h"
#include "psbmpc.h"
#include "cb_cost_functor.cuh"

#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0f
#endif
#ifndef RAD2DEG
#define RAD2DEG 180.0f / M_PI
#endif


/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC() 
	: 
	ownship(Ownship()), pars(PSBMPC_Parameters())
{
	cpe = CPE(pars.cpe_method, 1000, 100, 0, pars.dt);

	map_offset_sequences();
}

/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class deconstructor
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::~PSBMPC() = default;


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
	Eigen::VectorXd offset_sequence_counter(2 * pars.n_M), offset_sequence(2 * pars.n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);

	control_behaviours.resize(2 * pars.n_M, pars.n_cbs);
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		control_behaviours.col(cb) = offset_sequence;

		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
	std::cout << "Number of control behaviours: " << control_behaviours.cols() << std::endl;
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
void PSBMPC::increment_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
	)
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
*  Name     : initialize_predictions
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_prediction(
	Obstacle_Data &data														// In: Dynamic obstacle information
	)
{
	int n_obst = data.new_obstacles.size();
	cpe.set_number_of_obstacles(n_obst);
	n_ps.resize(n_obst);

	int n_a = data.new_obstacles[0].get_intention_probabilities().size();
	
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
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), data.new_obstacles[i].kf->get_state());
		//std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
		//std::cout << "t_cpa = " << t_cpa(i) << std::endl;
		//std::cout << "d_cpa = " << d_cpa(i) << std::endl;
		if (n_a == 1)
		{
			n_ps[i] = 1;
			ps_ordering_i.resize(1);
			if (!pars.obstacle_colav_on)	{ ps_ordering_i[0] = KCC; } // One intention: KCC for independent obstacle prediction
			else							{ ps_ordering_i[0] = SM;  } // and CC starboard maneuver for dependent obstacle prediction
			
			ps_course_changes_i.resize(1);
			ps_course_changes_i[0] = 0;
			ps_weights_i.resize(1);
			ps_weights_i(0)= 1;
			ps_maneuver_times_i.resize(1);
			ps_maneuver_times_i(0) = 0;
		}
		else
		{
			if (!pars.obstacle_colav_on)
			{
				// Space obstacle maneuvers evenly throughout horizon, depending on CPA configuration
				if (d_cpa(i) > pars.d_safe || (d_cpa(i) <= pars.d_safe && t_cpa(i) > pars.T)) // No predicted collision inside time horizon
				{
					n_turns = std::floor(pars.T / pars.t_ts);
				} 
				else  // Safety zone violation at CPA inside prediction horizon, as d_cpa <= d_safe				
				{
					if (t_cpa(i) > pars.t_ts)	{ n_turns = std::floor(t_cpa(i) / pars.t_ts); }
					else					{ n_turns = 1; }	
				}

				n_ps[i] = 1 + 2 * pars.obstacle_course_changes.size() * n_turns;
				set_up_independent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, n_turns, data, i);
			}
			else // Set up dependent obstacle prediction scenarios
			{
				n_ps[i] = 3;
				set_up_dependent_obstacle_prediction_variables(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i, data, i);
			}	
		}
		data.new_obstacles[i].initialize_prediction(ps_ordering_i, ps_course_changes_i, ps_weights_i, ps_maneuver_times_i);		
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	maneuver_times.resize(pars.n_M);
	// First avoidance maneuver is always at t0
	maneuver_times.setZero();

	double t_cpa_min, d_safe_i;
	std::vector<bool> maneuvered_by(n_obst);
	int index_closest;
	for (int M = 1; M < pars.n_M; M++)
	{
		// This is the solution so far if n_obst = 0. And also:
		// If a predicted collision occurs with the closest obstacle, avoidance maneuver 
		// M is taken right after the obstacle possibly maneuvers (which will be at t_0 + M * t_ts
		// if the independent obstacle prediction scheme is used), given that t_cpa > t_ts. 
		// If t_cpa < t_ts, the subsequent maneuver is taken at t_0 + M * t_ts + 1 anyways (simplification)
		maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);
		
		// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
		t_cpa_min = 1e10; index_closest = -1;
		for (int i = 0; i < n_obst; i++)
		{
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.new_obstacles[i].get_length());
			// For the current avoidance maneuver, determine which obstacle that should be
			// considered, i.e. the closest obstacle that is not already passed (which means
			// that the previous avoidance maneuver happened before CPA with this obstacle)
			if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min)
			{	
				t_cpa_min = t_cpa(i);
				index_closest = i;
			}	
		}

		if (index_closest != -1)
		{
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.new_obstacles[index_closest].get_width());
			// If no predicted collision,  avoidance maneuver M with the closest
			// obstacle (that is not passed) is taken at t_cpa_min
			if (d_cpa(index_closest) > d_safe_i)
			{
				std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
				maneuvered_by[index_closest] = true;
				maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
			}
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
	const int n_turns, 														// In: number of predicted turns for the obstacle 
	const Obstacle_Data &data,												// In: Dynamic obstacle information
	const int i 															// In: Index of obstacle in consideration
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

			ps_maneuver_times_i[ps] = turn_count * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}
		// Port maneuvers
		else
		{
			ps_ordering_i[ps] = PM;

			ps_maneuver_times_i[ps] = turn_count * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = - pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
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
	Pr_CC_i = data.new_obstacles[i].get_a_priori_CC_probability();
	switch(data.ST_i_0[i])
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
			t_obst_passed = find_time_of_passing(data, i);
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
					if (ps_maneuver_times_i(ps) * pars.dt < t_obst_passed - pars.t_ts)
					{
						ps_weights_i(ps) = Pr_CC_i;
					}							
				} 
			}
			break;
		default :
			std::cout << "This situation type does not exist" << std::endl;
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
	const Obstacle_Data &data,												// In: Dynamic obstacle information
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
	Pr_CC_i = data.new_obstacles[i].get_a_priori_CC_probability();
	switch(data.ST_i_0[i])
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
			std::cout << "This situation type does not exist" << std::endl;
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
	const Obstacle_Data &data,											// In: Dynamic obstacle information
	const int i 														// In: Index of relevant obstacle
	)
{
	double t_obst_passed(1e12), t, psi_A, d_AB;
	Eigen::VectorXd xs_A = trajectory.col(0);
	Eigen::VectorXd xs_B = data.new_obstacles[i].kf->get_state();
	Eigen::Vector2d p_A, p_B, v_A, v_B, L_AB;
	p_A(0) = xs_A(0); p_A(1) = xs_A(1); psi_A = xs_A(2);
	v_A(0) = xs_A(3); v_A(1) = xs_A(4); 
	v_A = rotate_vector_2D(v_A, psi_A);
	p_B(0) = xs_B(0); p_B(1) = xs_B(1);
	v_B(0) = xs_B(2); v_B(1) = xs_B(3); 

	bool A_is_overtaken, B_is_overtaken, is_passed;

	int n_samples = pars.T / pars.dt;
	for (int k = 0; k < n_samples; k++)
	{
		t = k * pars.dt;
		p_A = p_A + v_A * t;
		p_B = p_B + v_B * t;

		L_AB = p_B - p_A;
		d_AB = L_AB.norm();
		L_AB = L_AB.normalized();

		A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  		&&
						v_A.norm() > 0.25;

		B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							  		&&
						v_B.norm() > 0.25;

		is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
					!A_is_overtaken) 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
					!B_is_overtaken)) 											&&
					d_AB > pars.d_safe;
		
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
	const Obstacle_Data &data,												// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.new_obstacles.size(); i++)
	{
		d_0i(0) = data.new_obstacles[i].kf->get_state()(0) - xs(0);
		d_0i(1) = data.new_obstacles[i].kf->get_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;

		// If all obstacles are passed, even though inside colav range,
		// then no need for colav
		if (data.IP_0[i]) 	{ colav_active = false; }
		else 				{ colav_active = true; }
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
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
	int n_samples = std::round(pars.T / pars.dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (pars.prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(2, n_samples / pars.p_step);
		for (int k = 0; k < n_samples; k += pars.p_step)
		{
			optimal_trajectory.col(count) = trajectory.block<2, 1>(0, k);
			if (count < std::round(n_samples / pars.p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(2, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 2, n_samples);
	}
}