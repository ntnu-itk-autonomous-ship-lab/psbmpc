/****************************************************************************************
*
*  File name : obstacle_manager.cpp
*
*  Function  : Class functions for the obstacle management interface
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
#include "obstacle_manager.h"
#include <iostream>
#include <iomanip>

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
*  Name     : Class constructor
*  Function : 
*  Author   :
*  Modified :
*****************************************************************************************/
Obstacle_Manager::Obstacle_Manager()
{	
	T_lost_limit = 15.0; 	// 15.0 s obstacle no longer relevant after this time
	T_tracked_limit = 15.0; // 15.0 s obstacle still relevant if tracked for so long, choice depends on survival rate

	obstacle_filter_on = false;
}

/****************************************************************************************
*  Name     : update_obstacle_status
*  Function : Updates various information on each obstacle at the current time
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::update_obstacle_status(
	const Eigen::Matrix<double, 6, 1> &ownship_state						// In: Current time own-ship state
	)
{
	int n_obst = data.new_obstacles.size();
	data.obstacle_status.resize(13, n_obst);
	double ID_0, RB_0, COG_0, SOG_0; 
	Eigen::Vector2d d_0i;
	Eigen::Vector4d xs_i;
	for(int i = 0; i < n_obst; i++)
	{
		xs_i = data.new_obstacles[i].kf->get_state();

		ID_0 = data.new_obstacles[i].get_ID();
		
		d_0i = (xs_i.block<2, 1>(0, 0) - ownship_state.block<2, 1>(0, 0));

		COG_0 = atan2(xs_i(3), xs_i(2));

		SOG_0 = xs_i.block<2, 1>(2, 0).norm();

		RB_0 = angle_difference_pmpi(atan2(d_0i(1), d_0i(0)), ownship_state(2));

		data.obstacle_status.col(i) << ID_0, 											// Obstacle ID
								  SOG_0, 												// Speed over ground of obstacle
								  wrap_angle_to_02pi(COG_0) * RAD2DEG, 					// Course over ground of obstacle
								  RB_0 * RAD2DEG, 										// Relative bearing
								  d_0i.norm(),											// Range
								  data.HL_0[i], 										// Hazard level of obstacle at optimum
								  data.IP_0[i], 										// If obstacle is passed by or not 
								  data.AH_0[i], 										// If obstacle is ahead or not
								  data.S_TC_0[i], 										// If obstacle is starboard or not
								  data.H_TC_0[i],										// If obstacle is head on or not
								  data.X_TC_0[i],										// If crossing situation or not
								  data.O_TC_0[i],										// If ownship overtakes obstacle or not
								  data.Q_TC_0[i];										// If obstacle overtakes ownship or not
	}
}

/****************************************************************************************
*  Name     : display_obstacle_information
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::display_obstacle_information() 			
{
	std::cout << "Obstacle information:" << std::endl;
	std::cout << "ID   SOG   COG   R-BRG   RNG   HL   IP   AH   SB   HO   CRG   OTG   OT" << std::endl;
	for (size_t i = 0; i < data.new_obstacles.size(); i++)
	{
		std::cout << std::setw(4) << data.obstacle_status.col(i).transpose() << std::endl;
	}
}

/****************************************************************************************
*  Name     : operator()
*  Function : Manages new obstacle information, updates data structures and situation
*			  information accordingly 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::operator()(
	const PSBMPC_Parameters &psbmpc_pars,												// In: Parameters of the obstacle manager boss: PSBMPC
	const Eigen::Matrix<double, 6, 1> &ownship_state,									// In: Current time own-ship state
	const double ownship_length,														// In: Dimension of ownship along longest axis
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
	const Eigen::MatrixXd &obstacle_intention_probabilities, 							// In: Obstacle intention probability information
	const Eigen::VectorXd &obstacle_a_priori_CC_probabilities 							// In: Obstacle a priori COLREGS compliance probabilities
	) 			
{
	update_obstacles(psbmpc_pars, obstacle_states, obstacle_covariances, obstacle_intention_probabilities, obstacle_a_priori_CC_probabilities);

	update_situation_type_and_transitional_variables(psbmpc_pars, ownship_state, ownship_length);
}

/****************************************************************************************
	Private functions
****************************************************************************************/
/****************************************************************************************
*  Name     : determine_situation_type
*  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::determine_situation_type(
	ST& st_A,																// In/out: Situation type of vessel A
	ST& st_B,																// In/out: Situation type of vessel B
	const PSBMPC_Parameters &psbmpc_pars, 									// In: Parameters of the obstacle manager boss: PSBMPC		
	const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A 
	const double psi_A, 													// In: Heading of vessel A
	const Eigen::Vector2d &v_B, 											// In: (NE) Velocity vector of vessel B
	const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
	const double d_AB 														// In: Distance from vessel A to vessel B
	)
{
	// Crash situation, assume reactive maneuvers like in an overtaking scenario
	if (d_AB < psbmpc_pars.d_safe) 
	{
		st_A = D; st_B = D; 
		return;
	} 
	// Outside consideration range
	else if(d_AB > psbmpc_pars.d_close)
	{
		st_A = A; st_B = A;
		return;
	} 
	// Inside consideration range
	else
	{
		bool B_is_starboard, A_is_overtaken, B_is_overtaken;
		bool is_ahead, is_passed, is_head_on, is_crossing;

		is_ahead = v_A.dot(L_AB) > cos(psbmpc_pars.phi_AH) * v_A.norm();

		A_is_overtaken = v_A.dot(v_B) > cos(psbmpc_pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  		&&
						v_A.norm() > 0.25;

		B_is_overtaken = v_B.dot(v_A) > cos(psbmpc_pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							  		&&
						v_B.norm() > 0.25;

		B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

		is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
					!A_is_overtaken) 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
					!B_is_overtaken)) 											&&
					d_AB > psbmpc_pars.d_safe;

		is_head_on = v_A.dot(v_B) < - cos(psbmpc_pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() > 0.25											&&
					v_B.norm() > 0.25											&&
					is_ahead;

		is_crossing = v_A.dot(v_B) < cos(psbmpc_pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
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
*  Name     : update_obstacles
*  Function : Takes in new obstacle information and updates the single obstacle data 
*			  structures.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle_Manager::update_obstacles(
	const PSBMPC_Parameters &psbmpc_pars,												// In: Parameters of the obstacle manager boss: PSBMPC
	const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
	const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
	const Eigen::MatrixXd &obstacle_intention_probabilities, 							// In: Obstacle intention probability information
	const Eigen::VectorXd &obstacle_a_priori_CC_probabilities 							// In: Obstacle a priori COLREGS compliance probabilities
	) 			
{
	// Clear "old" new obstacles before the update
	data.new_obstacles.clear();
	
	int n_obst_old = data.old_obstacles.size();
	int n_obst_new = obstacle_states.cols();

	bool obstacle_exist;
	for (int i = 0; i < n_obst_new; i++)
	{
		obstacle_exist = false;
		for (int j = 0; j < n_obst_old; j++)
		{
			if ((double)data.old_obstacles[j].get_ID() == obstacle_states(8, i))
			{
				data.old_obstacles[j].reset_duration_lost();

				data.old_obstacles[j].update(
					obstacle_states.col(i), 
					obstacle_covariances.col(i), 
					obstacle_intention_probabilities.col(i),
					obstacle_a_priori_CC_probabilities(i),
					obstacle_filter_on,
					psbmpc_pars.dt);

				data.new_obstacles.push_back(std::move(data.old_obstacles[j]));

				obstacle_exist = true;

				break;
			}
		}
		if (!obstacle_exist)
		{
			data.new_obstacles.push_back(std::move(Tracked_Obstacle(
				obstacle_states.col(i), 
				obstacle_covariances.col(i),
				obstacle_intention_probabilities.col(i), 
				obstacle_a_priori_CC_probabilities(i),
				obstacle_filter_on,  
				psbmpc_pars.T, 
				psbmpc_pars.dt)));
		}
	}
	// Keep terminated obstacles that may still be relevant, and compute duration lost as input to the cost of collision risk
	// Obstacle track may be lost due to sensor/detection failure, or the obstacle may go out of COLAV-target range
	// Detection failure will lead to the start (creation) of a new track (obstacle), typically after a short duration,
	// whereas an obstacle that is out of COLAV-target range may re-enter range with the same id.
	if (obstacle_filter_on)
	{
		for (size_t j = 0; j < data.old_obstacles.size(); j++)
		{
			data.old_obstacles[j].increment_duration_lost(psbmpc_pars.dt * psbmpc_pars.p_step);

			if (data.old_obstacles[j].get_duration_tracked() >= T_tracked_limit 	&&
				(data.old_obstacles[j].get_duration_lost() < T_lost_limit || data.old_obstacles[j].kf->get_covariance()(0,0) <= 5.0))
			{
				data.old_obstacles[j].update(obstacle_filter_on, psbmpc_pars.dt);

				data.new_obstacles.push_back(std::move(data.old_obstacles[j]));
			}
		}
	}
	// Clear old obstacle vector, which consist of transferred (nullptr) and terminated obstacles
	// Then set equal to the new obstacle vector
	data.old_obstacles.resize(data.new_obstacles.size());
	for (size_t i = 0; i < data.new_obstacles.size(); i++)
	{
		data.old_obstacles[i] = Tracked_Obstacle(data.new_obstacles[i]);
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
void Obstacle_Manager::update_situation_type_and_transitional_variables(
	const PSBMPC_Parameters &psbmpc_pars,							// In: Parameters of the obstacle manager boss: PSBMPC
	const Eigen::Matrix<double, 6, 1> &ownship_state,				// In: Current time own-ship state
	const double ownship_length										// In: Dimension of ownship along longest axis
	)
{
	bool is_close;

	// A : Own-ship, B : Obstacle i
	Eigen::Vector2d v_A, v_B, L_AB;
	double psi_A, psi_B, d_AB;
	v_A(0) = ownship_state(3);
	v_A(1) = ownship_state(4);
	psi_A = wrap_angle_to_pmpi(ownship_state(2));
	v_A = rotate_vector_2D(v_A, psi_A);

	int n_obst = data.new_obstacles.size();
	data.ST_0.resize(n_obst);   data.ST_i_0.resize(n_obst);
	
	data.AH_0.resize(n_obst);   data.S_TC_0.resize(n_obst); data.S_i_TC_0.resize(n_obst); 
	data.O_TC_0.resize(n_obst); data.Q_TC_0.resize(n_obst); data.IP_0.resize(n_obst); 
	data.H_TC_0.resize(n_obst); data.X_TC_0.resize(n_obst);

	//std::cout << "Situation types:: 0 : (ST = Ø), 1 : (ST = OT, SO), 2 : (ST = CR, SO), 3 : (ST = OT, GW), 4 : (ST = HO, GW), 5 : (ST = CR, GW)" << std::endl;
	//std::cout << A << std::endl;
	for (int i = 0; i < n_obst; i++)
	{
		v_B(0) = data.new_obstacles[i].kf->get_state()(2);
		v_B(1) = data.new_obstacles[i].kf->get_state()(3);
		psi_B = atan2(v_B(1), v_B(0));

		L_AB(0) = data.new_obstacles[i].kf->get_state()(0) - ownship_state(0);
		L_AB(1) = data.new_obstacles[i].kf->get_state()(1) - ownship_state(1);
		d_AB = L_AB.norm();

		// Decrease the distance between the vessels by their respective max dimension
		d_AB = d_AB - 0.5 * (ownship_length + data.new_obstacles[i].get_length()); 
		
		L_AB = L_AB.normalized();

		determine_situation_type(data.ST_0[i], data.ST_i_0[i], psbmpc_pars, v_A, psi_A, v_B, L_AB, d_AB);
		
		//std::cout << "Own-ship situation type wrt obst i = " << i << " ? " << ST_0[i] << std::endl;
		//std::cout << "Obst i = " << i << " situation type wrt ownship ? " << ST_i_0[i] << std::endl;

		/*********************************************************************
		* Transitional variable update
		*********************************************************************/
		is_close = d_AB <= psbmpc_pars.d_close;

		data.AH_0[i] = v_A.dot(L_AB) > cos(psbmpc_pars.phi_AH) * v_A.norm();

		//std::cout << "Obst i = " << i << " ahead at t0 ? " << AH_0[i] << std::endl;
		
		// Obstacle on starboard side
		data.S_TC_0[i] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

		//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << S_TC_0[i] << std::endl;

		// Ownship on starboard side of obstacle
		data.S_i_TC_0[i] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

		//std::cout << "Own-ship on starboard side of obst i = " << i << " at t0 ? " << S_i_TC_0[i] << std::endl;

		// Ownship overtaking the obstacle
		data.O_TC_0[i] = v_B.dot(v_A) > cos(psbmpc_pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
			  	v_B.norm() < v_B.norm()							    						&&
				v_B.norm() > 0.25															&&
				is_close 																	&&
				data.AH_0[i];

		//std::cout << "Own-ship overtaking obst i = " << i << " at t0 ? " << O_TC_0[i] << std::endl;

		// Obstacle overtaking the ownship
		data.Q_TC_0[i] = v_A.dot(v_B) > cos(psbmpc_pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() < v_B.norm()							  			&&
				v_A.norm() > 0.25 											&&
				is_close 													&&
				!data.AH_0[i];

		//std::cout << "Obst i = " << i << " overtaking the ownship at t0 ? " << Q_TC_0[i] << std::endl;

		// Determine if the obstacle is passed by
		data.IP_0[i] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
				!data.Q_TC_0[i])		 										||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 			&& // Obstacle's perspective	
				!data.O_TC_0[i]))		 										&&
				d_AB > psbmpc_pars.d_safe;
		
		std::cout << "Obst i = " << i << " passed by at t0 ? " << data.IP_0[i] << std::endl;

		// This is not mentioned in article, but also implemented here..				
		data.H_TC_0[i] = v_A.dot(v_B) < - cos(psbmpc_pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25																&&
				v_B.norm() > 0.25																&&
				data.AH_0[i];
		
		//std::cout << "Head-on at t0 wrt obst i = " << i << " ? " << H_TC_0[i] << std::endl;

		// Crossing situation, a bit redundant with the !is_passed condition also, 
		// but better safe than sorry (could be replaced with B_is_ahead also)
		data.X_TC_0[i] = v_A.dot(v_B) < cos(psbmpc_pars.phi_CR) * v_A.norm() * v_B.norm()	&&
				!data.H_TC_0[i]																&&
				!data.O_TC_0[i] 															&&
				!data.Q_TC_0[i] 	 														&&
				!data.IP_0[i]																&&
				v_A.norm() > 0.25															&&
				v_B.norm() > 0.25;

		//std::cout << "Crossing at t0 wrt obst i = " << i << " ? " << X_TC_0[i] << std::endl;
	}
}