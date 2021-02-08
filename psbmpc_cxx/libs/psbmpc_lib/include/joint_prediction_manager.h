/****************************************************************************************
*
*  File name : joint_prediction_manager.h
*
*  Function  : Header file for the prediction management interface used in the PSB-MPC
*			   when considering dynamic obstacles with their own deliberative COLAV
*			   systems.
*
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

#ifndef _JOINT_PREDICTION_MANAGER_H_
#define _JOINT_PREDICTION_MANAGER_H_

#include "prediction_obstacle.h"
#include "obstacle_manager.h"
#include "Eigen/Dense"
#include <vector>
#include <memory>
#include <string>

class Joint_Prediction_Manager
{
private:

	// Array of strings and precisions for the obstacle status states
	std::vector<std::string> status_str = {"ID", "SOG", "COG", "RB", "RNG", "HL", "IP", "AH", "SB", "HO", "CRG", "OTG", "OT"};

	// Need one obstacle data structure for each obstacle. Used in the Obstacle_SBMPC. 
	std::vector<Obstacle_Data<Prediction_Obstacle>> data;

	/****************************************************************************************
	*  Name     : determine_situation_type
	*  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class Parameter_Object>
	void determine_situation_type(
		ST& st_A,																// In/out: Situation type of vessel A
		ST& st_B,																// In/out: Situation type of vessel B
		const Parameter_Object &mpc_pars, 										// In: Parameters of the obstacle manager boss	
		const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A 
		const double psi_A, 													// In: Heading of vessel A
		const Eigen::Vector2d &v_B, 											// In: (NE) Velocity vector of vessel B
		const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
		const double d_AB 														// In: Distance from vessel A to vessel B
		)
	{
		// Crash situation or outside consideration range
		if(d_AB < mpc_pars.d_safe || d_AB > mpc_pars.d_close)
		{
			st_A = A; st_B = A;
			return;
		} 
		// Inside consideration range
		else
		{
			bool B_is_starboard(false), A_is_overtaken(false), B_is_overtaken(false);
			bool is_ahead(false), is_passed(false), is_head_on(false), is_crossing(false);

			is_ahead = v_A.dot(L_AB) > cos(mpc_pars.phi_AH) * v_A.norm();

			A_is_overtaken = v_A.dot(v_B) > cos(mpc_pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
							v_A.norm() < v_B.norm()							  		&&
							v_A.norm() > 0.25;

			B_is_overtaken = v_B.dot(v_A) > cos(mpc_pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
							v_B.norm() < v_A.norm()							  		&&
							v_B.norm() > 0.25;

			B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
						!A_is_overtaken) 											||
						(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
						!B_is_overtaken)) 											&&
						d_AB > mpc_pars.d_safe;

			is_head_on = v_A.dot(v_B) < - cos(mpc_pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() > 0.25											&&
						v_B.norm() > 0.25											&&
						is_ahead;

			is_crossing = v_A.dot(v_B) < cos(mpc_pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
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
	*			  structures for the "intelligent" obstacle i.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class Parameter_Object>
	void update_obstacles(
		const int i, 																		// In: Index of obstacle i to update data for				
		const Parameter_Object &mpc_pars,													// In: Parameters of the obstacle manager boss
		const Eigen::Matrix<double, 7, -1> &obstacle_states, 								// In: Other dynamic obstacle states 
		const int k																			// In: Index of the current predicted time t_k																
		) 			
	{
		int n_obst_old = data[i].obstacles.size();
		int n_obst_new = obstacle_states.cols();
		bool obstacle_exist(false);
		for (int j = 0; j < n_obst_new; j++)
		{
			obstacle_exist = false;
			for (int z = 0; z < n_obst_old; z++)
			{
				if ((double)data[i].obstacles[z].get_ID() == obstacle_states(6, j))
				{
					data[i].obstacles[z].update(obstacle_states.block<4, 1>(0, j), k);

					data[i].new_obstacles.push_back(std::move(data[i].obstacles[z]));

					obstacle_exist = true;

					break;
				}
			}
			if (!obstacle_exist)
			{
				data[i].new_obstacles.push_back(std::move(Prediction_Obstacle(
					obstacle_states.col(j), 
					false,
					mpc_pars.T, 
					mpc_pars.dt)));
			}
		}
		data[i].obstacles = std::move(data[i].new_obstacles);	
	}

	/****************************************************************************************
	*  Name     : update_situation_type_and_transitional_variables
	*  Function : Updates the situation type for obstacle i (wrt all other obstacles) and
	*			  for other obstacles (wrt obstacle i), and the transitional cost indicators 
	*			  O, Q, X, S at the current predicted time t_k wrt all other obstacles.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class Parameter_Object>
	void update_situation_type_and_transitional_variables(
		const int i, 																		// In: Index of obstacle asking for a situational awareness update
		const Parameter_Object &mpc_pars,													// In: Parameters of the obstacle manager boss, an Obstacle_SBMPC
		const Eigen::Vector4d &obstacle_i_state,											// In: Current predicted time state of obstacle i
		const double obstacle_i_length,														// In: Dimension of obstacle i along longest axis
		const int k																			// In: Index of the current predicted time t_k
		)
	{
		bool is_close(false);

		// A : Obstacle i, B : Obstacle j
		Eigen::Vector2d v_A, v_B, L_AB;
		double psi_A(0.0), psi_B(0.0), d_AB(0.0);
		v_A(0) = obstacle_i_state(2);
		v_A(1) = obstacle_i_state(3);
		psi_A = atan2(v_A(1), v_A(0));

		int n_obst = data[i].obstacles.size();
		data[i].ST_0.resize(n_obst);   data[i].ST_i_0.resize(n_obst);
		
		data[i].AH_0.resize(n_obst);   data[i].S_TC_0.resize(n_obst); data[i].S_i_TC_0.resize(n_obst); 
		data[i].O_TC_0.resize(n_obst); data[i].Q_TC_0.resize(n_obst); data[i].IP_0.resize(n_obst); 
		data[i].H_TC_0.resize(n_obst); data[i].X_TC_0.resize(n_obst);

		//std::cout << "Situation types:: 0 : (ST = Ø), 1 : (ST = OT, SO), 2 : (ST = CR, SO), 3 : (ST = OT, GW), 4 : (ST = HO, GW), 5 : (ST = CR, GW)" << std::endl;
		for (int j = 0; j < n_obst; j++)
		{
			v_B(0) = data[i].obstacles[j].get_state(k)(2);
			v_B(1) = data[i].obstacles[j].get_state(k)(3);
			psi_B = atan2(v_B(1), v_B(0));

			L_AB(0) = data[i].obstacles[j].get_state(k)(0) - obstacle_i_state(0);
			L_AB(1) = data[i].obstacles[j].get_state(k)(1) - obstacle_i_state(1);
			d_AB = L_AB.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_AB = d_AB - 0.5 * (obstacle_i_length + data[i].obstacles[j].get_length()); 
			
			L_AB = L_AB.normalized();

			determine_situation_type(data[i].ST_0[j], data[i].ST_i_0[j], mpc_pars, v_A, psi_A, v_B, L_AB, d_AB);
			
			//std::cout << "Obstacle i = " << i << " situation type wrt obst j = " << j << " ? " << data[i].ST_0[j] << std::endl;
			//std::cout << "Obst j = " << j << " situation type wrt obstacle i = " << i << " ? " << data[i].ST_i_0[j] << std::endl;

			/*********************************************************************
			* Transitional variable update
			*********************************************************************/
			is_close = d_AB <= mpc_pars.d_close;

			data[i].AH_0[j] = v_A.dot(L_AB) > cos(mpc_pars.phi_AH) * v_A.norm();

			//std::cout << "Obst j = " << j << " ahead at t0 ? " << data[i].AH_0[j] << std::endl;
			
			// Obstacle on starboard side
			data[i].S_TC_0[j] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << data[i].S_TC_0[j] << std::endl;

			// Ownship on starboard side of obstacle
			data[i].S_i_TC_0[j] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			//std::cout << "Obstacle i = " << i << " on starboard side of obstacle j = " << j << " at t0 ? " << data[i].S_i_TC_0[j] << std::endl;

			// Ownship overtaking the obstacle
			data[i].O_TC_0[j] = v_B.dot(v_A) > cos(mpc_pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							    						&&
					v_B.norm() > 0.25															&&
					is_close 																	&&
					data[i].AH_0[j];

			//std::cout << "Own-ship overtaking obst j = " << j << " at t0 ? " << data[i].O_TC_0[j] << std::endl;

			// Obstacle overtaking the ownship
			data[i].Q_TC_0[j] = v_A.dot(v_B) > cos(mpc_pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  							&&
					v_A.norm() > 0.25 															&&
					is_close 																	&&
					!data[i].AH_0[j];

			//std::cout << "Obst j = " << j << " overtaking obstacle i = " << i << " at t0 ? " << data[i].Q_TC_0[j] << std::endl;

			// Determine if the obstacle is passed by
			data[i].IP_0[j] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
					!data[i].Q_TC_0[j])		 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 				&& // Obstacle's perspective	
					!data[i].O_TC_0[j]))		 										&&
					d_AB > mpc_pars.d_safe;
			
			//std::cout << "Obst j = " << j << " passed by at t0 ? " << data[i].IP_0[j] << std::endl;

			// This is not mentioned in article, but also implemented here..				
			data[i].H_TC_0[j] = v_A.dot(v_B) < - cos(mpc_pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() > 0.25																&&
					v_B.norm() > 0.25																&&
					data[i].AH_0[j];
			
			//std::cout << "Head-on at t0 wrt obst j = " << j << " ? " << data[i].H_TC_0[j] << std::endl;

			// Crossing situation, a bit redundant with the !is_passed condition also, 
			// but better safe than sorry (could be replaced with B_is_ahead also)
			data[i].X_TC_0[j] = v_A.dot(v_B) < cos(mpc_pars.phi_CR) * v_A.norm() * v_B.norm()	&&
					!data[i].H_TC_0[j]															&& 
					!data[i].IP_0[j]															&&
					v_A.norm() > 0.25															&&
					v_B.norm() > 0.25;

			//std::cout << "Crossing at t0 wrt obst j = " << j << " ? " << data[i].X_TC_0[j] << std::endl;
		}
	}

public:

	Joint_Prediction_Manager(const int n_obst);

	~Joint_Prediction_Manager();

	Obstacle_Data<Prediction_Obstacle>& get_data(int i) { return data[i]; };

	void update_obstacle_status(const int i, const Eigen::Vector4d &obstacle_i_state, const int k);

	void display_obstacle_information(const int i);

	/****************************************************************************************
	*  Name     : operator()
	*  Function : Manages new obstacle information, updates data structures and situation
	*			  information accordingly 
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	template <class Parameter_Object>
	void operator()(
		const Parameter_Object &mpc_pars,													// In: Parameters of the obstacle manager boss, an Obstacle_SBMPC
		const std::vector<Prediction_Obstacle> &pobstacles, 								// In: Vector of Prediction Obstacles
		const Eigen::VectorXd &xs_os_aug_k, 												// In: Augmented ownship state consisting of [x, y, Vx, Vy, l, w, ID] at the current predicted time
		const int k																			// In: Index of the current predicted time t_k
		) 			
	{
		int n_obst = pobstacles.size();

		Eigen::Matrix<double, 7, -1> other_obstacle_states(7, n_obst);

		Eigen::Vector4d obstacle_i_state, ownship_4d_state;
		double obstacle_i_length(0.0);
		int count(0);
		for (int i = 0; i < n_obst; i++)
		{
			obstacle_i_state = pobstacles[i].get_state(k);
			obstacle_i_length = pobstacles[i].get_length();

			// Aquire information from all other obstacles
			count = 0;
			for (int j = 0; j < n_obst + 1; j++)
			{
				if (j == n_obst) // Ownship is the last "other obstacle" with ID = n_obst
				{
					other_obstacle_states.col(count) = xs_os_aug_k; 
					count += 1;
				}
				else if (j != i)
				{
					other_obstacle_states.block<4, 1>(0, count) = pobstacles[j].get_state(k);
					other_obstacle_states(4, count) = pobstacles[j].get_length();
					other_obstacle_states(5, count) = pobstacles[j].get_width();
					other_obstacle_states(6, count) = pobstacles[j].get_ID();
					count += 1;
				}
				else
				{
					// Continue
				}
			}
			//std::cout << other_obstacle_states.transpose() << std::endl;

			update_obstacles<Parameter_Object>(i, mpc_pars, other_obstacle_states, k);

			update_situation_type_and_transitional_variables<Parameter_Object>(i, mpc_pars, obstacle_i_state, obstacle_i_length, k);
		}
	}
};

#endif 