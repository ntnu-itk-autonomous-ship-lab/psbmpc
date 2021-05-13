/****************************************************************************************
*
*  File name : obstacle_manager.hpp
*
*  Function  : Header file for the obstacle management interface and data structure for
*			   keeping information on dynamic obstacles. As of now only
*			   used for dynamic obstacle management.
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

#pragma once

#include "psbmpc_parameters.hpp"
#include "tracked_obstacle.hpp"

#include "Eigen/Dense"
#include <string>


namespace PSBMPC_LIB
{
	enum ST 
	{
		A, 														// Non-COLREGS situation	(ST = Ø)
		B, 														// Stand-on in Overtaking 	(ST = OT, SO)
		C, 														// Stand-on in Crossing 	(ST = CR, SO)
		D, 														// Give-way in Overtaking 	(ST = OT, GW)
		E, 														// Give-way in Head-on 		(ST = HO, GW)
		F 														// Give-way in Crossing 	(ST = CR, GW)
	};	

	template <class Obstacle_Type>
	class Obstacle_Data
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
		// and <obstacle is passed> (IP_0) indicators
		std::vector<bool> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

		// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
		std::vector<ST> ST_0, ST_i_0;

		// Obstacle hazard levels, on a scale from 0 to 1 (output from PSBMPC)
		Eigen::VectorXd HL_0;

		std::vector<Obstacle_Type, Eigen::aligned_allocator<Obstacle_Type>> obstacles;
		std::vector<Obstacle_Type, Eigen::aligned_allocator<Obstacle_Type>> new_obstacles;

		Eigen::MatrixXd obstacle_status;

		Obstacle_Data() {}

		~Obstacle_Data() {}
	};

	class Obstacle_Manager
	{
	private:

		// Array of strings and precisions for the obstacle status states
		std::vector<std::string> status_str = {"ID", "SOG", "COG", "RB", "RNG", "HL", "IP", "AH", "SB", "HO", "CRG", "OTG", "OT"};

		double T_lost_limit, T_tracked_limit;

		bool obstacle_filter_on;

		Obstacle_Data<Tracked_Obstacle> data;

		/****************************************************************************************
		*  Name     : determine_situation_type
		*  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void determine_situation_type(
			ST& st_A,																// In/out: Situation type of vessel A
			ST& st_B,																// In/out: Situation type of vessel B
			const Eigen::Vector2d &v_A,												// In: (NE) Velocity vector of vessel A 
			const double psi_A, 													// In: Heading of vessel A
			const Eigen::Vector2d &v_B, 											// In: (NE) Velocity vector of vessel B
			const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
			const double d_AB, 														// In: Distance from vessel A to vessel B
			const MPC_Type &mpc														// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			// Crash situation or outside consideration range
			if(d_AB < mpc.pars.d_safe || d_AB > mpc.pars.d_close)
			{
				st_A = A; st_B = A;
				return;
			} 
			// Inside consideration range
			else
			{
				bool B_is_starboard, A_is_overtaken, B_is_overtaken;
				bool is_ahead, is_passed, is_head_on, is_crossing;

				is_ahead = v_A.dot(L_AB) > cos(mpc.pars.phi_AH) * v_A.norm();

				A_is_overtaken = v_A.dot(v_B) > cos(mpc.pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
								v_A.norm() < v_B.norm()							  		&&
								v_A.norm() > 0.25;

				B_is_overtaken = v_B.dot(v_A) > cos(mpc.pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
								v_B.norm() < v_A.norm()							  		&&
								v_B.norm() > 0.25;

				B_is_starboard = CPU::angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

				is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
							!A_is_overtaken) 											||
							(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
							!B_is_overtaken)) 											&&
							d_AB > mpc.pars.d_safe;

				is_head_on = v_A.dot(v_B) < - cos(mpc.pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
							v_A.norm() > 0.25											&&
							v_B.norm() > 0.25											&&
							is_ahead;

				is_crossing = v_A.dot(v_B) < cos(mpc.pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
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
		template <class MPC_Type>
		void update_obstacles(
			const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
			const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
			const MPC_Type &mpc																	// In: Calling MPC (either PSB-MPC or SB-MPC)
			) 			
		{
			int n_obst_old = data.obstacles.size();
			int n_obst_new = obstacle_states.cols();

			bool obstacle_exist;
			for (int i = 0; i < n_obst_new; i++)
			{
				obstacle_exist = false;
				for (int j = 0; j < n_obst_old; j++)
				{
					if ((double)data.obstacles[j].get_ID() == obstacle_states(8, i))
					{
						data.obstacles[j].reset_duration_lost();

						data.obstacles[j].update(
							obstacle_states.col(i), 
							obstacle_covariances.col(i), 
							obstacle_filter_on,
							mpc.pars.dt);

						data.new_obstacles.push_back(std::move(data.obstacles[j]));

						obstacle_exist = true;

						break;
					}
				}
				if (!obstacle_exist)
				{
					data.new_obstacles.push_back(std::move(Tracked_Obstacle(
						obstacle_states.col(i), 
						obstacle_covariances.col(i),
						obstacle_filter_on,  
						mpc.pars.T, 
						mpc.pars.dt)));
				}
			}
			// Keep terminated obstacles that may still be relevant, and compute duration lost as input to the cost of collision risk
			// Obstacle track may be lost due to sensor/detection failure, or the obstacle may go out of COLAV-target range
			// Detection failure will lead to the start (creation) of a new track (obstacle), typically after a short duration,
			// whereas an obstacle that is out of COLAV-target range may re-enter range with the same id.
			if (obstacle_filter_on)
			{
				for (size_t j = 0; j < data.obstacles.size(); j++)
				{
					data.obstacles[j].increment_duration_lost(mpc.pars.dt * mpc.pars.p_step);

					if (data.obstacles[j].get_duration_tracked() >= T_tracked_limit 	&&
						(data.obstacles[j].get_duration_lost() < T_lost_limit || data.obstacles[j].kf.get_covariance()(0,0) <= 5.0))
					{
						data.obstacles[j].update(obstacle_filter_on, mpc.pars.dt);

						data.new_obstacles.push_back(std::move(data.obstacles[j]));
					}
				}
			}
			// Then, after having all relevant obstacles (includes transferred ones from the old vector, and newly detected ones)
			// in "new_obstacles", transfer all of these to the "obstacle" vector to be used by other classes.
			data.obstacles = std::move(data.new_obstacles);	
		}

		/****************************************************************************************
		*  Name     : update_situation_type_and_transitional_variables
		*  Function : Updates the situation type for the own-ship (wrt all obstacles) and
		*			  obstacles (wrt own-ship) and the transitional cost indicators O, Q, X, S 
		*			  at the current time t0 wrt all obstacles.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void update_situation_type_and_transitional_variables(
			const Eigen::VectorXd &ownship_state,							// In: Current time own-ship state
			const double ownship_length,										// In: Dimension of ownship along longest axis
			const MPC_Type &mpc												// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			bool is_close;

			// A : Own-ship, B : Obstacle i
			Eigen::Vector2d v_A, v_B, L_AB;
			double psi_A, psi_B, d_AB;
			if (ownship_state.size() == 4)
			{
				v_A(0) = ownship_state(3) * cos(ownship_state(2));
				v_A(1) = ownship_state(3) * sin(ownship_state(2));
				psi_A = ownship_state(2);
			}
			else
			{
				v_A(0) = ownship_state(3);
				v_A(1) = ownship_state(4);
				psi_A = ownship_state(2);
				v_A = CPU::rotate_vector_2D(v_A, psi_A);
			}
			
			int n_obst = data.obstacles.size();
			data.ST_0.resize(n_obst);   data.ST_i_0.resize(n_obst);
			
			data.AH_0.resize(n_obst);   data.S_TC_0.resize(n_obst); data.S_i_TC_0.resize(n_obst); 
			data.O_TC_0.resize(n_obst); data.Q_TC_0.resize(n_obst); data.IP_0.resize(n_obst); 
			data.H_TC_0.resize(n_obst); data.X_TC_0.resize(n_obst);

			//std::cout << "Situation types:: 0 : (ST = Ø), 1 : (ST = OT, SO), 2 : (ST = CR, SO), 3 : (ST = OT, GW), 4 : (ST = HO, GW), 5 : (ST = CR, GW)" << std::endl;
			//std::cout << A << std::endl;
			for (int i = 0; i < n_obst; i++)
			{
				v_B(0) = data.obstacles[i].kf.get_state()(2);
				v_B(1) = data.obstacles[i].kf.get_state()(3);
				psi_B = atan2(v_B(1), v_B(0));

				L_AB(0) = data.obstacles[i].kf.get_state()(0) - ownship_state(0);
				L_AB(1) = data.obstacles[i].kf.get_state()(1) - ownship_state(1);
				d_AB = L_AB.norm();

				// Decrease the distance between the vessels by their respective max dimension
				d_AB = d_AB - 0.5 * (ownship_length + data.obstacles[i].get_length()); 
				
				L_AB = L_AB.normalized();

				determine_situation_type(data.ST_0[i], data.ST_i_0[i], v_A, psi_A, v_B, L_AB, d_AB, mpc);
				
				//std::cout << "Own-ship situation type wrt obst i = " << i << " ? " << ST_0[i] << std::endl;
				//std::cout << "Obst i = " << i << " situation type wrt ownship ? " << ST_i_0[i] << std::endl;

				/*********************************************************************
				* Transitional variable update
				*********************************************************************/
				is_close = d_AB <= mpc.pars.d_close;

				data.AH_0[i] = v_A.dot(L_AB) > cos(mpc.pars.phi_AH) * v_A.norm();

				//std::cout << "Obst i = " << i << " ahead at t0 ? " << AH_0[i] << std::endl;
				
				// Obstacle on starboard side
				data.S_TC_0[i] = CPU::angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

				//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << S_TC_0[i] << std::endl;

				// Ownship on starboard side of obstacle
				data.S_i_TC_0[i] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

				//std::cout << "Own-ship on starboard side of obst i = " << i << " at t0 ? " << S_i_TC_0[i] << std::endl;

				// Ownship overtaking the obstacle
				data.O_TC_0[i] = v_B.dot(v_A) > cos(mpc.pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							    					&&
						v_B.norm() > 0.25														&&
						is_close 																&&
						data.AH_0[i];

				//std::cout << "Own-ship overtaking obst i = " << i << " at t0 ? " << O_TC_0[i] << std::endl;

				// Obstacle overtaking the ownship
				data.Q_TC_0[i] = v_A.dot(v_B) > cos(mpc.pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  						&&
						v_A.norm() > 0.25 														&&
						is_close 																&&
						!data.AH_0[i];

				//std::cout << "Obst i = " << i << " overtaking the ownship at t0 ? " << Q_TC_0[i] << std::endl;

				// Determine if the obstacle is passed by
				data.IP_0[i] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
						!data.Q_TC_0[i])		 										||
						(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 			&& // Obstacle's perspective	
						!data.O_TC_0[i]))		 										&&
						d_AB > mpc.pars.d_safe;
				
				//std::cout << "Obst i = " << i << " passed by at t0 ? " << data.IP_0[i] << std::endl;

				// This is not mentioned in article, but also implemented here..				
				data.H_TC_0[i] = v_A.dot(v_B) < - cos(mpc.pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() > 0.25															&&
						v_B.norm() > 0.25															&&
						data.AH_0[i];
				
				//std::cout << "Head-on at t0 wrt obst i = " << i << " ? " << H_TC_0[i] << std::endl;

				// Crossing situation, a bit redundant with the !is_passed condition also, 
				// but better safe than sorry (could be replaced with B_is_ahead also)
				data.X_TC_0[i] = v_A.dot(v_B) < cos(mpc.pars.phi_CR) * v_A.norm() * v_B.norm()	&&
						!data.H_TC_0[i]															&&
						!data.IP_0[i]															&&
						v_A.norm() > 0.25														&&
						v_B.norm() > 0.25;

				//std::cout << "Crossing at t0 wrt obst i = " << i << " ? " << X_TC_0[i] << std::endl;
			}
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Obstacle_Manager();

		Obstacle_Data<Tracked_Obstacle>& get_data() { return data; }

		void update_obstacle_status(const Eigen::VectorXd &ownship_state);

		void display_obstacle_information();

		/****************************************************************************************
		*  Name     : operator()
		*  Function : Manages new obstacle information, updates data structures and situation
		*			  information accordingly 
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void operator()(
			const Eigen::VectorXd &ownship_state,												// In: Current time own-ship state
			const double ownship_length,														// In: Dimension of ownship along longest axis
			const Eigen::Matrix<double, 9, -1> &obstacle_states, 								// In: Dynamic obstacle states 
			const Eigen::Matrix<double, 16, -1> &obstacle_covariances, 							// In: Dynamic obstacle covariances
			const MPC_Type &mpc																	// In: Calling MPC (either PSB-MPC or SB-MPC)
			) 			
		{
			update_obstacles(obstacle_states, obstacle_covariances, mpc);

			update_situation_type_and_transitional_variables(ownship_state, ownship_length, mpc);
		}
	};
}