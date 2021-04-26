/****************************************************************************************
*
*  File name : obstacle_predictor.hpp
*
*  Function  : Header file for the obstacle predictor class used by the PSB-MPC.
*			   Predicts dynamic obstacle trajectories using current time information.
*			   Transfers the trajectory data to (tracked) obstacle data structures.
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#include "cpu/utilities_cpu.hpp"
#include "obstacle_manager.hpp"
#include "mrou.hpp"

#include <vector>
#include <memory>

namespace PSBMPC_LIB
{
	class Obstacle_Predictor
	{
	private:

		// Number of prediction scenarios for each obstacle
		std::vector<int> n_ps;

		// Ordering of obstacle i`s prediction scenarios, size n_ps x 1
		std::vector<Intention> ps_ordering_i;
		
		// Matrices of prediction scenario course changes and maneuver times for an obstacle i, size n_cc x n_ps
		Eigen::MatrixXd ps_course_changes_i, ps_maneuver_times_i;

		/****************************************************************************************
		*  Name     : set_up_independent_obstacle_prediction_variables
		*  Function : 
		*  Author   : 
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void Obstacle_Predictor::set_up_independent_obstacle_prediction_v1(
			const double t_cpa_i, 													// In: Time to Closest Point of Approach for obstacle i wrt own-ship
			const int i, 															// In: Index of obstacle in consideration
			const MPC_Type &mpc														// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int turn_count(0), turn_start(0), n_turns(0), course_change_count(0);

			if (turn_start >= 0) // Alternative maneuvers are only up until cpa with the own-ship
			{
				n_turns = std::ceil((t_cpa_i - turn_start * mpc.pars.t_ts) / mpc.pars.t_ts);
			}
			else 							
			{
				n_turns = 0;
			}
			n_ps[i] = 1 + 2 * mpc.pars.obstacle_course_changes.size() * n_turns;

			//std::cout << "obst i = " << i << " | t_cpa = " << t_cpa_i << "n_turns = " << n_turns << std::endl;

			ps_ordering_i.resize(n_ps[i]);
			ps_ordering_i[0] = KCC;
			ps_maneuver_times_i.resize(n_ps[i]);
			ps_maneuver_times_i[0] = 0;
			ps_course_changes_i.resize(n_ps[i]);
			ps_course_changes_i[0] = 0;
			
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_ordering_i[ps] = SM;

					ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

					ps_course_changes_i(ps) = mpc.pars.obstacle_course_changes(course_change_count);
					if (++course_change_count == mpc.pars.obstacle_course_changes.size())
					{
						if(++turn_count == n_turns) turn_count = 0;
						course_change_count = 0;
					} 
				}
				// Port maneuvers
				else
				{
					ps_ordering_i[ps] = PM;

					ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

					ps_course_changes_i(ps) = - mpc.pars.obstacle_course_changes(course_change_count);
					if (++course_change_count == mpc.pars.obstacle_course_changes.size())
					{
						if(++turn_count == n_turns) turn_count = 0;
						course_change_count = 0;
					} 
				}	
			}
			/* std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
			std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl; */
		}

		template <class MPC_Type>
		void Obstacle_Predictor::set_up_independent_obstacle_prediction_v2(
			const double t_cpa_i, 													// In: Time to Closest Point of Approach for obstacle i wrt own-ship
			const int i, 															// In: Index of obstacle in consideration
			const MPC_Type &mpc														// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_cc(2);

			Eigen::VectorXd possible_course_changes((n_ps[i] - 1) / 2 + 1);
			if (n_ps[i] == 3)
			{
				possible_course_changes << 45 * DEG2RAD;
			}
			else if (n_ps[i] == 5)
			{
				possible_course_changes << 45 * DEG2RAD, 90 * DEG2RAD
			}
			else
			{
				possible_course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
			}

			//std::cout << "obst i = " << i << " | t_cpa = " << t_cpa_i << "n_turns = " << n_turns << std::endl;

			ps_ordering_i.resize(n_ps[i]);
			ps_ordering_i[0] = KCC;
			ps_maneuver_times_i.resize(n_cc, n_ps[i]);
			ps_course_changes_i.resize(n_cc, n_ps[i]);
			
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_ordering_i[ps] = SM;
					ps_maneuver_times_i(0, ps) = 0;
					ps_maneuver_times_i(1, ps) = mpc.pars.t_ts;

					ps_course_changes_i(0, ps) = 0;
					ps_course_changes_i(1, ps) = 45 * DEG2RAD;
				}
				// Port maneuvers
				else
				{
					ps_ordering_i[ps] = PM;
					ps_maneuver_times_i(0, ps) = 0;
					ps_maneuver_times_i(1, ps) = mpc.pars.t_ts;
				}	
			}
			/* std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
			std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl; */
		}

		/****************************************************************************************
		*  Name     : initialize_independent_prediction (v1 and v2)
		*  Function : Sets up independent obstacle prediction.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void Obstacle_Predictor::initialize_independent_prediction_v1(
			Obstacle_Data<Tracked_Obstacle> &data,								// In/Out: Dynamic obstacle information
			const int i, 														// In: Index of obstacle whose prediction to initialize
			const Eigen::VectorXd &ownship_state,								// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 												// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_obst = data.obstacles.size();
			n_ps.resize(n_obst);
			int n_a(0);
			if (n_obst > 0)
			{
				n_a = data.obstacles[0].get_intention_probabilities().size();
			} 
			Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
			Eigen::Vector2d p_cpa, v_os_0;
			Eigen::Vector4d xs_i_0, xs_0;
			if (ownship_state.rows() == 4)
			{
				v_os_0(0) = ownship_state(3) * cos(ownship_state(2));
				v_os_0(1) = ownship_state(3) * sin(ownship_state(2));
			}
			else
			{
				v_os_0(0) = ownship_state(3); v_os_0(1) = ownship_state(4);
				v_os_0 = rotate_vector_2D(v_os_0, ownship_state(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = ownship_state.block<2, 1>(0, 0);
			xs_0(2) = v_os_0(0); xs_0(3) = v_os_0(1);
			Eigen::Matrix<double, 2, -1> waypoints_i;

				n_ps[i] = 1;
				
			xs_i_0 = data.obstacles[i].kf.get_state();
			/* std::cout << "xs_i_0 = " << xs_i_0.transpose() << std::endl;
			std::cout << "xs_0 = " << xs_0.transpose() << std::endl; */
			calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);
			/* std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
			std::cout << "t_cpa(i) = " << t_cpa(i) << std::endl;
			std::cout << "d_cpa(i) = " << d_cpa(i)<< std::endl; */
			if (n_a == 1 || data.IP_0[i])
			{
				/* std::cout << "Obstacle i = " << i << "is passed => 1 PS only" << std::endl; */
				ps_ordering_i.resize(1);
				ps_ordering_i[0] = KCC;			
				ps_course_changes_i.resize(1);
				ps_course_changes_i[0] = 0;
				ps_maneuver_times_i.resize(1);
				ps_maneuver_times_i(0) = 0;
			}
			else
			{
				set_up_independent_obstacle_prediction_v1(t_cpa(i), i, mpc);
			}
		}

		template <class MPC_Type>
		void Obstacle_Predictor::initialize_independent_prediction_v2(
			Obstacle_Data<Tracked_Obstacle> &data,								// In/Out: Dynamic obstacle information
			const int i, 														// In: Index of obstacle whose prediction to initialize
			const Eigen::VectorXd &ownship_state,								// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 												// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_obst = data.obstacles.size();
			n_ps.resize(n_obst);
			int n_a(0);
			if (n_obst > 0)
			{
				n_a = data.obstacles[0].get_intention_probabilities().size();
			} 
			Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
			Eigen::Vector2d p_cpa, v_os_0;
			Eigen::Vector4d xs_i_0, xs_0;
			if (ownship_state.rows() == 4)
			{
				v_os_0(0) = ownship_state(3) * cos(ownship_state(2));
				v_os_0(1) = ownship_state(3) * sin(ownship_state(2));
			}
			else
			{
				v_os_0(0) = ownship_state(3); v_os_0(1) = ownship_state(4);
				v_os_0 = rotate_vector_2D(v_os_0, ownship_state(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = ownship_state.block<2, 1>(0, 0);
			xs_0(2) = v_os_0(0); xs_0(3) = v_os_0(1);
			Eigen::Matrix<double, 2, -1> waypoints_i;

			n_ps[i] = 1;
			
			xs_i_0 = data.obstacles[i].kf.get_state();
			/* std::cout << "xs_i_0 = " << xs_i_0.transpose() << std::endl;
			std::cout << "xs_0 = " << xs_0.transpose() << std::endl; */
			calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);
			/* std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
			std::cout << "t_cpa(i) = " << t_cpa(i) << std::endl;
			std::cout << "d_cpa(i) = " << d_cpa(i)<< std::endl; */
			if (n_a == 1 || data.IP_0[i])
			{
				/* std::cout << "Obstacle i = " << i << "is passed => 1 PS only" << std::endl; */
				ps_ordering_i.resize(1);
				ps_ordering_i[0] = KCC;			
				ps_course_changes_i.resize(1);
				ps_course_changes_i[0] = 0;
				ps_maneuver_times_i.resize(1);
				ps_maneuver_times_i(0) = 0;
			}
			else
			{
				set_up_independent_obstacle_prediction_v2(t_cpa(i), i, mpc);
			}
		}

		/****************************************************************************************
		*  Name     : predict_independent_trajectories
		*  Function : Predicts the obstacle trajectories for scenarios where the obstacle
		*			  does not take the own-ship into account.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void predict_independent_trajectories_v1(
			Obstacle_Data<Tracked_Obstacle> &data,				// In/Out: Dynamic obstacle information
			const int i,										// In: Index of obstacle whose trajectories to predict
			const Eigen::VectorXd &ownship_state, 				// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_samples = std::round(mpc.pars.T / mpc.pars.dt);
			resize_trajectories(n_samples);

			int n_ps_independent = ps_course_changes.size();
			
			Eigen::MatrixXd P_p; P_p.resize(16, n_samples);
			P_p.col(0) = CPU::flatten(data.obstacles[i].kf.get_covariance());

			std::vector<Eigen::MatrixXd> xs_p(n_ps_independent);

			Eigen::VectorXd ownship_state_sl = ownship_state;
			
			Eigen::Vector2d v_p, v_p_new, v_A, v_B, L_AB;
			double chi_ps, t = 0, psi_A, d_AB;
			bool have_turned;
			for(int ps = 0; ps < n_ps_independent; ps++)
			{
				xs_p[ps].resize(4, n_samples);
				xs_p[ps].col(0) = data.obstacles[i].kf.get_state();

				ownship_state_sl = ownship_state;

				v_p(0) = kf.get_state()(2);
				v_p(1) = kf.get_state()(3);

				have_turned = false;	
				for(int k = 0; k < n_samples; k++)
				{
					t = (k + 1) * dt;

					if (ownship_state_sl.size() == 4)
					{
						v_B(0) = ownship_state_sl(2);
						v_B(1) = ownship_state_sl(3);
					}
					else
					{
						v_B(0) = ownship_state_sl(3);
						v_B(1) = ownship_state_sl(4);
						v_B = CPU::rotate_vector_2D(v_B, ownship_state_sl(2));
					}

					psi_A = atan2(xs_p[ps](4), xs_p[ps](0));
					L_AB = xs_p[ps].block<2, 1>(0, k) - ownship_state_sl.block<2, 1>(0, 0);
					d_AB = L_AB.norm();
					L_AB.normalize();

					if (!mu[ps])
					{
						mu[ps] = mpc.mpc_cost.determine_COLREGS_violation(v_A, psi_A, v_B, L_AB, d_AB);
					}
				
					switch (ps_ordering[ps])
					{
						case KCC :	
							break; // Proceed
						case SM :
							if (k == ps_maneuver_times[ps] && !have_turned)
							{
								chi_ps = atan2(v_p(1), v_p(0)); 
								v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
								v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
								v_p = v_p_new;
								have_turned = true;
							}
							break;
						case PM : 
							if (k == ps_maneuver_times[ps] && !have_turned)
							{
								chi_ps = atan2(v_p(1), v_p(0)); 
								v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
								v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
								v_p = v_p_new;
								have_turned = true;
							}
							break;
						default :
							// Throw
							break;
					}

					if (k < n_samples - 1)
					{
						xs_p[ps].col(k + 1) = mrou.predict_state(xs_p[ps].col(k), v_p, mpc.pars.dt);

						if (ps == 0) P_p.col(k + 1) = CPU::flatten(mrou.predict_covariance(P_p.col(0), t));

						// Propagate ownship assuming straight line trajectory
						if (ownship_state_sl.size() == 4)
						{
							ownship_state_sl(0) = ownship_state_sl(0) + mpc.pars.dt * ownship_state_sl(3) * cos(ownship_state_sl(2));
							ownship_state_sl(1) = ownship_state_sl(1) + mpc.pars.dt * ownship_state_sl(3) * sin(ownship_state_sl(2));
							ownship_state_sl.block<2, 1>(2, 0) = ownship_state_sl.block<2, 1>(2, 0);
						}
						else
						{
							ownship_state_sl.block<2, 1>(0, 0) =  ownship_state_sl.block<2, 1>(0, 0) + 
								mpc.pars.dt * CPU::rotate_vector_2D(ownship_state_sl.block<2, 1>(3, 0), ownship_state_sl(2, 0));
							ownship_state_sl.block<4, 1>(2, 0) = ownship_state_sl.block<4, 1>(2, 0);
						}
					}
				}
			}
		}

		/****************************************************************************************
		*  Name     : predict_independent_trajectories_v2
		*  Function : More refined obstacle prediction with avoidance-like trajectories,
		*			  including the straight-line trajectory
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void predict_independent_trajectories_v2(
			Obstacle_Data<Tracked_Obstacle> &data,				// In/Out: Dynamic obstacle information
			const int i,										// In: Index of obstacle whose trajectories to predict
			const Eigen::VectorXd &ownship_state, 				// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_samples = std::round(mpc.pars.T / mpc.pars.dt);
			resize_trajectories(n_samples);

			int n_ps_independent = ps_course_changes.size();
			
			Eigen::MatrixXd P_p; P_p.resize(16, n_samples);
			P_p.col(0) = CPU::flatten(data.obstacles[i].kf.get_covariance());

			std::vector<Eigen::MatrixXd> xs_p(n_ps_independent);

			Eigen::VectorXd ownship_state_sl = ownship_state;
			
			Eigen::Vector2d v_p, v_p_new, v_A, v_B, L_AB;
			double chi_ps, t = 0, psi_A, d_AB;
			bool have_turned;
			for(int ps = 0; ps < n_ps_independent; ps++)
			{
				xs_p[ps].resize(4, n_samples);
				xs_p[ps].col(0) = data.obstacles[i].kf.get_state();

				ownship_state_sl = ownship_state;

				v_p(0) = kf.get_state()(2);
				v_p(1) = kf.get_state()(3);

				have_turned = false;	
				for(int k = 0; k < n_samples; k++)
				{
					t = (k + 1) * mpc.pars.dt;

					if (ownship_state_sl.size() == 4)
					{
						v_B(0) = ownship_state_sl(2);
						v_B(1) = ownship_state_sl(3);
					}
					else
					{
						v_B(0) = ownship_state_sl(3);
						v_B(1) = ownship_state_sl(4);
						v_B = CPU::rotate_vector_2D(v_B, ownship_state_sl(2));
					}

					psi_A = atan2(xs_p[ps](4), xs_p[ps](0));
					L_AB = xs_p[ps].block<2, 1>(0, k) - ownship_state_sl.block<2, 1>(0, 0);
					d_AB = L_AB.norm();
					L_AB.normalize();

					if (!mu[ps])
					{
						mu[ps] = mpc.mpc_cost.determine_COLREGS_violation(v_A, psi_A, v_B, L_AB, d_AB);
					}
				
					switch (ps_ordering[ps])
					{
						case KCC :	
							break; // Proceed
						case SM :
							if (k == ps_maneuver_times[ps] && !have_turned)
							{
								chi_ps = atan2(v_p(1), v_p(0)); 
								v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
								v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
								v_p = v_p_new;
								have_turned = true;
							}
							break;
						case PM : 
							if (k == ps_maneuver_times[ps] && !have_turned)
							{
								chi_ps = atan2(v_p(1), v_p(0)); 
								v_p_new(0) = v_p.norm() * cos(chi_ps + ps_course_changes[ps]);
								v_p_new(1) = v_p.norm() * sin(chi_ps + ps_course_changes[ps]);
								v_p = v_p_new;
								have_turned = true;
							}
							break;
						default :
							// Throw
							break;
					}

					if (k < n_samples - 1)
					{
						xs_p[ps].col(k + 1) = mrou.predict_state(xs_p[ps].col(k), v_p, mpc.pars.dt);

						if (ps == 0) P_p.col(k + 1) = CPU::flatten(mrou.predict_covariance(P_p.col(0), t));

						// Propagate ownship assuming straight line trajectory
						if (ownship_state_sl.size() == 4)
						{
							ownship_state_sl(0) = ownship_state_sl(0) + mpc.pars.dt * ownship_state_sl(3) * cos(ownship_state_sl(2));
							ownship_state_sl(1) = ownship_state_sl(1) + mpc.pars.dt * ownship_state_sl(3) * sin(ownship_state_sl(2));
							ownship_state_sl.block<2, 1>(2, 0) = ownship_state_sl.block<2, 1>(2, 0);
						}
						else
						{
							ownship_state_sl.block<2, 1>(0, 0) =  ownship_state_sl.block<2, 1>(0, 0) + 
								mpc.pars.dt * CPU::rotate_vector_2D(ownship_state_sl.block<2, 1>(3, 0), ownship_state_sl(2, 0));
							ownship_state_sl.block<4, 1>(2, 0) = ownship_state_sl.block<4, 1>(2, 0);
						}
					}
				}
			}

			data.obstacles[i].setup_prediction(xs_p, v_ou_p, P_p, ps_ordering_i);
		}
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		MROU mrou;

		Obstacle_Predictor() {}

		int get_n_ps_i(const int i) const { return n_ps[i]; }

		std::vector<int> get_n_ps() const { return n_ps; }

		/****************************************************************************************
		*  Name     : operator()
		*  Function : Initializes and predicts obstacle trajectories
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void operator()(
			Obstacle_Data<Tracked_Obstacle> &data, 					// In/Out: Dynamic obstacle information
			const Eigen::VectorXd &ownship_state, 					// In: Own-ship state at the current time
			const MPC_Type &mpc										// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_obst = data.obstacles.size();
			for (int i = 0; i < n_obst; i++)
			{
				//v1: Dumb course maneuver prediction
				//initialize_independent_prediction_v1(data, i, ownship_state, mpc);
				//predict_independent_trajectories_v1(data, i, ownship_state, mpc);

				//v2: LOS-like prediction
				initialize_independent_prediction_v2(data, i, ownship_state, mpc);
				predict_independent_trajectories_v2(data, i, ownship_state, mpc);
			}
		}
	};
}