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

		// Prediction scenario ordering, size n_ps x 1 of intentions
		std::vector<Intention> ps_ordering;

		// Course change ordering and the corresponding times they occur, for the independent prediction scenarios: n_cc x n_ps
		Eigen::MatrixXd ps_course_changes, ps_maneuver_times;
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		MROU mrou;

		Obstacle_Predictor() {}

		/****************************************************************************************
		*  Name     : initialize_independent_prediction (v1 and v2)
		*  Function : Sets up independent obstacle prediction.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void Obstacle_Predictor::initialize_independent_prediction_v1(
			Obstacle_Data<Tracked_Obstacle> &data,								// In/Out: Dynamic obstacle information
			const Eigen::VectorXd &ownship_state,								// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_obst = data.obstacles.size();
			n_ps.resize(n_obst);
			pobstacles.resize(n_obst);
			int n_a(0);
			if (n_obst > 0)
			{
				n_a = data.obstacles[0].get_intention_probabilities().size();
			} 
			//***********************************************************************************
			// Obstacle prediction initialization
			//***********************************************************************************
			std::vector<Intention> ps_ordering_i;
			Eigen::VectorXd ps_course_changes_i;
			Eigen::VectorXd ps_maneuver_times_i;

			Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
			Eigen::Vector2d p_cpa, v_os_0;
			Eigen::Vector4d xs_i_0, xs_0;
			if (trajectory.rows() == 4)
			{
				v_os_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
				v_os_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
			}
			else
			{
				v_os_0(0) = trajectory(3, 0); v_os_0(1) = trajectory(4, 0);
				v_os_0 = rotate_vector_2D(v_os_0, trajectory(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
			xs_0(2) = v_os_0(0); xs_0(3) = v_os_0(1);
			Eigen::Matrix<double, 2, -1> waypoints_i;

			// only use intelligent prediction n_a > 1 intentions are considered
			// and obstacle colav is on
			use_joint_prediction = false; 
			for (int i = 0; i < n_obst; i++)
			{
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
					set_up_independent_obstacle_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i, t_cpa(i), i);

					pobstacles[i] = Prediction_Obstacle(data.obstacles[i]);
					if (pars.obstacle_colav_on)
					{
						use_joint_prediction = true;

						// Set obstacle waypoints to a straight line out from its current time position 
						// if no future obstacle trajectory is available
						waypoints_i.resize(2, 2);
						xs_i_0 = pobstacles[i].get_initial_state();
						waypoints_i.col(0) = xs_i_0.block<2, 1>(0, 0);
						waypoints_i.col(1) = waypoints_i.col(0) + xs_i_0.block<2, 1>(2, 0) * pars.T;
						pobstacles[i].set_waypoints(waypoints_i);

						n_ps[i] += 1;
					}

					
				}
				data.obstacles[i].initialize_independent_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i);	

				data.obstacles[i].predict_independent_trajectories<PSBMPC>(pars.T, pars.dt, trajectory.col(0), *this);
			}


		}

		template <class MPC_Type>
		void Obstacle_Predictor::initialize_independent_prediction_v2(
			Obstacle_Data<Tracked_Obstacle> &data,								// In/Out: Dynamic obstacle information
			const Eigen::VectorXd &ownship_state,								// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_obst = data.obstacles.size();
			int n_ps = mpc.pars.n_r;

			int n_a(0);
			if (n_obst > 0)
			{
				n_a = data.obstacles[0].get_intention_probabilities().size();
			} 
			//***********************************************************************************
			// Obstacle prediction initialization
			//***********************************************************************************
			ps_ordering_i.resize(n_ps);
			ps_course_changes_i.resize(n_ps);
			ps_maneuver_times_i.resize(n_ps);

			Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
			Eigen::Vector2d p_cpa, v_os_0;
			Eigen::Vector4d xs_i_0, xs_0;
			if (trajectory.rows() == 4)
			{
				v_os_0(0) = ownship_state(3, 0) * cos(ownship_state(2, 0));
				v_os_0(1) = ownship_state(3, 0) * sin(ownship_state(2, 0));
			}
			else
			{
				v_os_0(0) = ownship_state(3, 0); v_os_0(1) = ownship_state(4, 0);
				v_os_0 = rotate_vector_2D(v_os_0, ownship_state(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = ownship_state.block<2, 1>(0, 0);
			xs_0(2) = v_os_0(0); xs_0(3) = v_os_0(1);
			Eigen::Matrix<double, 2, -1> waypoints_i;

			// only use intelligent prediction n_a > 1 intentions are considered
			// and obstacle colav is on
			use_joint_prediction = false; 
			for (int i = 0; i < n_obst; i++)
			{
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
					set_up_independent_obstacle_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i, t_cpa(i), i);

					pobstacles[i] = Prediction_Obstacle(data.obstacles[i]);
					if (pars.obstacle_colav_on)
					{
						use_joint_prediction = true;

						// Set obstacle waypoints to a straight line out from its current time position 
						// if no future obstacle trajectory is available
						waypoints_i.resize(2, 2);
						xs_i_0 = pobstacles[i].get_initial_state();
						waypoints_i.col(0) = xs_i_0.block<2, 1>(0, 0);
						waypoints_i.col(1) = waypoints_i.col(0) + xs_i_0.block<2, 1>(2, 0) * pars.T;
						pobstacles[i].set_waypoints(waypoints_i);

						n_ps[i] += 1;
					}
				}
				data.obstacles[i].initialize_independent_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i);	

				data.obstacles[i].predict_independent_trajectories<PSBMPC>(pars.T, pars.dt, ownship_state, *this);
			}
		}

		/****************************************************************************************
		*  Name     : operator()
		*  Function : Initializes and predicts obstacle trajectories
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void operator()(
			Obstacle_Data<Tracked_Obstacle> &data,
			const Eigen::VectorXd &ownship_state,
			const double T,
			const double dt,
			const MPC_Type &mpc)
		{
			

			//v1: Dumb course maneuver prediction
			//initialize_independent_prediction_v1(ownship_state, data);
			//predict_independent_trajectories_v1(data, T, dt, ownship_state, mpc);

			//v2: LOS-like prediction
			//initialize_independent_prediction_v2(ownship_state, data);
			predict_independent_trajectories_v2(data, T, dt, ownship_state, mpc);
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
			const double T, 									// In: Prediction horizon
			const double dt, 									// In: Prediction time step
			const Eigen::VectorXd &ownship_state, 				// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_samples = std::round(T / dt);
			resize_trajectories(n_samples);

			int n_ps_independent = ps_course_changes.size();
			
			Eigen::VectorXd ownship_state_sl = ownship_state;
			P_p.col(0) = CPU::flatten(kf.get_covariance());

			Eigen::Vector2d v_p, v_p_new, v_A, v_B, L_AB;
			double chi_ps, t = 0, psi_A, d_AB;
			bool have_turned;
			for(int ps = 0; ps < n_ps_independent; ps++)
			{
				ownship_state_sl = ownship_state;

				v_p(0) = kf.get_state()(2);
				v_p(1) = kf.get_state()(3);

				xs_p[ps].col(0) = kf.get_state();
				
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
						xs_p[ps].col(k + 1) = mrou.predict_state(xs_p[ps].col(k), v_p, dt);

						if (ps == 0) P_p.col(k + 1) = CPU::flatten(mrou.predict_covariance(P_0, t));

						// Propagate ownship assuming straight line trajectory
						if (ownship_state_sl.size() == 4)
						{
							ownship_state_sl(0) = ownship_state_sl(0) + dt * ownship_state_sl(3) * cos(ownship_state_sl(2));
							ownship_state_sl(1) = ownship_state_sl(1) + dt * ownship_state_sl(3) * sin(ownship_state_sl(2));
							ownship_state_sl.block<2, 1>(2, 0) = ownship_state_sl.block<2, 1>(2, 0);
						}
						else
						{
							ownship_state_sl.block<2, 1>(0, 0) =  ownship_state_sl.block<2, 1>(0, 0) + 
								dt * CPU::rotate_vector_2D(ownship_state_sl.block<2, 1>(3, 0), ownship_state_sl(2, 0));
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
			const double T, 									// In: Prediction horizon
			const double dt, 									// In: Prediction time step
			const Eigen::VectorXd &ownship_state, 				// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_samples = std::round(T / dt);
			resize_trajectories(n_samples);

			int n_ps_independent = ps_course_changes.size();
			
			Eigen::VectorXd ownship_state_sl = ownship_state;
			P_p.col(0) = CPU::flatten(kf.get_covariance());

			Eigen::Vector2d v_p, v_p_new, v_A, v_B, L_AB;
			double chi_ps, t = 0, psi_A, d_AB;
			bool have_turned;
			for(int ps = 0; ps < n_ps_independent; ps++)
			{
				ownship_state_sl = ownship_state;

				v_p(0) = kf.get_state()(2);
				v_p(1) = kf.get_state()(3);

				xs_p[ps].col(0) = kf.get_state();
				
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
						xs_p[ps].col(k + 1) = mrou.predict_state(xs_p[ps].col(k), v_p, dt);

						if (ps == 0) P_p.col(k + 1) = CPU::flatten(mrou.predict_covariance(P_0, t));

						// Propagate ownship assuming straight line trajectory
						if (ownship_state_sl.size() == 4)
						{
							ownship_state_sl(0) = ownship_state_sl(0) + dt * ownship_state_sl(3) * cos(ownship_state_sl(2));
							ownship_state_sl(1) = ownship_state_sl(1) + dt * ownship_state_sl(3) * sin(ownship_state_sl(2));
							ownship_state_sl.block<2, 1>(2, 0) = ownship_state_sl.block<2, 1>(2, 0);
						}
						else
						{
							ownship_state_sl.block<2, 1>(0, 0) =  ownship_state_sl.block<2, 1>(0, 0) + 
								dt * CPU::rotate_vector_2D(ownship_state_sl.block<2, 1>(3, 0), ownship_state_sl(2, 0));
							ownship_state_sl.block<4, 1>(2, 0) = ownship_state_sl.block<4, 1>(2, 0);
						}
					}
				}
			}
		}
	};
}