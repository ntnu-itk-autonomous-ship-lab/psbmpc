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
		// Max 3sigma cross-track standard deviation for the prediction scenario uncertainty
		double r_ct;

		// Possible course_changes for an obstacle in the MROU prediction
		Eigen::VectorXd course_changes;

		// Number of prediction scenarios for each obstacle
		std::vector<int> n_ps;
		
		// Matrices of prediction scenario course changes and maneuver times for an obstacle i, size n_cc x n_ps
		Eigen::MatrixXd ps_course_changes_i, ps_maneuver_times_i;

		// Prediction scenario trajectory vector for obstacle i
		std::vector<Eigen::MatrixXd> xs_p_i;

		// Prediction scenario trajectory covariance for obstacle i
		Eigen::MatrixXd P_p_i;

		/****************************************************************************************
		*  Name     : set_up_independent_obstacle_prediction_variables
		*  Function : 
		*  Author   : 
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void set_up_independent_obstacle_prediction(
			const double t_cpa_i, 													// In: Time to Closest Point of Approach for obstacle i wrt own-ship
			const int i, 															// In: Index of obstacle in consideration
			const MPC_Type &mpc														// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_cc(2); // Number of course changes for the obstacle (first at t_0)
			int course_change_count(0);

			course_changes.resize((n_ps[i] - 1) / 2 + 1);
			if (n_ps[i] == 3)
			{
				course_changes << 45 * DEG2RAD;
			}
			else if (n_ps[i] == 5)
			{
				course_changes << 45 * DEG2RAD, 90 * DEG2RAD;
			}
			else
			{
				course_changes << 30 * DEG2RAD, 60 * DEG2RAD, 90 * DEG2RAD;
			}

			//std::cout << "obst i = " << i << " | t_cpa = " << t_cpa_i << "n_turns = " << n_turns << std::endl;

			ps_maneuver_times_i.resize(n_cc, n_ps[i]);
			ps_course_changes_i.resize(n_cc, n_ps[i]);
			
			for (int ps = 1; ps < n_ps[i]; ps++)
			{
				// Starboard maneuvers
				if (ps < (n_ps[i] - 1) / 2 + 1)
				{
					ps_maneuver_times_i(0, ps) = 0;
					ps_maneuver_times_i(1, ps) = mpc.pars.t_ts;

					ps_course_changes_i(0, ps) = course_changes(course_change_count);
					ps_course_changes_i(1, ps) = - course_changes(course_change_count);

					if (++course_change_count == course_changes.size()) { course_change_count = 0; }
				}
				// Port maneuvers
				else
				{
					ps_maneuver_times_i(0, ps) = 0;
					ps_maneuver_times_i(1, ps) = mpc.pars.t_ts;

					ps_course_changes_i(0, ps) = - course_changes(course_change_count);
					ps_course_changes_i(1, ps) = course_changes(course_change_count);

					if (++course_change_count == course_changes.size()) { course_change_count = 0; }
				}	
			}
			std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
			std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl;
		}

		/****************************************************************************************
		*  Name     : initialize_independent_prediction
		*  Function : Sets up independent obstacle prediction.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void initialize_independent_prediction(
			const Obstacle_Data<Tracked_Obstacle> &data,						// In/Out: Dynamic obstacle information
			const int i, 														// In: Index of obstacle whose prediction to initialize
			const Eigen::VectorXd &ownship_state,								// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 												// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			Eigen::VectorXd t_cpa, d_cpa;
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
				v_os_0 = CPU::rotate_vector_2D(v_os_0, ownship_state(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = ownship_state.block<2, 1>(0, 0);
			xs_0(2) = v_os_0(0); xs_0(3) = v_os_0(1);
			Eigen::Matrix<double, 2, -1> waypoints_i;

			n_ps[i] = data.obstacles[i].get_scenario_probabilities().size();
			
			xs_i_0 = data.obstacles[i].kf.get_state();
			/* std::cout << "xs_i_0 = " << xs_i_0.transpose() << std::endl;
			std::cout << "xs_0 = " << xs_0.transpose() << std::endl; */
			CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);
			/* std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
			std::cout << "t_cpa(i) = " << t_cpa(i) << std::endl;
			std::cout << "d_cpa(i) = " << d_cpa(i)<< std::endl; */
			if (data.IP_0[i])
			{
				/* std::cout << "Obstacle i = " << i << "is passed => 1 PS only" << std::endl; */		
				ps_course_changes_i.resize(1);
				ps_course_changes_i[0] = 0;
				ps_maneuver_times_i.resize(1);
				ps_maneuver_times_i(0) = 0;
			}
			else
			{
				set_up_independent_obstacle_prediction(t_cpa(i), i, mpc);
			}
		}

		/****************************************************************************************
		*  Name     : predict_independent_trajectories
		*  Function : More refined obstacle prediction with avoidance-like trajectories,
		*			  including the straight-line trajectory
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <class MPC_Type>
		void predict_independent_trajectories(
			Obstacle_Data<Tracked_Obstacle> &data,				// In/Out: Dynamic obstacle information
			const int i,										// In: Index of obstacle whose trajectories to predict
			const Eigen::VectorXd &ownship_state, 				// In: Own-ship state, either [x, y, psi, u, v, r]^T or [x, y, chi, U]^T
			const MPC_Type &mpc 								// In: Calling MPC (either PSB-MPC or SB-MPC)
			)
		{
			int n_samples = std::round(mpc.pars.T / mpc.pars.dt);

			Eigen::Matrix4d P_0, P;
			Eigen::Matrix2d P_rot_2D;
			P_p_i.resize(16, n_samples);
			P_0 = data.obstacles[i].kf.get_covariance();
			P_p_i.col(0) = CPU::flatten(P_0);

			xs_p_i.resize(n_ps[i]);

			Eigen::Vector2d v_p, v_p_new;
			double chi_ps_ou(0.0), chi_ps(0.0), t(0.0);
			int turn_count(0);
			for(int ps = 0; ps < n_ps[i]; ps++)
			{
				xs_p_i[ps].resize(4, n_samples);
				xs_p_i[ps].col(0) = data.obstacles[i].kf.get_state();

				v_p(0) = data.obstacles[i].kf.get_state()(2);
				v_p(1) = data.obstacles[i].kf.get_state()(3);

				turn_count = 0;
				for(int k = 0; k < n_samples; k++)
				{
					t = k * mpc.pars.dt;

					// Starboard/port maneuver velocity rotation
					if (t == ps_maneuver_times_i(turn_count, ps) && ps != 0)
					{	
						chi_ps_ou = atan2(v_p(1), v_p(0)); 
						v_p_new(0) = v_p.norm() * cos(chi_ps_ou + ps_course_changes_i[ps]);
						v_p_new(1) = v_p.norm() * sin(chi_ps_ou + ps_course_changes_i[ps]);
						v_p = v_p_new;
						turn_count += 1;
					}

					if (k < n_samples - 1)
					{
						xs_p_i[ps].col(k + 1) = mrou.predict_state(xs_p_i[ps].col(k), v_p, mpc.pars.dt);

						chi_ps = atan2(xs_p_i[ps](3, k + 1), xs_p_i[ps](2, k + 1));
						if (ps == 0) 
						{
							P = mrou.predict_covariance(P_0, t + mpc.pars.dt);
							P_p_i.col(k + 1) = CPU::flatten(P);

							// Add constraint on cross-track variance here
							P_rot_2D = CPU::rotate_matrix_2D(P.block<2, 2>(0, 0), chi_ps);
							
							if (3 * sqrt(P_rot_2D(1, 1)) > r_ct)
							{
								std::cout << P_rot_2D << std::endl;
								P_rot_2D(1, 1) = pow(r_ct, 2) / 3.0;
								//P_p_i.col(k + 1) = 
							}
						}
					}
				}
			}
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
			n_ps.resize(n_obst);

			for (int i = 0; i < n_obst; i++)
			{
				initialize_independent_prediction(data, i, ownship_state, mpc);

				predict_independent_trajectories(data, i, ownship_state, mpc);

				// Transfer data to the tracked obstacle
				data.obstacles[i].set_trajectories(xs_p_i);
				data.obstacles[i].set_trajectory_covariance(P_p_i);

				// Calculate scenario probabilities using intention model
				// ..............
			}
		}
	};
}