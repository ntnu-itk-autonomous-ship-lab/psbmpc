/****************************************************************************************
*
*  File name : mpc_cost.hpp
*
*  Function  : Header file for the MPC Cost class (PSB/SB-MPC), which contains
*              implementations for the COLAV MPC cost functions.
*  
*            ---------------------
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

#include "obstacle_manager.hpp"
#include "cpu/prediction_obstacle_cpu.hpp"
#include "grounding_hazard_manager.hpp"
#include "engine.h"

namespace PSBMPC_LIB
{
	namespace CPU
	{
		template <typename Parameters>
		class MPC_Cost
		{
		private:

			Parameters pars;

			inline double Delta_u(const double u_1, const double u_2) const 		{ return pars.K_du * fabs(u_1 - u_2); } 

			inline double K_chi(const double chi) const								{ if (chi > 0) return pars.K_chi_strb * pow(chi, 2); else return pars.K_chi_port * pow(chi, 2); }

			inline double Delta_chi(const double chi_1, const double chi_2) const 	{ if (chi_1 > 0) return pars.K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return pars.K_dchi_port * pow(fabs(chi_1 - chi_2), 2); }

			// Grounding hazard related methods
			//===========================
			// Static obstacles as no-go lines

			int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const;                          

			bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const; 

			bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line) const;                        

			bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2) const;  

			double distance_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2) const;   

			double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) const;
			//===========================
		public:
			//===========================
			// Static obstacles as polygons
			bool determine_if_inside_polygon(const Eigen::Vector2d &p, const polygon_2D &poly) const;

			Eigen::Vector2d distance_to_line_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2) const;   

			Eigen::Vector2d distance_to_polygon(const Eigen::Vector2d &p, const polygon_2D &poly) const;
			

		

			MPC_Cost() {}

			MPC_Cost(const Parameters &pars) : pars(pars) {}

			template <class Obstacle_Data>
			bool determine_transitional_cost_indicator(
				const double psi_A, 
				const double psi_B, 
				const Eigen::Vector2d &L_AB, 
				const double chi_m,
				const Obstacle_Data &data,
				const int i) const;

			bool determine_COLREGS_violation(
				const Eigen::Vector2d &v_A, 
				const double psi_A, 
				const Eigen::Vector2d &v_B,
				const Eigen::Vector2d &L_AB, 
				const double d_AB) const;

			// PSBMPC, SBMPC and Obstacle_SBMPC dynamic obstacle cost, respectively
			// This one is used in the GPU PSBMPC on the host side
			std::tuple<double, double> calculate_dynamic_obstacle_cost(
				const Eigen::VectorXd &max_cost_ps, 
				const Eigen::VectorXd &mu_i_ps,
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i);

			// This one is used in the CPU PSBMPC for cost plotting
			std::tuple<double, double> calculate_dynamic_obstacle_cost(
				Eigen::VectorXd &max_cost_ps,
				Eigen::VectorXd &mu_i_ps,
				const Eigen::MatrixXd &trajectory, 
				const Eigen::MatrixXd &P_c_i, 
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;

			// Regular one used in the CPU PSBMPC
			std::tuple<double, double> calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory, 
				const Eigen::MatrixXd &P_c_i, 
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;
			
			// SBMPC version
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory, 
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;

			// Obstacle SBMPC version
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory, 
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const Obstacle_Data<Prediction_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;
			//

			inline double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) const { return pars.K_coll * pow((v_1 - v_2).norm(), 2); }

			double calculate_ad_hoc_collision_risk(const double d_AB, const double t) const;

			double calculate_control_deviation_cost(const Eigen::VectorXd &offset_sequence, const double u_m_last, const double chi_m_last) const;

			double calculate_chattering_cost(const Eigen::VectorXd &offset_sequence, const Eigen::VectorXd &maneuver_times) const;

			double calculate_grounding_cost(const Eigen::MatrixXd &trajectory, const Eigen::Matrix<double, 4, -1>& static_obstacles, const double ownship_length) const;
			double calculate_grounding_cost(
				const Eigen::MatrixXd &trajectory, 
				const std::vector<polygon_2D> &polygons, 
				const double V_w, 
				const Eigen::Vector2d &wind_direction) const;
		};

		/****************************************************************************************
		*  Name     : determine_transitional_cost_indicator
		*  Function : Determine if a transitional cost should be applied for the current
		*			  control behavior, using the method in Hagen, 2018.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		template <class Obstacle_Data>
		bool MPC_Cost<Parameters>::determine_transitional_cost_indicator(
			const double psi_A, 													// In: Heading of vessel A
			const double psi_B, 													// In: Heading of vessel B
			const Eigen::Vector2d &L_AB, 											// In: LOS vector pointing from vessel A to vessel B
			const double chi_m, 													// In: Candidate course offset currently followed
			const Obstacle_Data &data,												// In: Dynamic obstacle information
			const int i 															// In: Index of obstacle
			) const
		{
			bool S_TC(false), S_i_TC(false), O_TC(false), Q_TC(false), X_TC(false), H_TC(false);

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
		*  Name     : determine_COLREGS_violation
		*  Function : Determine if vessel A violates COLREGS with respect to vessel B.
		*			  
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_COLREGS_violation(
			const Eigen::Vector2d& v_A,												// In: (NE) Velocity vector of vessel A
			const double psi_A, 													// In: Heading of vessel A
			const Eigen::Vector2d& v_B, 											// In: (NE) Velocity vector of vessel B
			const Eigen::Vector2d& L_AB, 											// In: LOS vector pointing from vessel A to vessel B
			const double d_AB 														// In: Distance from vessel A to vessel B
			) const
		{
			bool B_is_starboard(false), A_is_overtaken(false), B_is_overtaken(false);
			bool is_ahead(false), is_passed(false), is_head_on(false), is_crossing(false);

			is_ahead = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

			bool is_close = d_AB <= pars.d_close;

			A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
							v_A.norm() < v_B.norm()							  	&&
							v_A.norm() > 0.25;

			B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
							v_B.norm() < v_A.norm()							  	&&
							v_B.norm() > 0.25;

			B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
						!A_is_overtaken) 											||
						(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
						!B_is_overtaken)) 											&&
						d_AB > pars.d_safe;

			is_head_on = v_A.dot(v_B) < - cos(pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() > 0.25											&&
						v_B.norm() > 0.25											&&
						is_ahead;

			is_crossing = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
						v_A.norm() > 0.25											&&
						v_B.norm() > 0.25											&&
						!is_head_on 												&&
						!is_passed;

			return is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken));
		}

		/****************************************************************************************
		*  Name     : calculate_dynamic_obstacle_cost
		*  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i
		*             Overloads depending on if its PSBMPC (GPU/CPU)/SBMPC/Obstacle_SBMPC. 
		*			  Also calculates the COLREGS penalty wrt obstacle i.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		std::tuple<double, double> MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::VectorXd &max_cost_i_ps,                       // In: Max cost wrt obstacle i in prediction scenario ps, and the control behaviour from the calling loop
			const Eigen::VectorXd &mu_i_ps,                       		// In: COLREGS violation indicator wrt obstacle i in prediction scenario ps, and the control behaviour from the calling loop
			const Obstacle_Data<Tracked_Obstacle> &data,				// In: Dynamic obstacle information
			const int i 												// In: Index of obstacle
			)
		{
			double cost_do(0.0), mu_i(0.0);

			Eigen::VectorXd Pr_s_i = data.obstacles[i].get_scenario_probabilities();
			assert(max_cost_i_ps.size() == Pr_s_i.size());

			cost_do = Pr_s_i.dot(max_cost_i_ps);
			mu_i = Pr_s_i.dot(mu_i_ps);

			/* std::cout << "max_cost_i_ps = " << max_cost_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_do = " << cost_do << std::endl; */

			return std::make_tuple<double, double>(std::move(cost_do), std::move(mu_i));
		}

		template <typename Parameters>
		std::tuple<double, double> MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			Eigen::VectorXd &max_cost_i_ps,								// In/Out: Max dynamic obstacle cost associated with the current control behaviour, wrt obstacle i in prediction scenario ps
			Eigen::VectorXd &mu_i_ps,									// In/Out: Indicator of COLREGS violation for the own-ship wrt the obstacle in all prediction scenarios 
			const Eigen::MatrixXd &trajectory,                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::MatrixXd &P_c_i,								// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Obstacle_Data<Tracked_Obstacle> &data,				// In: Dynamic obstacle information
			const int i, 												// In: Index of obstacle
			const double ownship_length                                 // In: Length of the ownship along the body x-axis
			) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost_do(0.0), cost_ps(0.0), mu_i(0.0), cost_coll(0.0), l_i(0.0);

			int n_samples = trajectory.cols();
			Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

			int n_ps = xs_i_p.size();

			max_cost_i_ps.resize(n_ps); 
			max_cost_i_ps.setZero();
			mu_i_ps.resize(n_ps);
			mu_i_ps.setZero();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), d_0i_p(0.0);
			bool mu(false);
			for(int k = 0; k < n_samples; k++)
			{
				if (trajectory.rows() == 4)
				{
					v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
					v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
					psi_0_p = trajectory(2, k);
				}
				else
				{
					psi_0_p = trajectory(2, k); 
					v_0_p(0) = trajectory(3, k); 
					v_0_p(1) = trajectory(4, k); 
					v_0_p = rotate_vector_2D(v_0_p, psi_0_p);
				}
				
				for(int ps = 0; ps < n_ps; ps++)
				{
					L_0i_p = xs_i_p[ps].block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
					d_0i_p = L_0i_p.norm();

					// Decrease the distance between the vessels by their respective max dimension
					d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + data.obstacles[i].get_length())); 

					L_0i_p.normalize();

					v_i_p(0) = xs_i_p[ps](2, k);
					v_i_p(1) = xs_i_p[ps](3, k);

					cost_coll = calculate_collision_cost(v_0_p, v_i_p);

					if (k > 0 && mu_i_ps(ps) < 0.1)
					{
						mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);
						if (mu) { mu_i_ps(ps) = 1.0; }
						else 	{ mu_i_ps(ps) = 0.0; }
					}

					// Track loss modifier to collision cost
					if (data.obstacles[i].get_duration_lost() > pars.p_step)
					{
						l_i = pars.dt * pars.p_step / data.obstacles[i].get_duration_lost();
					} 
					else
					{
						l_i = 1;
					}

					// PSB-MPC formulation with probabilistic collision cost
					cost_ps = l_i * cost_coll * P_c_i(ps, k);

					// Maximize wrt time
					if (cost_ps > max_cost_i_ps(ps))
					{
						max_cost_i_ps(ps) = cost_ps;
					}
					if (ps == 1)
						printf("k = %d | C = %.4f | P_c_i = %.6f | mu = %d | v_i_p = %.2f, %.2f | psi_0_p = %.2f | v_0_p = %.2f, %.2f | d_0i_p = %.2f | L_0i_p = %.2f, %.2f\n", 
							k, cost_coll, P_c_i(ps, k), mu, v_i_p(0), v_i_p(1), psi_0_p, v_0_p(0), v_0_p(1), d_0i_p, L_0i_p(0), L_0i_p(1));
				}
			}
			
			Eigen::VectorXd Pr_s_i = data.obstacles[i].get_scenario_probabilities();
			assert(Pr_s_i.size() == max_cost_i_ps.size());

			// Weight prediction scenario costs by the scenario probabilities
			cost_do = Pr_s_i.dot(max_cost_i_ps);

			mu_i = Pr_s_i.dot(mu_i_ps);
			/* 
			std::cout << "max_cost_i_ps = " << max_cost_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_i(i) = " << cost_do << std::endl; */

			return std::make_tuple<double, double>(std::move(cost_do), std::move(mu_i));
		}

		template <typename Parameters>
		std::tuple<double, double> MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::MatrixXd &P_c_i,								// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Obstacle_Data<Tracked_Obstacle> &data,				// In: Dynamic obstacle information
			const int i, 												// In: Index of obstacle
			const double ownship_length                                 // In: Length of the ownship along the body x-axis
			) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost_do(0.0), cost_ps(0.0), mu_i(0.0), C(0.0), l_i(0.0);

			int n_samples = trajectory.cols();
			Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

			int n_ps = xs_i_p.size();
			Eigen::VectorXd max_cost_i_ps(n_ps), mu_i_ps(n_ps);
			max_cost_i_ps.setZero();
			mu_i_ps.setZero();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), d_0i_p(0.0);
			bool mu;
			for(int k = 0; k < n_samples; k++)
			{
				if (trajectory.rows() == 4)
				{
					v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
					v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
					psi_0_p = trajectory(2, k);
				}
				else
				{
					psi_0_p = trajectory(2, k); 
					v_0_p(0) = trajectory(3, k); 
					v_0_p(1) = trajectory(4, k); 
					v_0_p = rotate_vector_2D(v_0_p, psi_0_p);
				}
				
				for(int ps = 0; ps < n_ps; ps++)
				{
					L_0i_p = xs_i_p[ps].block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
					d_0i_p = L_0i_p.norm();

					// Decrease the distance between the vessels by their respective max dimension
					d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + data.obstacles[i].get_length())); 

					L_0i_p = L_0i_p.normalized();

					v_i_p(0) = xs_i_p[ps](2, k);
					v_i_p(1) = xs_i_p[ps](3, k);

					C = calculate_collision_cost(v_0_p, v_i_p);

					if (k > 0 && mu_i_ps(ps) < 0.1)
					{
						mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);
						if (mu) { mu_i_ps(ps) = 1.0; }
						else 	{ mu_i_ps(ps) = 0.0; }
					}

					// Track loss modifier to collision cost
					if (data.obstacles[i].get_duration_lost() > pars.p_step)
					{
						l_i = pars.dt * pars.p_step / data.obstacles[i].get_duration_lost();
					} 
					else
					{
						l_i = 1;
					}

					// PSB-MPC formulation with probabilistic collision cost
					cost_ps = l_i * C * P_c_i(ps, k);

					// Maximize wrt time
					if (cost_ps > max_cost_i_ps(ps))
					{
						max_cost_i_ps(ps) = cost_ps;
					}
				}
			}
			
			Eigen::VectorXd Pr_s_i = data.obstacles[i].get_scenario_probabilities();
			assert(Pr_s_i.size() == max_cost_i_ps.size());

			// Weight prediction scenario costs by the scenario probabilities
			cost_do = Pr_s_i.dot(max_cost_i_ps);

			mu_i = Pr_s_i.dot(mu_i_ps);
			/* 
			std::cout << "max_cost_i_ps = " << max_cost_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_i(i) = " << cost_do << std::endl; */

			return std::make_tuple<double, double>(std::move(cost_do), std::move(mu_i));
		}

		// SBMPC do cost
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::VectorXd &offset_sequence,                     // In: Offset sequence currently followed by the own-ship
			const Eigen::VectorXd &maneuver_times,                      // In: Time of each maneuver in the offset sequence
			const Obstacle_Data<Tracked_Obstacle> &data,				// In: Dynamic obstacle information
			const int i, 												// In: Index of obstacle
			const double ownship_length                                 // In: Length of the ownship along the body x-axis
			) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost(0.0), max_cost(0.0), C(0.0), l_i(0.0), R(0.0);

			int n_samples = trajectory.cols();

			std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), psi_i_p(0.0), d_0i_p(0.0), chi_m(0.0); //R(0.0);
			bool mu(false), trans(false);
			for(int k = 0; k < n_samples; k++)
			{
				if (trajectory.rows() == 4)
				{
					v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
					v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
					psi_0_p = trajectory(2, k);
				}
				else
				{
					psi_0_p = trajectory(2, k); 
					v_0_p(0) = trajectory(3, k); 
					v_0_p(1) = trajectory(4, k); 
					v_0_p = rotate_vector_2D(v_0_p, psi_0_p);
				}

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

				L_0i_p = xs_i_p[0].block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
				d_0i_p = L_0i_p.norm();

				// Decrease the distance between the vessels by their respective max dimension
				d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + data.obstacles[i].get_length())); 

				L_0i_p = L_0i_p.normalized();

				v_i_p(0) = xs_i_p[0](2, k);
				v_i_p(1) = xs_i_p[0](3, k);
				psi_i_p = atan2(v_i_p(1), v_i_p(0));

				C = calculate_collision_cost(v_0_p, v_i_p);

				if (k > 0)
				{
					mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);
				}

				trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, chi_m, data, i);

				R = calculate_ad_hoc_collision_risk(d_0i_p, (k + 1) * pars.dt);

				// Track loss modifier to collision cost
				if (data.obstacles[i].get_duration_lost() > pars.p_step)
				{
					l_i = pars.dt * pars.p_step / data.obstacles[i].get_duration_lost();
				} else
				{
					l_i = 1;
				}
				
				// SB-MPC formulation with ad-hoc collision risk
				cost = l_i * C * R + pars.kappa * mu  + pars.kappa_TC * trans;

				if (cost > max_cost)
				{
					max_cost = cost;
				}
			}
			return max_cost;
		}

		// Obstacle SBMPC do cost
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::VectorXd &offset_sequence,                     // In: Offset sequence currently followed by the own-ship
			const Eigen::VectorXd &maneuver_times,                      // In: Time of each maneuver in the offset sequence
			const Obstacle_Data<Prediction_Obstacle> &data,				// In: Dynamic obstacle information
			const int i, 												// In: Index of obstacle
			const double ownship_length                                 // In: Length of the ownship along the body x-axis
			) const
		{
			double cost(0.0), max_cost(0.0), C(0.0), R(0.0);

			int n_samples = trajectory.cols();

			Eigen::MatrixXd xs_i_p = data.obstacles[i].get_predicted_trajectory();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), psi_i_p(0.0), d_0i_p(0.0), chi_m(0.0);
			bool mu(false), trans(false);
			for(int k = 0; k < n_samples; k++)
			{
				if (trajectory.rows() == 4)
				{
					v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
					v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
					psi_0_p = trajectory(2, k);
				}
				else
				{
					psi_0_p = trajectory(2, k); 
					v_0_p(0) = trajectory(3, k); 
					v_0_p(1) = trajectory(4, k); 
					v_0_p = rotate_vector_2D(v_0_p, psi_0_p);
				}

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
				d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + data.obstacles[i].get_length())); 

				L_0i_p = L_0i_p.normalized();

				v_i_p(0) = xs_i_p(2, k);
				v_i_p(1) = xs_i_p(3, k);
				psi_i_p = atan2(v_i_p(1), v_i_p(0));

				C = calculate_collision_cost(v_0_p, v_i_p);

				if (k > 0)
				{
					mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);
				}

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
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_ad_hoc_collision_risk(
			const double d_AB, 											// In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
																		// 	   reduced by half the length of the two vessels (or only own-ship if static obstacles are considered)
			const double t 												// In: Prediction time t > t0 (= 0)
			) const
		{
			double R = 0;
			if (d_AB <= pars.d_safe)
			{
				assert(t > 0);
				R = pow(pars.d_safe / d_AB, pars.q) * (1 / pow(fabs(t), pars.p)); 
			}
			return R;
		}

		/****************************************************************************************
		*  Name     : calculate_control_deviation_cost
		*  Function : Determines penalty due to using offsets to guidance references ++
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_control_deviation_cost(
			const Eigen::VectorXd &offset_sequence,                         // In: Offset_sequence currently followed by the own-ship
			const double u_m_last,                                          // In: Previous optimal output surge modification
			const double chi_m_last                                         // In: Previous optimal output course modification
			) const
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
			/* printf("K_u (1 - u_m_0) = %.4f | Delta_u(u_m_0, u_m_last) = %.4f | K_chi(chi_0) = %.4f | Delta_chi(chi_0, chi_last) = %.4f\n", 
				pars.K_u * (1 - offset_sequence[0]), Delta_u(offset_sequence[0], u_m_last), K_chi(offset_sequence[1]), Delta_chi(offset_sequence[1], chi_m_last)); */

			return cost / (double)pars.n_M;
		}

		/****************************************************************************************
		*  Name     : calculate_chattering_cost
		*  Function : Determines penalty due to using wobbly (changing between positive and negative)
		* 			  course modifications
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_chattering_cost(
			const Eigen::VectorXd &offset_sequence,                         // In: Offset sequence currently followed by the own-ship
			const Eigen::VectorXd &maneuver_times                           // In: Time of each maneuver in the offset sequence
			) const
		{
			double cost = 0.0;

			if (pars.n_M == 1) 
			{
				return cost;
			}

			double delta_t = 0.0;
			for(int M = 0; M < pars.n_M; M++)
			{
				if (M < pars.n_M - 1)
				{
					if ((offset_sequence(2 * M + 1) > 0 && offset_sequence(2 * M + 3) < 0) ||
						(offset_sequence(2 * M + 1) < 0 && offset_sequence(2 * M + 3) > 0))
					{
						delta_t = maneuver_times(M + 1) - maneuver_times(M);
						cost += pars.K_sgn * exp( - delta_t / pars.T_sgn);
					}
				}
			}
			return cost / (double)(pars.n_M - 1);
		}

		/****************************************************************************************
		*  Name     : calculate_grounding_cost
		*  Function : Determines penalty due to grounding ownship on static obstacles (no-go zones)
		*  Author   : Trym Tengesdal & Tom Daniel Grande
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_grounding_cost(
			const Eigen::MatrixXd &trajectory,									// In: Predicted ownship trajectory
			const std::vector<polygon_2D> &polygons,							// In: Static obstacle information
			const double V_w,													// In: Estimated wind speed
			const Eigen::Vector2d &wind_direction 								// In: Unit vector in NE describing the estimated wind direction
			) const
		{
			int n_samples = std::round(pars.T / pars.dt);
			int n_static_obst = polygons.size();
			double max_cost_g(0.0), cost_g(0.0);	 

			Eigen::Vector2d L_0j;
			double d_0j(0.0), t(0.0), phi_j(0.0);
			for (int k = 0; k < n_samples; k++)
			{
				t = pars.dt * k;

				cost_g = 0.0;
				for (int j = 0; j < n_static_obst; j++)
				{
					L_0j = distance_to_polygon(trajectory.block<2, 1>(0, k), polygons[j]);
					d_0j = L_0j.norm();
					L_0j.normalize();

					phi_j = std::max(0.0, L_0j.dot(wind_direction));

					cost_g += (pars.G_1 + pars.G_2 * phi_j * pow(V_w, 2)) * exp(- (pars.G_3 * d_0j + pars.G_4 * t));
				}

				if (max_cost_g < cost_g)
				{
					max_cost_g = cost_g;
				}
			}
			return max_cost_g;
		}

		/****************************************************************************************
		*  Name     : calculate_grounding_cost
		*  Function : Determines penalty due grounding ownship on static obstacles (no-go zones)
		*  Author   : Trym Tengesdal & Giorgio D. Kwame Minde Kufoalor
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_grounding_cost(
			const Eigen::MatrixXd &trajectory,                                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::Matrix<double, 4, -1>& static_obstacles,						// In: Static obstacle information
			const double ownship_length                                                 // In: Length of the ownship along the body x-axis
			) const
		{
			double d_geo(0.0), t(0.0), g_cost(0.0); 
			int n_static_obst = static_obstacles.cols();
			int n_samples = std::round(pars.T / pars.dt);
			// so 1 and 2 : endpoints of line describing static obstacle
			Eigen::Vector2d p_0, p_1, so_1, so_2; 

			Eigen::VectorXd cost_j(n_static_obst);
			cost_j.setZero();

			// Check if it is necessary to calculate this cost
			bool is_geo_constraint = false;
			for (int j = 0; j < n_static_obst; j++)
			{
				p_0 << trajectory.block<2, 1>(0, 0);
				p_1 << trajectory.block<2, 1>(0, n_samples - 1);

				so_1 << static_obstacles.block<2, 1>(0, j);
				so_2 << static_obstacles.block<2, 1>(2, j);

				d_geo = distance_to_line(p_1, so_1, so_2);

				// Decrease distance by the half the own-ship length
				d_geo = d_geo - 0.5 * ownship_length;

				if (!is_geo_constraint)
				{
					is_geo_constraint = determine_if_lines_intersect(p_0, p_1, so_1, so_2) || determine_if_behind(p_1, so_1, so_2, d_geo);
					break;
				}
			}

			if (n_static_obst == 0 || !is_geo_constraint)
			{
				return 0.0;
			}
			
			for (int k = 0; k < n_samples - 1; k++)
			{
				t = (k + 1) * pars.dt;

				for (int j = 0; j < n_static_obst; j++)
				{
					p_0 << trajectory.block<2, 1>(0, k);

					so_1 << static_obstacles.block<2, 1>(0, j);
					so_2 << static_obstacles.block<2, 1>(2, j);

					d_geo = distance_to_static_obstacle(p_0, so_1, so_2);

					// Decrease distance by the half the own-ship length
					d_geo = d_geo - 0.5 * ownship_length;

					g_cost = pars.G * calculate_ad_hoc_collision_risk(d_geo, t);

					// Maximize wrt time
					if (g_cost > cost_j(j))
					{
						cost_j(j) = g_cost;
					}
				}
			}

			// Return maximum wrt the present static obstacles
			return cost_j.maxCoeff();
		}

		/****************************************************************************************
			Private functions
		****************************************************************************************/
		/****************************************************************************************
		*  Name     : find_triplet_orientation
		*  Function : Find orientation of ordered triplet (p, q, r)
		*  Author   : 
		*  Modified : 
		*****************************************************************************************/
		template <typename Parameters>
		int MPC_Cost<Parameters>::find_triplet_orientation(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q, 
			const Eigen::Vector2d &r
			) const
		{
			double epsilon = 1e-12; // abs(val) less than 1e-12 m^2 is considered zero for this check
			// Calculate z-component of cross product (q - p) x (r - q)
			double val = (q(0) - p(0)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(0) - q(0));

			//printf("p = %.6f, %.6f | q = %.6f, %.6f | r = %.6f, %.6f | val = %.15f\n", p(0), p(1), q(0), q(1), r(0), r(1), val);
			if (val >= -epsilon && val <= epsilon) 	{ return 0; } // colinear
			else if (val > epsilon) 				{ return 1; } // clockwise
			else 									{ return 2; } // counterclockwise
		}

		/****************************************************************************************
		*  Name     : determine_if_on_segment
		*  Function : Determine if the point q is on the segment pr
		*			  (really if q is inside the rectangle with diagonal pr...)
		*  Author   : 
		*  Modified : 
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_if_on_segment(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q, 
			const Eigen::Vector2d &r
			) const
		{
			if (q(0) <= std::max(p(0), r(0)) && q(0) >= std::min(p(0), r(0)) &&
				q(1) <= std::max(p(1), r(1)) && q(1) >= std::min(p(1), r(1)))
			{
				return true;
			}
			return false;
		}

		/****************************************************************************************
		*  Name     : determine_if_behind
		*  Function : Check if the point p_1 is behind the line defined by v_1 and v_2
		*  Author   : Giorgio D. Kwame Minde Kufoalor
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_if_behind(
			const Eigen::Vector2d &p_1, 
			const Eigen::Vector2d &v_1, 
			const Eigen::Vector2d &v_2, 
			const double distance_to_line
			) const
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
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_if_lines_intersect(
			const Eigen::Vector2d &p_1, 
			const Eigen::Vector2d &q_1, 
			const Eigen::Vector2d &p_2, 
			const Eigen::Vector2d &q_2
			) const
		{
			// Find the four orientations needed for general and
			// special cases
			int o_1 = find_triplet_orientation(p_1, q_1, p_2);
			int o_2 = find_triplet_orientation(p_1, q_1, q_2);
			int o_3 = find_triplet_orientation(p_2, q_2, p_1);
			int o_4 = find_triplet_orientation(p_2, q_2, q_1);

			//printf("o_1 = %d | o_2 = %d | o_3 = %d | o_4 = %d\n", o_1, o_2, o_3, o_4);
			// General case
			if (o_1 != o_2 && o_3 != o_4) { return true; }

			// Special Cases
			// p_1, q_1 and p_2 are colinear and p_2 lies on segment p_1 -> q_1
			if (o_1 == 0 && determine_if_on_segment(p_1, p_2, q_1)) { return true; }

			// p_1, q_1 and q_2 are colinear and q_2 lies on segment p_1 -> q_1
			if (o_2 == 0 && determine_if_on_segment(p_1, q_2, q_1)) { return true; }

			// p_2, q_2 and p_1 are colinear and p_1 lies on segment p_2 -> q_2
			if (o_3 == 0 && determine_if_on_segment(p_2, p_1, q_2)) { return true; }

			// p_2, q_2 and q_1 are colinear and q_1 lies on segment p_2 -> q_2
			if (o_4 == 0 && determine_if_on_segment(p_2, q_1, q_2)) { return true; }

			return false; // Doesn't fall in any of the above cases
		}

		/****************************************************************************************
		*  Name     : distance_to_line
		*  Function : Calculate distance from p to the line defined by q_1 and q_2
		*  Author   : Giorgio D. Kwame Minde Kufoalor
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::distance_to_line(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q_1, 
			const Eigen::Vector2d &q_2
			) const
		{   
			Eigen::Vector3d a;
			Eigen::Vector3d b;
			a << (q_1 - q_2), 0.0;
			b << (p - q_2), 0.0;

			Eigen::Vector3d c = a.cross(b);
			if (a.norm() > 0) return c.norm() / a.norm();
			else return -1;
		}

		/****************************************************************************************
		*  Name     : distance_to_static_obstacle
		*  Function : Calculate distance from p to obstacle defined by line segment {v_1, v_2}
		*  Author   : Giorgio D. Kwame Minde Kufoalor
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::distance_to_static_obstacle(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &v_1, 
			const Eigen::Vector2d &v_2
			) const
		{
			double d2line = distance_to_line(p, v_1, v_2);

			if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
			else return std::min((v_1 - p).norm(),(v_2 - p).norm());
		}

		/****************************************************************************************
		*  Name     : determine_if_inside_polygon
		*  Function : 
		*  Author   : Trym Tengesdal
		*  Modified : 
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_if_inside_polygon(
			const Eigen::Vector2d &p, 
			const polygon_2D &poly
			) const
		{
			int line_intersect_count = 0, n_vertices(0);
			Eigen::Vector2d v, v_next;			

			// Find bounding box of polygon to use for ray creation
			Eigen::Matrix2d bbox;
			bbox(0, 0) = 1e10; bbox(1, 0) = 1e10; bbox(0, 1) = -1e10; bbox(1, 1) = -1e10;
			for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); it++)
			{
				v(0) = boost::geometry::get<0>(*it); v(1) = boost::geometry::get<1>(*it);
				if (v(0) < bbox(0, 0)) { bbox(0, 0) = v(0); } // x_min
				if (v(1) < bbox(1, 0)) { bbox(1, 0) = v(1); } // y_min
				if (v(0) > bbox(0, 1)) { bbox(0, 1) = v(0); } // x_max
				if (v(1) > bbox(1, 1)) { bbox(1, 1) = v(1); } // y_max
				n_vertices += 1;
			}
			if (n_vertices < 3) { return false; }

			Eigen::Vector2d p_ray_end = p + 1.1 * (bbox.col(1) - p);
			int v_count = 0;
			for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
			{
				v(0) = boost::geometry::get<0>(*it); v(1) = boost::geometry::get<1>(*it);
				v_next(0) = boost::geometry::get<0>(*(it + 1)); v_next(1) = boost::geometry::get<1>(*(it + 1));
				
				if (determine_if_lines_intersect(p, p_ray_end, v, v_next))
				{
					// Special case when p is colinear with line segment from v -> v_next
					if (find_triplet_orientation(v, p, v_next) == 0)
					{
						return determine_if_on_segment(v, p, v_next);
					}
					line_intersect_count += 1;
				}
				v_count += 1;
			}
			return line_intersect_count % 2 == 1;
		}

		/****************************************************************************************
		*  Name     : distance_to_line_segment
		*  Function : Calculate distance from p to line segment {v_1, v_2}
		*  Author   : Trym Tengesdal
		*  Modified : 
		*****************************************************************************************/
		template <typename Parameters>
		Eigen::Vector2d MPC_Cost<Parameters>::distance_to_line_segment(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q_1, 
			const Eigen::Vector2d &q_2
			) const
		{
			double epsilon = 0.00001, l_sqrt(0.0), t_line(0.0);
			Eigen::Vector3d a, b;
			Eigen::Vector2d projection;
			a << (q_2 - q_1), 0.0;
			b << (p - q_1), 0.0;

			l_sqrt = a(0) * a(0) + a(1) * a(1);
			if (l_sqrt <= epsilon)	{ return q_1 - p; }

			t_line = std::max(0.0, std::min(1.0, a.dot(b) / l_sqrt));
			projection = q_1 + t_line * (q_2 - q_1);

			return projection - p;
		}

		/****************************************************************************************
		*  Name     : distance_to_polygon
		*  Function : Calculate distance vector from p to polygon
		*  Author   : Trym Tengesdal
		*  Modified : 
		*****************************************************************************************/
		template <typename Parameters>
		Eigen::Vector2d MPC_Cost<Parameters>::distance_to_polygon(
			const Eigen::Vector2d &p, 
			const polygon_2D &poly
			) const
		{
			Eigen::Vector2d d2poly, d2line;
			if (determine_if_inside_polygon(p, poly))
			{
				d2poly(0) = 0.0f; d2poly(1) = 0.0f;
				return d2poly;
			}
			d2poly(0) = 1e10; d2poly(1) = 1e10;
			Eigen::Vector2d v, v_next;
			for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
			{
				v(0) = boost::geometry::get<0>(*it); v(1) = boost::geometry::get<1>(*it);
				v_next(0) = boost::geometry::get<0>(*(it + 1)); v_next(1) = boost::geometry::get<1>(*(it + 1));

				d2line = distance_to_line_segment(p, v, v_next);
				
				if (d2line.norm() < d2poly.norm())
				{
					d2poly = d2line;
				}
			}
			return d2poly;
		}
	}
}