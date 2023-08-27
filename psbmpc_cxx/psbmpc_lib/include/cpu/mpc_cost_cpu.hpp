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
#include "psbmpc_defines.hpp"
#include "obstacle_manager.hpp"
#include "grounding_hazard_manager.hpp"
#include "colregs_violation_evaluator.hpp"

#if ENABLE_PSBMPC_DEBUGGING
#include <engine.h>
#endif
#include <map>
#include <vector>
#include <Eigen/Dense>

namespace PSBMPC_LIB
{
	// Transitional variables are used  by the SBMPC to implement the transitional cost in Hagen, 2018.
	struct Transitional_Variables
	{
		// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
		// and <obstacle is passed> (IP_0) indicators
		std::vector<bool> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

		void clear()
		{
			AH_0.clear();
			S_TC_0.clear();
			S_i_TC_0.clear();
			O_TC_0.clear();
			Q_TC_0.clear();
			IP_0.clear();
			H_TC_0.clear();
			X_TC_0.clear();
		}

		Transitional_Variables() {}
	};
	namespace CPU
	{
		template <typename Parameters>
		class MPC_Cost
		{
		private:
			Parameters pars;

			std::map<int, COLREGS_Violation_Evaluator> colregs_violation_evaluators;

			inline double Delta_u(const double u_1, const double u_2) const { return pars.K_du * fabs(u_1 - u_2); }

			// SBMPC delta chi funcs in path deviation cost
			inline double K_chi(const double chi) const
			{
				if (chi > 0)
					return pars.K_chi_strb * pow(chi, 2);
				else
					return pars.K_chi_port * pow(chi, 2);
			}

			inline double Delta_chi(const double chi_1, const double chi_2) const
			{
				if (chi_1 > 0)
					return pars.K_dchi_strb * pow(wrap_angle_to_pmpi(chi_1 - chi_2), 2);
				else
					return pars.K_dchi_port * pow(wrap_angle_to_pmpi(chi_1 - chi_2), 2);
			}
			//

			int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const;

			bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const;

			bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line) const;

			bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2) const;

			double distance_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2) const;

			double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) const;

		public:
			bool determine_if_inside_polygon(const Eigen::Vector2d &p, const polygon_2D &poly) const;

			Eigen::Vector2d distance_to_line_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2) const;

			Eigen::Vector2d distance_to_polygon(const Eigen::Vector2d &p, const polygon_2D &poly) const;

			MPC_Cost() {}

			MPC_Cost(const Parameters &pars) : pars(pars) {}

			/* MPC_Cost(const Parameters &pars, const std::map<int, COLREGS_Violation_Evaluator> colregs_violation_evaluators)
				: pars(pars), colregs_violation_evaluators(colregs_violation_evaluators) {} */

			MPC_Cost &operator=(const MPC_Cost &other) = default;

			bool determine_transitional_cost_indicator(
				const Transitional_Variables &tv,
				const double psi_A,
				const double psi_B,
				const Eigen::Vector2d &L_AB,
				const double chi_m,
				const int i) const;

			bool determine_COLREGS_violation(
				const Eigen::Vector2d &v_A,
				const double psi_A,
				const Eigen::Vector2d &v_B,
				const Eigen::Vector2d &L_AB,
				const double d_AB) const;

			// PSBMPC and SBMPC dynamic obstacle cost, respectively
			// This one is used in the GPU PSBMPC on the host side
			double calculate_dynamic_obstacle_cost(
				const Eigen::VectorXd &h_do_i_ps,
				const Dynamic_Obstacles &obstacles,
				const int i);

			// This one is used in the CPU PSBMPC for cost plotting
			double calculate_dynamic_obstacle_cost(
				Eigen::VectorXd &h_do_i_ps,
				const Eigen::MatrixXd &trajectory,
				const Eigen::MatrixXd &P_c_i,
				const Dynamic_Obstacles &obstacles,
				const int i,
				const double ownship_length) const;

			// CPU PSBMPC cost
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory,
				const Eigen::MatrixXd &P_c_i,
				const Dynamic_Obstacles &obstacles,
				const int i,
				const double ownship_length) const;

			// SBMPC version
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory,
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const Dynamic_Obstacles &obstacles,
				const Transitional_Variables &tv,
				const int i,
				const double ownship_length) const;

			inline double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) const { return pars.K_coll * pow((v_1 - v_2).norm(), 2); }

			double calculate_ad_hoc_collision_risk(const double d_AB, const double t) const;

			double calculate_control_deviation_cost(
				const Eigen::VectorXd &offset_sequence,
				const double u_m_last,
				const double chi_m_last) const;

			double calculate_control_deviation_cost(
				const Eigen::VectorXd &offset_sequence,
				const double u_m_last,
				const double chi_m_last,
				const double max_cross_track_error) const;

			double calculate_grounding_cost(const Eigen::MatrixXd &trajectory, const Eigen::Matrix<double, 4, -1> &static_obstacles, const double ownship_length) const;
			double calculate_grounding_cost(
				const Eigen::MatrixXd &trajectory,
				const Static_Obstacles &polygons,
				const double V_w,
				const Eigen::Vector2d &wind_direction) const;

			double calculate_grounding_cost(
				Eigen::VectorXd &max_cost_j,
				const Eigen::MatrixXd &trajectory,
				const Static_Obstacles &polygons,
				const double V_w,
				const Eigen::Vector2d &wind_direction) const;

			void update_colregs_violation_node(
				const Eigen::Vector4d &ownship_state,
				const Eigen::Vector4d &obstacle_state,
				const int i)
			{
				colregs_violation_evaluators[i].update(ownship_state, obstacle_state);
			}

			double calculate_colregs_violation_cost(
				const Eigen::MatrixXd &ownship_trajectory,
				const Eigen::MatrixXd &obstacle_trajectory,
				const Eigen::VectorXd &offset_sequence,
				const double &Pr_WGW,
				const double &Pr_CCEM,
				const int i)
			{
				colregs_violation_evaluators.at(i).reset();
				double d_0i_0 = evaluateDistance(ownship_trajectory.col(0), obstacle_trajectory.col(0)); // In: Distance at the current time t_0 between ownship and obstacle i
				double d_cpa = INFINITY;																 // In: Distance at predicted CPA between ownship and obstacle i
				Eigen::Vector4d ownship_CPA_state;														 // In: Ownship state at CPA
				Eigen::Vector4d obstacle_CPA_state_vx_vy;												 // In: Obstacle i state at CPA

				for (auto t = 0; t < std::min(ownship_trajectory.cols(), obstacle_trajectory.cols()); ++t)
				{
					const double current_distance = evaluateDistance(ownship_trajectory.col(t), obstacle_trajectory.col(t));
					if (current_distance < d_cpa)
					{
						d_cpa = current_distance;
						ownship_CPA_state = ownship_trajectory.col(t);
						obstacle_CPA_state_vx_vy = obstacle_trajectory.col(t);
					}
					colregs_violation_evaluators.at(i).evaluate_predicted_maneuver_changes(ownship_trajectory(2, t), ownship_trajectory(3, t));
				}

				if (colregs_violation_evaluators.count(i))
				{
					bool so_violation = colregs_violation_evaluators.at(i).evaluate_SO_violation(d_0i_0);
					bool gw_violation = colregs_violation_evaluators.at(i).evaluate_GW_violation(ownship_CPA_state, obstacle_CPA_state_vx_vy, d_cpa);
					bool readily_apparant_violation = colregs_violation_evaluators.at(i).evaluate_readily_apparent_violation(offset_sequence[1], offset_sequence[0]);
					return pars.kappa_SO * so_violation * Pr_WGW + pars.kappa_GW * gw_violation * Pr_CCEM + pars.kappa_RA * readily_apparant_violation;
				}
				else
				{
					throw std::runtime_error("Attempting to evaluate colregs violation for an uninitialized ship");
					// Run colregs_violation_evaluators to fix
				}
			}

			double calculate_colregs_violation_cost(
				const Eigen::MatrixXd &ownship_trajectory,
				const Dynamic_Obstacles &obstacles,
				const Eigen::VectorXd &offset_sequence)
			{
				double total_cost = 0;
				for (const auto &obstacle : obstacles)
				{
					update_colregs_violation_node(ownship_trajectory.col(0), obstacle.get_trajectories()[0].col(0), obstacle.get_ID());

					const auto trajs = obstacle.get_trajectories();
					const auto probabilities = obstacle.get_scenario_probabilities();
					const double Pr_WGW = obstacle.get_Pr_WGW();
					const double Pr_CCEM = obstacle.get_Pr_CCEM();
					for (size_t i = 0; i < trajs.size(); ++i)
					{
						total_cost += probabilities(i) * calculate_colregs_violation_cost(ownship_trajectory, trajs[i], offset_sequence, Pr_WGW, Pr_CCEM, obstacle.get_ID());
					}
				}
				return total_cost;
			}

			double calculate_colregs_violation_cost(
				const Eigen::VectorXd &h_colregs_i_ps,
				const Dynamic_Obstacles &obstacles,
				const int i) const
			{
				Eigen::VectorXd Pr_s_i = obstacles[i].get_scenario_probabilities();
				assert(Pr_s_i.size() == h_colregs_i_ps.size());
				return Pr_s_i.dot(h_colregs_i_ps);
			}
		};

		/****************************************************************************************
		 *  Name     : determine_transitional_cost_indicator
		 *  Function : Determine if a transitional cost should be applied for the current
		 *			  control behavior, using the method in Hagen, 2018. Used in the SBMPC
		 *  Author   : Trym Tengesdal
		 *  Modified :
		 *****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_transitional_cost_indicator(
			const Transitional_Variables &tv, // In: Struct of all current time obstacle transitional variables
			const double psi_A,				  // In: Heading of vessel A
			const double psi_B,				  // In: Heading of vessel B
			const Eigen::Vector2d &L_AB,	  // In: LOS vector pointing from vessel A to vessel B
			const double chi_m,				  // In: Candidate course offset currently followed
			const int i						  // In: Index of obstacle
		) const
		{
			bool S_TC(false), S_i_TC(false), O_TC(false), Q_TC(false), X_TC(false), H_TC(false);

			// Obstacle on starboard side
			S_TC = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			// Ownship on starboard side of obstacle
			S_i_TC = angle_difference_pmpi(atan2(-L_AB(1), -L_AB(0)), psi_B) > 0;

			// For ownship overtaking the obstacle: Check if obstacle is on opposite side of
			// ownship to what was observed at t0
			if (!tv.S_TC_0[i])
			{
				O_TC = tv.O_TC_0[i] && S_TC;
			}
			else
			{
				O_TC = tv.O_TC_0[i] && !S_TC;
			};

			// For obstacle overtaking the ownship: Check if ownship is on opposite side of
			// obstacle to what was observed at t0
			if (!tv.S_i_TC_0[i])
			{
				Q_TC = tv.Q_TC_0[i] && S_i_TC;
			}
			else
			{
				Q_TC = tv.Q_TC_0[i] && !S_i_TC;
			};

			// For crossing: Check if obstacle is on opposite side of ownship to what was
			// observed at t0
			X_TC = tv.X_TC_0[i] && tv.S_TC_0[i] && S_TC && (chi_m < 0);

			// This is not mentioned in article, but also implemented here..
			// Transitional cost only valid by going from having obstacle on port side at
			// t0, to starboard side at time t
			if (!tv.S_TC_0[i])
			{
				H_TC = tv.H_TC_0[i] && S_TC;
			}
			else
			{
				H_TC = false;
			}
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
			const Eigen::Vector2d &v_A,	 // In: (NE) Velocity vector of vessel A
			const double psi_A,			 // In: Heading of vessel A
			const Eigen::Vector2d &v_B,	 // In: (NE) Velocity vector of vessel B
			const Eigen::Vector2d &L_AB, // In: LOS vector pointing from vessel A to vessel B
			const double d_AB			 // In: Distance from vessel A to vessel B
		) const
		{
			bool B_is_starboard(false), A_is_overtaken(false), B_is_overtaken(false);
			bool is_ahead(false), is_passed(false), is_head_on(false), is_crossing(false);

			is_ahead = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

			bool is_close = d_AB <= pars.d_close;

			A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() &&
							 v_A.norm() < v_B.norm() &&
							 v_A.norm() > 0.25;

			B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() &&
							 v_B.norm() < v_A.norm() &&
							 v_B.norm() > 0.25;

			B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm() && // Vessel A's perspective
						  !A_is_overtaken) ||
						 (v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() && // Vessel B's perspective
						  !B_is_overtaken)) &&
						d_AB > pars.d_safe;

			is_head_on = v_A.dot(v_B) < -cos(pars.phi_HO) * v_A.norm() * v_B.norm() &&
						 v_A.norm() > 0.25 &&
						 v_B.norm() > 0.25 &&
						 is_ahead;

			is_crossing = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm() &&
						  v_A.norm() > 0.25 &&
						  v_B.norm() > 0.25 &&
						  !is_head_on &&
						  !is_passed;

			return is_close && ((B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken));
		}

		/****************************************************************************************
		 *  Name     : calculate_dynamic_obstacle_cost
		 *  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i
		 *             Overloads depending on if its PSBMPC (GPU/CPU)/SBMPC.
		 *  Author   : Trym Tengesdal
		 *  Modified :
		 *****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::VectorXd &h_do_i_ps,	// In: Max cost wrt obstacle i in prediction scenario ps, and the control behaviour from the calling loop
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int i							// In: Index of obstacle
		)
		{
			double cost_do(0.0);

			Eigen::VectorXd Pr_s_i = obstacles[i].get_scenario_probabilities();
			assert(h_do_i_ps.size() == Pr_s_i.size());

			cost_do = Pr_s_i.dot(h_do_i_ps);

			/* std::cout << "h_do_i_ps = " << h_do_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_do = " << cost_do << std::endl; */

			return cost_do;
		}

		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			Eigen::VectorXd &h_do_i_ps,			// In/Out: Max dynamic obstacle cost associated with the current control behaviour, wrt obstacle i in prediction scenario ps
			const Eigen::MatrixXd &trajectory,	// In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::MatrixXd &P_c_i,		// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int i,						// In: Index of obstacle
			const double ownship_length			// In: Length of the ownship along the body x-axis
		) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost_do(0.0), cost_ps(0.0), cost_coll(0.0), l_i(0.0);

			int n_samples = trajectory.cols();
			Eigen::MatrixXd P_i_p = obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

			int n_ps = xs_i_p.size();

			h_do_i_ps.resize(n_ps);
			h_do_i_ps.setZero();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), d_0i_p(0.0);
			for (int k = 0; k < n_samples; k++)
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

				for (int ps = 0; ps < n_ps; ps++)
				{
					L_0i_p = xs_i_p[ps].block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
					d_0i_p = L_0i_p.norm();

					// Decrease the distance between the vessels by their respective max dimension
					d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + obstacles[i].get_length()));

					L_0i_p.normalize();

					v_i_p(0) = xs_i_p[ps](2, k);
					v_i_p(1) = xs_i_p[ps](3, k);

					cost_coll = calculate_collision_cost(v_0_p, v_i_p);

					// Track loss modifier to collision cost
					if (obstacles[i].get_duration_lost() > pars.p_step_do)
					{
						l_i = pars.dt * pars.p_step_do / obstacles[i].get_duration_lost();
					}
					else
					{
						l_i = 1;
					}

					// PSB-MPC formulation with probabilistic collision cost
					// cost_ps = l_i * cost_coll * P_c_i(ps, k);

					// Should discount time when using a prediction scheme where the uncertainty for
					// each obstacle prediction scenario is bounded by r_ct
					cost_ps = l_i * cost_coll * P_c_i(ps, k) * exp(-(double)k * pars.dt / pars.T_coll);

					// Maximize wrt time
					if (cost_ps > h_do_i_ps(ps))
					{
						h_do_i_ps(ps) = cost_ps;
					}
					/* if (ps == 1)
						printf("k = %d | C = %.4f | P_c_i = %.6f | mu = %d | v_i_p = %.2f, %.2f | psi_0_p = %.2f | v_0_p = %.2f, %.2f | d_0i_p = %.2f | L_0i_p = %.2f, %.2f\n",
							k, cost_coll, P_c_i(ps, k), mu, v_i_p(0), v_i_p(1), psi_0_p, v_0_p(0), v_0_p(1), d_0i_p, L_0i_p(0), L_0i_p(1)); */
				}
			}

			Eigen::VectorXd Pr_s_i = obstacles[i].get_scenario_probabilities();
			assert(Pr_s_i.size() == h_do_i_ps.size());

			// Weight prediction scenario costs by the scenario probabilities
			cost_do = Pr_s_i.dot(h_do_i_ps);

			/*
			std::cout << "h_do_i_ps = " << h_do_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_i(i) = " << cost_do << std::endl; */

			return cost_do;
		}

		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,	// In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::MatrixXd &P_c_i,		// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int i,						// In: Index of obstacle
			const double ownship_length			// In: Length of the ownship along the body x-axis
		) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost_do(0.0), cost_ps(0.0), cost_coll(0.0), l_i(0.0);

			int n_samples = trajectory.cols();
			Eigen::MatrixXd P_i_p = obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

			int n_ps = xs_i_p.size();
			Eigen::VectorXd h_do_i_ps(n_ps);
			h_do_i_ps.setZero();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), d_0i_p(0.0);
			for (int k = 0; k < n_samples; k++)
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

				for (int ps = 0; ps < n_ps; ps++)
				{
					L_0i_p = xs_i_p[ps].block<2, 1>(0, k) - trajectory.block<2, 1>(0, k);
					d_0i_p = L_0i_p.norm();

					// Decrease the distance between the vessels by their respective max dimension
					d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + obstacles[i].get_length()));

					L_0i_p = L_0i_p.normalized();

					v_i_p(0) = xs_i_p[ps](2, k);
					v_i_p(1) = xs_i_p[ps](3, k);

					cost_coll = calculate_collision_cost(v_0_p, v_i_p);

					// Track loss modifier to collision cost
					if (obstacles[i].get_duration_lost() > pars.p_step_do)
					{
						l_i = pars.dt * pars.p_step_do / obstacles[i].get_duration_lost();
					}
					else
					{
						l_i = 1;
					}

					// PSB-MPC formulation with probabilistic collision cost
					// cost_ps = l_i * cost_coll * P_c_i(ps, k);

					// Should discount time when using a prediction scheme where the uncertainty for
					// each obstacle prediction scenario is bounded by r_ct
					cost_ps = l_i * cost_coll * P_c_i(ps, k) * exp(-(double)k * pars.dt / pars.T_coll);

					// Maximize wrt time
					if (cost_ps > h_do_i_ps(ps))
					{
						h_do_i_ps(ps) = cost_ps;
					}
				}
			}

			Eigen::VectorXd Pr_s_i = obstacles[i].get_scenario_probabilities();
			assert(Pr_s_i.size() == h_do_i_ps.size());

			// Weight prediction scenario costs by the scenario probabilities
			cost_do = Pr_s_i.dot(h_do_i_ps);
			/*
			std::cout << "h_do_i_ps = " << h_do_i_ps.transpose() << std::endl;
			std::cout << "Pr_s_i = " << Pr_s_i.transpose() << std::endl;
			std::cout << "cost_i(i) = " << cost_do << std::endl; */

			return cost_do;
		}

		// SBMPC do cost
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,		// In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::VectorXd &offset_sequence, // In: Offset sequence currently followed by the own-ship
			const Eigen::VectorXd &maneuver_times,	// In: Time of each maneuver in the offset sequence
			const Dynamic_Obstacles &obstacles,		// In: Dynamic obstacle information
			const Transitional_Variables &tv,		// In: Struct of all current time obstacle transitional variables
			const int i,							// In: Index of obstacle
			const double ownship_length				// In: Length of the ownship along the body x-axis
		) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost(0.0), max_cost(0.0), C(0.0), l_i(0.0), R(0.0);

			int n_samples = trajectory.cols();

			std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), psi_i_p(0.0), d_0i_p(0.0), chi_m(0.0); // R(0.0);
			bool COLREGS_violation_indicator(false), TC_indicator(false);
			for (int k = 0; k < n_samples; k++)
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
				d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + obstacles[i].get_length()));

				L_0i_p = L_0i_p.normalized();

				v_i_p(0) = xs_i_p[0](2, k);
				v_i_p(1) = xs_i_p[0](3, k);
				psi_i_p = atan2(v_i_p(1), v_i_p(0));

				C = calculate_collision_cost(v_0_p, v_i_p);

				if (k > 0)
				{
					COLREGS_violation_indicator = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);
				}

				TC_indicator = determine_transitional_cost_indicator(tv, psi_0_p, psi_i_p, L_0i_p, chi_m, i);

				R = calculate_ad_hoc_collision_risk(d_0i_p, (k + 1) * pars.dt);

				// Track loss modifier to collision cost
				if (obstacles[i].get_duration_lost() > pars.p_step_opt)
				{
					l_i = pars.dt * pars.p_step_opt / obstacles[i].get_duration_lost();
				}
				else
				{
					l_i = 1;
				}

				// SB-MPC formulation with ad-hoc collision risk
				cost = l_i * C * R + pars.kappa * COLREGS_violation_indicator + pars.kappa_TC * TC_indicator;

				if (cost > max_cost)
				{
					max_cost = cost;
				}
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
			const double d_AB, // In: Distance between vessel A (typically the own-ship) and vessel B (typically an obstacle)
							   // 	   reduced by half the length of the two vessels (or only own-ship if static obstacles are considered)
			const double t	   // In: Prediction time t > t0 (= 0)
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
			const Eigen::VectorXd &offset_sequence, // In: Offset_sequence currently followed by the own-ship
			const double u_m_last,					// In: Previous optimal output surge modification
			const double chi_m_last					// In: Previous optimal output course modification
		) const
		{
			double cost = 0;
			for (int i = 0; i < pars.n_M; i++)
			{
				if (i == 0)
				{
					cost += pars.K_u * fabs(1 - offset_sequence[0]) + Delta_u(offset_sequence[0], u_m_last) +
							K_chi(offset_sequence[1]) + Delta_chi(offset_sequence[1], chi_m_last);
				}
				else
				{
					cost += pars.K_u * fabs(1 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
							K_chi(offset_sequence[2 * i + 1]) + Delta_chi(offset_sequence[2 * i + 1], offset_sequence[2 * i - 1]);
				}
			}
			/* printf("K_u (1 - u_m_0) = %.4f | Delta_u(u_m_0, u_m_last) = %.4f | K_chi(chi_0) = %.4f | Delta_chi(chi_0, chi_last) = %.4f\n",
				pars.K_u * (1 - offset_sequence[0]), Delta_u(offset_sequence[0], u_m_last), K_chi(offset_sequence[1]), Delta_chi(offset_sequence[1], chi_m_last)); */

			cost /= (double)pars.n_M;
			return cost;
		}

		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_control_deviation_cost(
			const Eigen::VectorXd &offset_sequence, // In: Offset_sequence currently followed by the own-ship
			const double u_m_last,					// In: Previous optimal output surge modification
			const double chi_m_last,				// In: Previous optimal output course modification
			const double max_cross_track_error		// In: Maximum absolute predicted cross track error
		) const
		{
			double cost = 0.0;
			for (int i = 0; i < pars.n_M; i++)
			{
				if (i == 0)
				{
					cost += pars.K_u * fabs(1.0 - offset_sequence[0]) + Delta_u(offset_sequence[0], u_m_last) +
							pars.K_chi * pow(offset_sequence[1], 2) +
							pars.K_dchi * pow(wrap_angle_to_pmpi(offset_sequence[1] - chi_m_last), 2);
				}
				else
				{
					cost += pars.K_u * fabs(1.0 - offset_sequence[2 * i]) + Delta_u(offset_sequence[2 * i], offset_sequence[2 * i - 2]) +
							pars.K_chi * pow(offset_sequence[2 * i + 1], 2) +
							pars.K_dchi * pow(wrap_angle_to_pmpi(offset_sequence[2 * i + 1] - offset_sequence[2 * i - 1]), 2);
				}
			}
			/* printf("K_u (1 - u_m_0) = %.4f | Delta_u(u_m_0, u_m_last) = %.4f | K_chi(chi_0) = %.4f | Delta_chi(chi_0, chi_last) = %.4f\n",
				pars.K_u * (1 - offset_sequence[0]), Delta_u(offset_sequence[0], u_m_last), K_chi(offset_sequence[1]), Delta_chi(offset_sequence[1], chi_m_last)); */

			cost /= (double)pars.n_M;

			cost += pars.K_e * max_cross_track_error;

			return cost;
		}

		/****************************************************************************************
		 *  Name     : calculate_grounding_cost
		 *  Function : Determines penalty due to grounding ownship on static obstacles (no-go zones)
		 *  Author   : Trym Tengesdal & Tom Daniel Grande
		 *  Modified :
		 *****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_grounding_cost(
			const Eigen::MatrixXd &trajectory,	  // In: Predicted ownship trajectory
			const Static_Obstacles &polygons,	  // In: Static obstacle information
			const double V_w,					  // In: Estimated wind speed
			const Eigen::Vector2d &wind_direction // In: Unit vector in NE describing the estimated wind direction
		) const
		{
			int n_samples = std::round(pars.T / pars.dt);
			int n_so = polygons.size();
			double max_cost_g(0.0), cost_g(0.0);

			Eigen::Vector2d L_0j;
			double d_0j(0.0), t(0.0), phi_j(0.0);

			for (int k = 0; k < n_samples; k += pars.p_step_grounding)
			{
				t = pars.dt * k;

				cost_g = 0.0;
				for (int j = 0; j < n_so; j++)
				{
					L_0j = distance_to_polygon(trajectory.block<2, 1>(0, k), polygons[j]);
					d_0j = L_0j.norm();
					L_0j.normalize();

					phi_j = std::max(0.0, L_0j.dot(wind_direction));

					if (d_0j >= pars.d_safe)
					{
						cost_g = (pars.G_1 + pars.G_2 * phi_j * pow(V_w, 2)) * exp(-(pars.G_3 * fabs(d_0j - pars.d_safe) + pars.G_4 * t));
					}
					else
					{
						cost_g = (pars.G_1 + pars.G_2 * phi_j * pow(V_w, 2)) * exp(-pars.G_4 * t);
					}

					if (max_cost_g < cost_g)
					{
						max_cost_g = cost_g;
					}
					// printf("t = %.2f | d_0j = %.6f | cost_g = %.6f | max_cost_g = %.6f\n", k * pars.dt, d_0j, cost_g, max_cost_g);
				}
			}
			return max_cost_g;
		}

		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_grounding_cost(
			Eigen::VectorXd &max_cost_j,		  // In/Out: Max costs wrt to each static obstacle, for debugging purposes
			const Eigen::MatrixXd &trajectory,	  // In: Predicted ownship trajectory
			const Static_Obstacles &polygons,	  // In: Static obstacle information
			const double V_w,					  // In: Estimated wind speed
			const Eigen::Vector2d &wind_direction // In: Unit vector in NE describing the estimated wind direction
		) const
		{
			int n_samples = std::round(pars.T / pars.dt);
			int n_so = polygons.size();
			double max_cost_g(0.0), cost_g(0.0);

			Eigen::Vector2d L_0j;
			double d_0j(0.0), t(0.0), phi_j(0.0);
			max_cost_j.resize(n_so);
			max_cost_j.setZero();
			for (int k = 0; k < n_samples; k += pars.p_step_grounding)
			{
				t = pars.dt * k;

				cost_g = 0.0;
				for (int j = 0; j < n_so; j++)
				{
					L_0j = distance_to_polygon(trajectory.block<2, 1>(0, k), polygons[j]);
					d_0j = L_0j.norm();
					L_0j.normalize();

					phi_j = std::max(0.0, L_0j.dot(wind_direction));

					if (d_0j >= pars.d_safe)
					{
						cost_g = (pars.G_1 + pars.G_2 * phi_j * pow(V_w, 2)) * exp(-(pars.G_3 * fabs(d_0j - pars.d_safe) + pars.G_4 * t));
					}
					else
					{
						cost_g = (pars.G_1 + pars.G_2 * phi_j * pow(V_w, 2)) * exp(-pars.G_4 * t);
					}
					if (max_cost_j(j) < cost_g)
					{
						max_cost_j(j) = cost_g;
					}
					if (max_cost_g < cost_g)
					{
						max_cost_g = cost_g;
					}
					// printf("t = %.2f | d_0j = %.6f | cost_g = %.6f | max_cost_g = %.6f\n", k * pars.dt, d_0j, cost_g, max_cost_g);
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
			const Eigen::MatrixXd &trajectory,					  // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::Matrix<double, 4, -1> &static_obstacles, // In: Static obstacle information
			const double ownship_length							  // In: Length of the ownship along the body x-axis
		) const
		{
			double d_geo(0.0), t(0.0), g_cost(0.0);
			int n_so = static_obstacles.cols();
			int n_samples = std::round(pars.T / pars.dt);
			// so 1 and 2 : endpoints of line describing static obstacle
			Eigen::Vector2d p_0, p_1, so_1, so_2;

			Eigen::VectorXd cost_j(n_so);
			cost_j.setZero();

			// Check if it is necessary to calculate this cost
			bool is_geo_constraint = false;
			for (int j = 0; j < n_so; j++)
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

			if (n_so == 0 || !is_geo_constraint)
			{
				return 0.0;
			}

			for (int k = 0; k < n_samples - 1; k++)
			{
				t = (k + 1) * pars.dt;

				for (int j = 0; j < n_so; j++)
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
			const Eigen::Vector2d &r) const
		{
			double epsilon = 1e-8; // abs(val) less than 1e-8 m^2 is considered zero for this check
			// Calculate z-component of cross product (q - p) x (r - q)
			double val = (q(0) - p(0)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(0) - q(0));

			// printf("p = %.6f, %.6f | q = %.6f, %.6f | r = %.6f, %.6f | val = %.15f\n", p(0), p(1), q(0), q(1), r(0), r(1), val);
			if (val > -epsilon && val < epsilon)
			{
				return 0;
			} // colinear
			else if (val > epsilon)
			{
				return 1;
			} // clockwise
			else
			{
				return 2;
			} // counterclockwise
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
			const Eigen::Vector2d &r) const
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
			const double distance_to_line) const
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
			const Eigen::Vector2d &q_2) const
		{
			// Find the four orientations needed for general and
			// special cases
			int o_1 = find_triplet_orientation(p_1, q_1, p_2);
			int o_2 = find_triplet_orientation(p_1, q_1, q_2);
			int o_3 = find_triplet_orientation(p_2, q_2, p_1);
			int o_4 = find_triplet_orientation(p_2, q_2, q_1);

			// printf("o_1 = %d | o_2 = %d | o_3 = %d | o_4 = %d\n", o_1, o_2, o_3, o_4);
			//  General case
			if (o_1 != o_2 && o_3 != o_4)
			{
				return true;
			}

			// Special Cases
			// p_1, q_1 and p_2 are colinear and p_2 lies on segment p_1 -> q_1
			if (o_1 == 0 && determine_if_on_segment(p_1, p_2, q_1))
			{
				return true;
			}

			// p_1, q_1 and q_2 are colinear and q_2 lies on segment p_1 -> q_1
			if (o_2 == 0 && determine_if_on_segment(p_1, q_2, q_1))
			{
				return true;
			}

			// p_2, q_2 and p_1 are colinear and p_1 lies on segment p_2 -> q_2
			if (o_3 == 0 && determine_if_on_segment(p_2, p_1, q_2))
			{
				return true;
			}

			// p_2, q_2 and q_1 are colinear and q_1 lies on segment p_2 -> q_2
			if (o_4 == 0 && determine_if_on_segment(p_2, q_1, q_2))
			{
				return true;
			}

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
			const Eigen::Vector2d &q_2) const
		{
			Eigen::Vector3d a;
			Eigen::Vector3d b;
			a << (q_1 - q_2), 0.0;
			b << (p - q_2), 0.0;

			Eigen::Vector3d c = a.cross(b);
			if (a.norm() > 0)
				return c.norm() / a.norm();
			else
				return -1;
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
			const Eigen::Vector2d &v_2) const
		{
			double d2line = distance_to_line(p, v_1, v_2);

			if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line))
				return d2line;
			else
				return std::min((v_1 - p).norm(), (v_2 - p).norm());
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
			const polygon_2D &poly) const
		{
//======================================================
// MATLAB PLOTTING FOR DEBUGGING
//======================================================
#if ENABLE_PSBMPC_DEBUGGING
			/* Engine *ep = engOpen(NULL);
				if (ep == NULL)
				{
					std::cout << "engine start failed!" << std::endl;
				}
				mxArray *polygon_matrix_mx(nullptr);
				mxArray *polygon_side_mx = mxCreateDoubleMatrix(2, 2, mxREAL);
				//mxArray *d_0j_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
				mxArray *p_os_ray_mx = mxCreateDoubleMatrix(2, 2, mxREAL);
				mxArray *bbox_mx = mxCreateDoubleMatrix(2, 2, mxREAL);
				double *p_polygon_matrix(nullptr);
				double *p_polygon_side = mxGetPr(polygon_side_mx);
				double *p_p_os_ray = mxGetPr(p_os_ray_mx);
				double *p_bbox = mxGetPr(bbox_mx);


				Eigen::Map<Eigen::Matrix2d> map_polygon_side(p_polygon_side, 2, 2);
				//Eigen::Map<Eigen::Vector2d> map_d_0j;
				Eigen::Map<Eigen::Matrix2d> map_p_os_ray(p_p_os_ray, 2, 2);
				Eigen::Map<Eigen::Matrix2d> map_bbox(p_bbox, 2, 2);

				Eigen::Matrix2d p_os_ray; p_os_ray.col(0) = p;
				Eigen::Matrix2d polygon_side; */
#endif
			//======================================================
			int line_intersect_count(0), n_vertices(0);
			Eigen::Vector2d v_prev, v, v_next;

			Eigen::MatrixXd vertices(2, 50000);
			// Find bounding box of polygon to use for ray creation
			Eigen::Matrix2d bbox;
			bbox(0, 0) = 1e10;
			bbox(1, 0) = 1e10;
			bbox(0, 1) = -1e10;
			bbox(1, 1) = -1e10;
			for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
			{
				v(0) = boost::geometry::get<0>(*it);
				v(1) = boost::geometry::get<1>(*it);

				vertices(0, n_vertices) = v(0);
				vertices(1, n_vertices) = v(1);

				if (v(0) < bbox(0, 0))
				{
					bbox(0, 0) = v(0);
				} // x_min
				if (v(1) < bbox(1, 0))
				{
					bbox(1, 0) = v(1);
				} // y_min
				if (v(0) > bbox(0, 1))
				{
					bbox(0, 1) = v(0);
				} // x_max
				if (v(1) > bbox(1, 1))
				{
					bbox(1, 1) = v(1);
				} // y_max
				n_vertices += 1;
			}
			vertices.conservativeResize(2, n_vertices);
			if (n_vertices < 3 ||
				p(0) < bbox(0, 0) || p(0) > bbox(0, 1) || p(1) < bbox(1, 0) || p(1) > bbox(1, 1))
			{
				return false;
			}

			Eigen::Vector2d p_ray_end;
			p_ray_end = bbox.col(1); // set ray end to x_max, y_max of polygon bbox
			p_ray_end(0) += 0.1;
			p_ray_end = p + 1.1 * (bbox.col(1) - p);

//======================================================
// MATLAB PLOTTING FOR DEBUGGING
//======================================================
#if ENABLE_PSBMPC_DEBUGGING
			/* p_os_ray.col(1) = p_ray_end;

				polygon_matrix_mx = mxCreateDoubleMatrix(2, n_vertices, mxREAL);
				p_polygon_matrix = mxGetPr(polygon_matrix_mx);
				Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, 2, n_vertices);
				map_polygon_matrix = vertices;
				map_bbox = bbox;
				map_p_os_ray = p_os_ray;

				engPutVariable(ep, "polygon_vertices", polygon_matrix_mx);
				engPutVariable(ep, "p_os_ray", p_os_ray_mx);
				engPutVariable(ep, "bbox", bbox_mx);
				engEvalString(ep, "init_plot_geometry_wrt_polygon"); */
#endif
			//======================================================
			int o_11(0), o_12(0);
			for (int l = 0; l < n_vertices; l++)
			{
				if (l == 0)
				{
					v_prev = vertices.col(n_vertices - 1);
				}
				else
				{
					v_prev = vertices.col(l - 1);
				}
				v = vertices.col(l);
				if (l == n_vertices - 1)
				{
					v_next = vertices.col(0);
				}
				else
				{
					v_next = vertices.col(l + 1);
				}

//======================================================
// MATLAB PLOTTING FOR DEBUGGING
//======================================================
#if ENABLE_PSBMPC_DEBUGGING
				/* polygon_side.col(0) = v; polygon_side.col(1) = v_next;
					map_polygon_side = polygon_side;
					engPutVariable(ep, "polygon_side", polygon_side_mx);
					engEvalString(ep, "plot_geometry_wrt_polygon"); */
#endif
				//======================================================

				if (determine_if_lines_intersect(p, p_ray_end, v, v_next))
				{
					// Special case when p is colinear with line segment from v -> v_next
					if (find_triplet_orientation(v, p, v_next) == 0)
					{
						return determine_if_on_segment(v, p, v_next);
					}
					line_intersect_count += 1;

					// Special case when the vertex v is colinear with p -> p_ray_end
					if (find_triplet_orientation(p, v, p_ray_end) == 0)
					{
						// Determine if:
						// 1) both polygon sides connected to v are below/above the ray: 0 or 2 intersections
						// 	  => add one to the line intersection count.
						// 2) if one side is below and the other above: 1 intersection.
						o_11 = find_triplet_orientation(p, p_ray_end, v_prev);
						o_12 = find_triplet_orientation(p, p_ray_end, v_next);
						if (o_11 == o_12)
						{
							line_intersect_count += 1;
						}
						// add one extra to account for the fact that the other side connecting the
						// vertex will be checked, when (v_next = v).
						line_intersect_count += 1;
					}
				}
			}
//======================================================
// MATLAB PLOTTING FOR DEBUGGING
//======================================================
#if ENABLE_PSBMPC_DEBUGGING
			/* mxDestroyArray(polygon_matrix_mx);
				mxDestroyArray(polygon_side_mx);
				mxDestroyArray(p_os_ray_mx);
				mxDestroyArray(bbox_mx);
				engClose(ep); */
#endif
			//======================================================
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
			const Eigen::Vector2d &q_2) const
		{
			double epsilon = 0.00001, l_sqrt(0.0), t_line(0.0);
			Eigen::Vector3d a, b;
			Eigen::Vector2d projection;
			a << (q_2 - q_1), 0.0;
			b << (p - q_1), 0.0;

			l_sqrt = a(0) * a(0) + a(1) * a(1);
			if (l_sqrt <= epsilon)
			{
				return q_1 - p;
			}

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
			const polygon_2D &poly) const
		{
			Eigen::Vector2d d2poly, d2line;
			if (determine_if_inside_polygon(p, poly))
			{
				d2poly(0) = 0.0;
				d2poly(1) = 0.0;
				return d2poly;
			}
			d2poly(0) = 1e10;
			d2poly(1) = 1e10;
			Eigen::Vector2d v, v_next;
			for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
			{
				v(0) = boost::geometry::get<0>(*it);
				v(1) = boost::geometry::get<1>(*it);
				v_next(0) = boost::geometry::get<0>(*(it + 1));
				v_next(1) = boost::geometry::get<1>(*(it + 1));

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
