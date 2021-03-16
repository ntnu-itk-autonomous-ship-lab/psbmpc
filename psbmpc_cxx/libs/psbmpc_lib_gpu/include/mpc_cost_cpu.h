/****************************************************************************************
*
*  File name : mpc_cost.h
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

#include "utilities_cpu.h"
#include "psbmpc_parameters.h"
#include "obstacle_manager.h"
#include "prediction_obstacle.h"
#include "Eigen/Dense"

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_2D;
typedef boost::geometry::model::polygon<point_2D> polygon_2D;

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

			int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const;                          

			bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) const; 

			bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line) const;                        

			bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2) const;  

			double distance_from_point_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2) const;                 

			double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) const;

		public:

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
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory, 
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const Eigen::MatrixXd &P_c_i, 
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;
			
			double calculate_dynamic_obstacle_cost(
				const Eigen::MatrixXd &trajectory, 
				const Eigen::VectorXd &offset_sequence,
				const Eigen::VectorXd &maneuver_times,
				const Obstacle_Data<Tracked_Obstacle> &data, 
				const int i, 
				const double ownship_length) const;

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

			double calculate_grounding_cost(const Eigen::MatrixXd &trajectory, const std::vector<polygon_2D> &polygons, const int n_static_obst) const;
			double calculate_grounding_cost(const Eigen::MatrixXd &trajectory, const Eigen::Matrix<double, 4, -1>& static_obstacles, const double ownship_length) const;
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

			// Extra condition that the COLREGS violation is only considered in an annulus; i.e. within d_close but outside d_safe.
			// The logic behind having to be outside d_safe is that typically a collision happens here, and thus COLREGS should be disregarded
			// in order to make a safe reactive avoidance maneuver, if possible.  
			return is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken)) && (d_AB > pars.d_safe);
		}

		/****************************************************************************************
		*  Name     : calculate_dynamic_obstacle_cost
		*  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i
		*             Two overloads depending on if its PSBMPC/SBMPC
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const Eigen::MatrixXd &trajectory,                          // In: Own-ship trajectory when following the current offset_sequence/control behaviour
			const Eigen::VectorXd &offset_sequence,                     // In: Offset sequence currently followed by the own-ship
			const Eigen::VectorXd &maneuver_times,                      // In: Time of each maneuver in the offset sequence
			const Eigen::MatrixXd &P_c_i,								// In: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
			const Obstacle_Data<Tracked_Obstacle> &data,				// In: Dynamic obstacle information
			const int i, 												// In: Index of obstacle
			const double ownship_length                                 // In: Length of the ownship along the body x-axis
			) const
		{
			// l_i is the collision cost modifier depending on the obstacle track loss.
			double cost(0.0), cost_ps(0.0), C(0.0), l_i(0.0);

			int n_samples = trajectory.cols();
			Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
			std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

			int n_ps = xs_i_p.size();
			Eigen::VectorXd max_cost_ps(n_ps), weights_ps(n_ps);
			max_cost_ps.setZero(); weights_ps.setZero();

			Eigen::Vector2d v_0_p, v_i_p, L_0i_p;
			double psi_0_p(0.0), psi_i_p(0.0), d_0i_p(0.0), chi_m(0.0); //R(0.0);
			bool mu(false), trans(false);
			for(int k = 0; k < n_samples; k++)
			{
				psi_0_p = trajectory(2, k); 
				v_0_p(0) = trajectory(3, k); 
				v_0_p(1) = trajectory(4, k); 
				v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

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
					//std::cout << "k = " << k << std::endl;
					//std::cout << "chi_m = " << chi_m * RAD2DEG << std::endl;
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
					psi_i_p = atan2(v_i_p(1), v_i_p(0));

					C = calculate_collision_cost(v_0_p, v_i_p);

					mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

					trans = determine_transitional_cost_indicator(psi_0_p, psi_i_p, L_0i_p, chi_m, data, i);

					//R = calculate_ad_hoc_collision_risk(d_0i_p, (k + 1) * pars.dt);

					// Track loss modifier to collision cost
					if (data.obstacles[i].get_duration_lost() > pars.p_step)
					{
						l_i = pars.dt * pars.p_step / data.obstacles[i].get_duration_lost();
					} 
					else
					{
						l_i = 1;
					}
					
					// SB-MPC formulation with ad-hoc collision risk
					//cost_ps = l_i * C * R + pars.kappa * mu  + pars.kappa_TC * trans;

					// PSB-MPC formulation with probabilistic collision cost
					cost_ps = l_i * C * P_c_i(ps, k) + pars.kappa * mu  + pars.kappa_TC * trans;

					// Maximize wrt time
					if (cost_ps > max_cost_ps(ps))
					{
						max_cost_ps(ps) = cost_ps;
					}
				}
			}

			// If only 1 prediction scenario
			// => Original PSB-MPC formulation
			if (n_ps == 1)
			{
				cost = max_cost_ps(0);
				return cost;
			}
			// Weight prediction scenario cost based on if obstacle follows COLREGS or not,
			// which means that higher cost is applied if the obstacle follows COLREGS
			// to a high degree (high Pr_CC_i with no COLREGS violation from its side)
			// and the own-ship breaches COLREGS

			std::vector<Intention> ps_ordering = data.obstacles[i].get_ps_ordering();
			std::vector<bool> mu_i = data.obstacles[i].get_COLREGS_violation_indicator();
			Eigen::VectorXi ps_intention_count = data.obstacles[i].get_ps_intention_count();

			double Pr_CC_i = data.obstacles[i].get_a_priori_CC_probability();
			if (Pr_CC_i < 0.0001) // Should not be allowed to be strictly 0
			{
				Pr_CC_i = 0.0001;
			}

			Eigen::Vector3d cost_a_weight_sums; cost_a_weight_sums.setZero();
			for (int ps = 0; ps < n_ps; ps++)
			{
				weights_ps(ps) = Pr_CC_i;
				if (mu_i[ps])
				{
					//printf("Obstacle i = %d breaks COLREGS in ps = %d\n", i, ps);
					weights_ps(ps) = 1 - Pr_CC_i;
				}
				
				if (ps_ordering[ps] == KCC)
				{
					cost_a_weight_sums(0) += weights_ps(ps);
				}
				else if (ps_ordering[ps] == SM)
				{
					cost_a_weight_sums(1) += weights_ps(ps);
				}
				else if (ps_ordering[ps] == PM)
				{
					cost_a_weight_sums(2) += weights_ps(ps);
				}
			}

			Eigen::Vector3d cost_a = {0, 0, 0};
			Eigen::VectorXd Pr_a = data.obstacles[i].get_intention_probabilities();
			assert(Pr_a.size() == 3);
			
			for(int ps = 0; ps < n_ps; ps++)
			{
				if (ps_ordering[ps] == KCC)
				{
					cost_a(0) += (weights_ps(ps) / cost_a_weight_sums(0)) * max_cost_ps(ps);
				}
				else if (ps_ordering[ps] == SM)
				{
					cost_a(1) +=  (weights_ps(ps) / cost_a_weight_sums(1)) * max_cost_ps(ps);
				}
				else if (ps_ordering[ps] == PM)
				{
					cost_a(2) +=  (weights_ps(ps) / cost_a_weight_sums(2)) * max_cost_ps(ps);
				}
			}
			
			// Average the cost for the starboard and port maneuver type of intentions
			if (ps_intention_count(0) > 0) 	{ cost_a(0) /= (double)ps_intention_count(0); }
			else 							{ cost_a(0) = 0.0;}
			if (ps_intention_count(1) > 0)	{ cost_a(1) /= (double)ps_intention_count(1); } 
			else							{ cost_a(1) = 0.0; }
			if (ps_intention_count(2) > 0)	{ cost_a(2) /= (double)ps_intention_count(2); } 
			else							{ cost_a(2) = 0.0; }

			// Weight by the intention probabilities
			cost = Pr_a.dot(cost_a);

			/* std::cout << "weights_ps = " << weights_ps.transpose() << std::endl;
			std::cout << "max_cost_ps = " << max_cost_ps.transpose() << std::endl;
			std::cout << "Pr_a = " << Pr_a.transpose() << std::endl;
			std::cout << "cost a = " << cost_a.transpose() << std::endl;
			std::cout << "cost_i(i) = " << cost << std::endl; */

			return cost;
		}

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
			bool mu, trans;
			for(int k = 0; k < n_samples; k++)
			{
				psi_0_p = trajectory(2, k); 
				v_0_p(0) = trajectory(3, k); 
				v_0_p(1) = trajectory(4, k); 
				v_0_p = rotate_vector_2D(v_0_p, psi_0_p);

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

				mu = determine_COLREGS_violation(v_0_p, psi_0_p, v_i_p, L_0i_p, d_0i_p);

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
				d_0i_p = abs(d_0i_p - 0.5 * (ownship_length + data.obstacles[i].get_length())); 

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
		*  Author   : Tom Daniel Grande
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::calculate_grounding_cost(
			const Eigen::MatrixXd &trajectory,									// In: Predicted ownship trajectory
			const std::vector<polygon_2D> &polygons,							// In: Static obstacle information
			const int n_static_obst 											// In: Number of static obstacles
			) const
		{
			int n_static_samples = std::round(pars.T_static / pars.dt);
			double g_cost 				= 0.0;	
			double eta  				= 25.0;		  	  	 //grounding sensitivity
			double my_1 				= 0.35; 		 	 //grounding cost
			double my_2 				= 1.0; 			 	 // wind disturbance risk
			double chi_j 				= 1.0; 			 	 // unit wind direction
			double V_w 					= 0.0; 			 	 // absolute wind velocity
			double K_omega 				= 50.0; 			 // horizon focus weight		 

			double d2poly, exp_term, exp_calc;
			point_2D p_os_k;
			for (int k = 0; k < n_static_samples - 1; k++)
			{
				p_os_k = point_2D(trajectory(1, k), trajectory(0, k));

				BOOST_FOREACH(polygon_2D const& poly, polygons)
				{
					d2poly = boost::geometry::distance(p_os_k, poly);

					exp_term = (-1.0 / (eta * eta) ) * (d2poly * d2poly + K_omega * k);

					exp_calc = ( my_1 + my_2 * chi_j * V_w * V_w) * std::exp(exp_term);

					g_cost = g_cost + exp_calc;
				}
			}

			return g_cost / (double) n_static_obst * pars.n_M;
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
			int n_static_samples = std::round(pars.T_static / pars.dt);
			// so 1 and 2 : endpoints of line describing static obstacle
			Eigen::Vector2d p_0, p_1, so_1, so_2; 

			Eigen::VectorXd cost_j(n_static_obst);
			cost_j.setZero();

			// Check if it is necessary to calculate this cost
			bool is_geo_constraint = false;
			for (int j = 0; j < n_static_obst; j++)
			{
				p_0 << trajectory.block<2, 1>(0, 0);
				p_1 << trajectory.block<2, 1>(0, n_static_samples - 1);

				so_1 << static_obstacles.block<2, 1>(0, j);
				so_2 << static_obstacles.block<2, 1>(2, j);

				d_geo = distance_from_point_to_line(p_1, so_1, so_2);

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
			
			for (int k = 0; k < n_static_samples - 1; k++)
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
		*  Author   : Giorgio D. Kwame Minde Kufoalor
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		int MPC_Cost<Parameters>::find_triplet_orientation(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q, 
			const Eigen::Vector2d &r
			) const
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
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		bool MPC_Cost<Parameters>::determine_if_on_segment(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q, 
			const Eigen::Vector2d &r
			) const
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
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::distance_from_point_to_line(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &q_1, 
			const Eigen::Vector2d &q_2
			) const
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
		*  Modified : By Trym Tengesdal for more readability
		*****************************************************************************************/
		template <typename Parameters>
		double MPC_Cost<Parameters>::distance_to_static_obstacle(
			const Eigen::Vector2d &p, 
			const Eigen::Vector2d &v_1, 
			const Eigen::Vector2d &v_2
			) const
		{
			double d2line = distance_from_point_to_line(p, v_1, v_2);

			if (determine_if_behind(p, v_1, v_2, d2line) || determine_if_behind(p, v_2, v_1, d2line)) return d2line;
			else return std::min((v_1-p).norm(),(v_2-p).norm());
		}
	}
}