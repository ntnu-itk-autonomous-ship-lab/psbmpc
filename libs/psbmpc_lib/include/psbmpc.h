/****************************************************************************************
*
*  File name : psbmpc.h
*
*  Function  : Header file for Probabilistic Scneario-based Model Predictive Control.
*			   
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

#ifndef _PSBMPC_H_
#define _PSBMPC_H_


#include "psbmpc_index.h"
#include "psbmpc_parameters.h"
#include "obstacle_manager.h"
#include "ownship.h"
#include "tracked_obstacle.h"
#include "cpe.h"
#include "Eigen/Dense"
#include <vector>
#include <memory>	

class PSBMPC
{
private:

	std::vector<int> n_ps;

	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

	double u_m_last;
	double chi_m_last;

	double min_cost;

	std::unique_ptr<Ownship> ownship;

	std::unique_ptr<CPE> cpe;

	Eigen::Matrix<double, 6, -1> trajectory;

	void reset_control_behaviour();

	void increment_control_behaviour();

	void initialize_prediction();

	void set_up_independent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const int i,
		const int n_turns);

	void set_up_dependent_obstacle_prediction_variables(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_weights,
		Eigen::VectorXd &ps_maneuver_times,
		const int i);

	double find_time_of_passing(const int i);

	void predict_trajectories_jointly();

	bool determine_colav_active(const int n_static_obst);

	bool determine_COLREGS_violation(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

	bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const int i,
		const double chi_m);
	bool determine_transitional_cost_indicator(const Eigen::VectorXd &xs_A, const Eigen::VectorXd &xs_B, const int i, const double chi_m);

	void calculate_collision_probabilities(Eigen::MatrixXd &P_c_i, const int i);

	double calculate_dynamic_obstacle_cost(const Eigen::MatrixXd &P_c_i, const int i);

	inline double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) { return pars.K_coll * (v_1 - v_2).norm(); };

	double calculate_ad_hoc_collision_risk(const double d_AB, const double t);

	// Methods dealing with control deviation cost
	double calculate_control_deviation_cost();

	inline double Delta_u(const double u_1, const double u_2) 		{ return pars.K_du * fabs(u_1 - u_2); }

	inline double K_chi(const double chi)							{ if (chi > 0) return pars.K_chi_strb * pow(chi, 2); else return pars.K_chi_port * pow(chi, 2); };

	inline double Delta_chi(const double chi_1, const double chi_2) 	{ if (chi_1 > 0) return pars.K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return pars.K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	double calculate_chattering_cost();

	// Methods dealing with geographical constraints
	double calculate_grounding_cost(const Eigen::Matrix<double, 4, -1>& static_obstacles);

    int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);                           

    bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);   

    bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line);                         

    bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2);   

    double distance_from_point_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2);                  

    double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2);

	//
	void assign_optimal_trajectory(Eigen::Matrix<double, 2, -1> &optimal_trajectory);

public:

	PSBMPC_Parameters pars;

	PSBMPC();

	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		std::unique_ptr<Obstacle_Manager> &obstacle_manager,
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

};

#endif 