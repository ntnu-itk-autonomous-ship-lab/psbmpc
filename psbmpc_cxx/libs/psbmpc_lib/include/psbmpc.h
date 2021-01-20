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
#include "joint_prediction_manager.h"
#include "obstacle_sbmpc.h"
#include "ownship.h"
#include "cpe.h"

#include "Eigen/Dense"
#include <vector>
#include <memory>	

class PSBMPC
{
private:
	// Amount of prediction scenarios for each obstacle
	std::vector<int> n_ps;

	// Control behavior related vectors and the own-ship maneuver times in the prediction horizon
	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

	// Previous optimal offsets/modifications
	double u_m_last;
	double chi_m_last;

	// Cost at the optimal solution
	double min_cost;

	// Own-ship predicted trajectory
	Eigen::Matrix<double, 6, -1> trajectory;

	Ownship ownship;

	CPE cpe;

	std::vector<Prediction_Obstacle> pobstacles;

	void reset_control_behaviour();

	void increment_control_behaviour();

	void initialize_prediction(Obstacle_Data<Tracked_Obstacle> &data, const Eigen::Matrix<double, 4, -1> &static_obstacles);

	void set_up_independent_obstacle_prediction(
		std::vector<Intention> &ps_ordering,
		Eigen::VectorXd &ps_course_changes,
		Eigen::VectorXd &ps_maneuver_times,
		const double t_cpa_i,
		const Obstacle_Data<Tracked_Obstacle> &data,
		const int i);

	// Obstacle prediction scenario pruning related methods
	void prune_obstacle_scenarios(Obstacle_Data<Tracked_Obstacle> &data);

	void calculate_ps_collision_probabilities(Eigen::VectorXd &P_c_i_ps, const Eigen::MatrixXd &P_c_i, const int i);

	void calculate_ps_collision_consequences(Eigen::VectorXd &C_i, const Obstacle_Data<Tracked_Obstacle> &data, const int i, const double dt, const int p_step);

	void calculate_ps_collision_risks(Eigen::VectorXd &R_c_i, Eigen::VectorXi indices_i, const Eigen::VectorXd &C_i, const Eigen::VectorXd &P_c_i_ps, const Obstacle_Data<Tracked_Obstacle> &data, const int i);

	void predict_trajectories_jointly(const Eigen::Matrix<double, 4, -1>& static_obstacles);

	double find_time_of_passing(const Obstacle_Data<Tracked_Obstacle> &data, const int i);
	//

	bool determine_colav_active(const Obstacle_Data<Tracked_Obstacle> &data, const int n_static_obst);

	bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const double chi_m,
		const Obstacle_Data<Tracked_Obstacle> &data,
		const int i);

	void calculate_instantaneous_collision_probabilities(Eigen::MatrixXd &P_c_i, const Obstacle_Data<Tracked_Obstacle> &data, const int i, const double dt, const int p_step);

	double calculate_dynamic_obstacle_cost(const Eigen::MatrixXd &P_c_i, const Obstacle_Data<Tracked_Obstacle> &data, const int i);

	inline double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) { return pars.K_coll * pow((v_1 - v_2).norm(), 2); };

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

	bool determine_COLREGS_violation(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB, 
		const double d_AB) const;

	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data<Tracked_Obstacle> &data);

};

#endif 