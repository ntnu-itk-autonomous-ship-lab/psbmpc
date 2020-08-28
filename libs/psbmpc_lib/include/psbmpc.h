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
#include "ownship.h"
#include "tracked_obstacle.h"
#include "cpe.h"
#include "Eigen/Dense"
#include <vector>
#include <memory>

enum ST 
{
	A, 														// Non-COLREGS situation	(ST = Ø)
	B, 														// Stand-on in Overtaking 	(ST = OT, SO)
	C, 														// Stand-on in Crossing 	(ST = CR, SO)
	D, 														// Give-way in Overtaking 	(ST = OT, GW)
	E, 														// Give-way in Head-on 		(ST = HO, GW)
	F 														// Give-way in Crossing 	(ST = CR, GW)
};	

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

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	std::vector<bool> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

	// Situation type variables at the current time for the own-ship (wrt all nearby obstacles) and nearby obstacles
	std::vector<ST> ST_0, ST_i_0;

	std::vector<std::unique_ptr<Tracked_Obstacle>> old_obstacles;
	std::vector<std::unique_ptr<Tracked_Obstacle>> new_obstacles;

	void reset_control_behaviour();

	void increment_control_behaviour();

	void initialize_par_limits();

	void initialize_pars();

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

	void determine_situation_type(
		ST &st_A,
		ST &st_B,
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

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

    void update_obstacles(
		const Eigen::Matrix<double, 9, -1>& obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities);

	void update_obstacle_status(Eigen::Matrix<double,-1,-1> &obstacle_status, const Eigen::VectorXd &HL_0);

	void update_situation_type_and_transitional_variables();

public:

	PSBMPC_Parameters pars;

	PSBMPC();

	void calculate_optimal_offsets(
		double &u_opt, 
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		Eigen::Matrix<double, -1, -1> &obstacle_status,
		Eigen::Matrix<double, -1, 1> &colav_status,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 6, 1> &ownship_state,
		const Eigen::Matrix<double, 9, -1> &obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::MatrixXd &obstacle_intention_probabilities,
		const Eigen::VectorXd &obstacle_a_priori_CC_probabilities,
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

};

#endif 