/****************************************************************************************
*
*  File name : psb_mpc.h
*
*  Function  : Header file for Probabilistic Scneario-based Model Predictive Control.
*			   Extends and modifies the SB-MPC implemented by Inger Berge Hagen and 
*			   Giorgio D. Kwame Minde Kufoalor through the Autosea project.
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

#include "cpe.h"
#include "psbmpc_index.h"
#include "ownship.h"
#include "obstacle.h"
#include "Eigen/Dense"
#include <vector>
#include <math.h>

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

	int n_cbs, n_M, n_a;
	std::vector<Eigen::VectorXd> u_offsets;
	std::vector<Eigen::VectorXd> chi_offsets;

	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times, course_changes;

	double u_m_last;
	double chi_m_last;

	double min_cost;

	Eigen::VectorXd dpar_low, dpar_high;
	Eigen::VectorXd ipar_low, ipar_high;

	CPE_Method cpe_method;

	Prediction_Method prediction_method;

	Guidance_Method guidance_method;

	double T, T_static, dt, p_step;
	double t_ts;
	double d_safe, d_close, d_init;
	double K_coll;
	double phi_AH, phi_OT, phi_HO, phi_CR;
	double kappa, kappa_TC;
	double K_u, K_du;
	double K_chi_strb, K_dchi_strb;
	double K_chi_port, K_dchi_port; 
	double K_sgn, T_sgn;
	double G;
	
	bool obstacle_filter_on, obstacle_colav_on;
	double T_lost_limit, T_tracked_limit;

	Ownship *ownship;

	CPE *cpe;

	Eigen::Matrix<double, 6, -1> trajectory;

	// Transitional indicator variables at the current time
	Eigen::Matrix<bool, -1, 1> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0;

	// Situation type variables at the current time
	ST ST_0;
	std::vector<ST> ST_i_0;

	std::vector<Obstacle*> old_obstacles;
	std::vector<Obstacle*> new_obstacles;

	void initialize_par_limits();

	void initialize_pars();

	void initialize_prediction();

	void reset_control_behavior();

	void increment_control_behavior();

	void predict_trajectories_jointly();

	bool determine_colav_active(const Eigen::Matrix<double, 6, 1> xs, const int n_static_obst);

	void determine_situation_type(
		ST& st_A,
		ST& st_B,
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

	bool determine_COLREGS_violation(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const double d_AB);

	bool determine_COLREGS_violation(const Eigen::VectorXd& xs_A, const Eigen::VectorXd& xs_B);

	bool determine_transitional_cost_indicator(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const int i,
		const double chi_m);
	bool determine_transitional_cost_indicator(const Eigen::VectorXd& xs_A, const Eigen::VectorXd& xs_B, const int i, const double chi_m);

	void update_transitional_variables(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const int i,
		const double chi_m);

	void update_transitional_variables(const Eigen::Matrix<double, 6, 1>& xs);

	
	double calculate_total_cost(
		const Eigen::MatrixXd& P_c,
		const Eigen::Matrix<double, 4, 1>& static_obstacle);

	void calculate_collision_probabilities(Eigen::MatrixXd& P_c_i);

	double calculate_collision_cost(const Eigen::Vector2d v_1, const Eigen::Vector2d v_2);

	void calculate_collision_cost(Eigen::VectorXd& cost, const Eigen::Matrix<double, 2, -1>& v_1, const Eigen::Matrix<double, 2, -1>& v_2);

	// Methods dealing with control deviation cost
	double calculate_control_deviation_cost();

	double Delta_u(const double u_1, const double u_2) const 		{ return K_du * fabs(u_1 - u_2); }

	double K_chi(const double chi) const 							{ if (chi > 0) return K_chi_strb * pow(chi, 2); else return K_chi_port * pow(chi, 2); };

	double Delta_chi(const double chi_1, const double chi_2) const 	{ if (chi_1 > 0) return K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	double calculate_chattering_cost();

	// Methods dealing with geographical constraints
	double calculate_grounding_cost(const Eigen::Matrix<double, 6, -1>& trajectory, const Eigen::Matrix<double, 4, -1>& static_obstacles);

    int find_triplet_orientation(const Eigen::Vector2d p, const Eigen::Vector2d q, const Eigen::Vector2d r);                           

    bool determine_if_on_segment(const Eigen::Vector2d p, const Eigen::Vector2d q, const Eigen::Vector2d r);   

    bool determine_if_behind(const Eigen::Vector2d p_1, const Eigen::Vector2d v_1, const Eigen::Vector2d v_2, const double d_to_line);                         

    bool determine_if_lines_intersect(const Eigen::Vector2d p_1, const Eigen::Vector2d q_1, const Eigen::Vector2d p_2, const Eigen::Vector2d q_2);   

    double distance_from_point_to_line(const Eigen::Vector2d p, const Eigen::Vector2d q_1, const Eigen::Vector2d q_2);                  

    double distance_to_static_obstacle(const Eigen::Vector2d p, const Eigen::Vector2d v_1, const Eigen::Vector2d v_2);


    void update_obstacles(
		const Eigen::Matrix<double, 9, -1>& obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::Matrix<double, -1, -1> &obstacle_intention_probabilities);

public:

	PSBMPC();

	~PSBMPC();

	CPE* cpe;

	CPE_Method get_cpe_method() const { return cpe_method; }; 

	Prediction_Method get_prediction_method() const { return prediction_method; };

	Guidance_Method get_guidance_method() const { return guidance_method; };

	void set_cpe_method(CPE_Method cpe_method) 						{ if (cpe_method >= CE && cpe_method <= MCSKF4D) this->cpe_method = cpe_method; };

	void set_prediction_method(Prediction_Method prediction_method) { if(prediction_method >= Linear && prediction_method <= ERK4) this->prediction_method = prediction_method; };

	void set_guidance_method(Guidance_Method guidance_method) 		{ if(guidance_method >= LOS && guidance_method <= HH) this->guidance_method = guidance_method; };

	int get_ipar(const int index) const;
	
	double get_dpar(const int index) const;

	std::vector<Eigen::VectorXd> get_mpar(const int index) const;

	void set_par(const int index, const int value);

	void set_par(const int index, const double value);

	void set_par(const int index, const std::vector<Eigen::VectorXd> value);

	bool get_obstacle_filter_status() const { return obstacle_filter_on; };

	void toggle_obstacle_filter(const bool value) { obstacle_filter_on = value; };

	bool get_obstacle_colav_status() const { return obstacle_colav_on; };

	void toggle_obstacle_colav(const bool value) { obstacle_colav_on = value; };

	void calculate_optimal_offsets(
		double &u_opt, 
		double &psi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		Eigen::Matrix<double, -1, -1> &obstacle_status,
		Eigen::Matrix<double, -1, 1> &colav_status,
		const double u_d, 
		const double psi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::VectorXd &ownship_state,
		const Eigen::Matrix<double, 9, -1> &obstacle_states, 
		const Eigen::Matrix<double, 16, -1> &obstacle_covariances,
		const Eigen::Matrix<double, -1, -1> &obstacle_intention_probabilities,
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

};

#endif 