/****************************************************************************************
*
*  File name : obstacle_sbmpc.cuh
*
*  Function  : Header file for Scenario-based Model Predictive Control used by obstacles
*			   in the PSB-MPC predictions.
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

#ifndef _OBSTACLE_SBMPC_H_
#define _OBSTACLE_SBMPC_H_

#include <thrust/device_vector.h>
#include "psbmpc_index.h"
#include "prediction_obstacle.cuh"
#include "obstacle_ship.cuh"
#include "Eigen/Dense"
#include <vector>
// MUST REPLACE std::vector with *ers...
class Obstacle_SBMPC
{
private:

	int n_cbs, n_M;

	Eigen::VectorXd *u_offsets;
	Eigen::VectorXd *chi_offsets;

	Eigen::VectorXd offset_sequence_counter, offset_sequence, maneuver_times;

	double u_m_last;
	double chi_m_last;

	double min_cost;

	Eigen::VectorXd dpar_low, dpar_high;
	Eigen::VectorXd ipar_low, ipar_high;

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
	
	bool obstacle_colav_on;

	Obstacle_Ship *ownship;

	Eigen::Matrix<double, 4, -1> trajectory;

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	bool *AH_0, *S_TC_0, *S_i_TC_0, *O_TC_0, *Q_TC_0, *IP_0, *H_TC_0, *X_TC_0;

	Prediction_Obstacle* old_obstacles;
	Prediction_Obstacle* new_obstacles;

	__host__ __device__ void initialize_par_limits();

	__host__ __device__ void initialize_pars();

	__device__ void initialize_prediction();

	__device__ void reset_control_behavior();

	__device__ void increment_control_behavior();

	__device__ bool determine_colav_active(const int n_static_obst);

	__device__ bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const int i,
		const double chi_m);

	__device__ bool determine_transitional_cost_indicator(const Eigen::VectorXd &xs_A, const Eigen::VectorXd &xs_B, const int i, const double chi_m);

	__device__ double calculate_dynamic_obstacle_cost(const int i);

	__device__ double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2);

	// Methods dealing with control deviation cost
	__device__ double calculate_control_deviation_cost();	

	__device__ double Delta_u(const double u_1, const double u_2) const 		{ return K_du * fabs(u_1 - u_2); }

	__device__ double K_chi(const double chi) const 							{ if (chi > 0) return K_chi_strb * pow(chi, 2); else return K_chi_port * pow(chi, 2); };

	__device__ double Delta_chi(const double chi_1, const double chi_2) const 	{ if (chi_1 > 0) return K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	__device__ double calculate_chattering_cost();

	// Methods dealing with geographical constraints
	__device__ double calculate_grounding_cost(const Eigen::Matrix<double, 4, -1>& static_obstacles);

    __device__ int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);                           

    __device__ bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);   

    __device__ bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line);                         

    __device__ bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2);   

    __device__ double distance_from_point_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2);                  

    __device__ double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2);

	__device__ void assign_obstacle_vector(std::vector<Prediction_Obstacle*> &lhs, const std::vector<Prediction_Obstacle*> &rhs);

    __device__ void update_obstacles(const Eigen::Matrix<double, 9, -1>& obstacle_states);

	__device__ void update_obstacle_status(Eigen::Matrix<double,-1,-1> &obstacle_status, const Eigen::VectorXd &HL_0);

	__device__ void update_transitional_variables();

public:

	__host__ __device__ Obstacle_SBMPC();

	__host__ __device__ Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ ~Obstacle_SBMPC();

	__host__ __device__ void clean();

	__host__ __device__ Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ bool determine_COLREGS_violation(const Eigen::VectorXd &xs_A, const Eigen::VectorXd &xs_B);

	__host__ __device__ bool determine_COLREGS_violation(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB,	
		const double d_AB);

	__host__ __device__ Prediction_Method get_prediction_method() const { return prediction_method; };

	__host__ __device__ Guidance_Method get_guidance_method() const { return guidance_method; };

	__host__ __device__ void set_prediction_method(Prediction_Method prediction_method) { if (prediction_method >= Linear && prediction_method <= ERK4) this->prediction_method = prediction_method; };

	__host__ __device__ void set_guidance_method(Guidance_Method guidance_method) 		{ if (guidance_method >= LOS && guidance_method <= CH) this->guidance_method = guidance_method; };

	__host__ __device__ int get_ipar(const int index) const;
	
	__host__ __device__ double get_dpar(const int index) const;

	__host__ __device__ void set_par(const int index, const int value);

	__host__ __device__ void set_par(const int index, const double value);

	__device__ void calculate_optimal_offsets(
		double &u_opt, 	
		double &chi_opt, 
		Eigen::Matrix<double, 2, -1> &predicted_trajectory,
		Eigen::Matrix<double, -1, -1> &obstacle_status,
		Eigen::Matrix<double, -1, 1> &colav_status,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Matrix<double, 4, 1> &ownship_state,
		const Eigen::Matrix<double, 9, -1> &obstacle_states, 
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

};

#endif 