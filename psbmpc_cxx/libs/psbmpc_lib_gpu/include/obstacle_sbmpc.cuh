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

#ifndef _OBSTACLE_SBMPC_CUH_
#define _OBSTACLE_SBMPC_CUH_

#include <thrust/device_vector.h>
#include "psbmpc_index.h"
#include "sbmpc_parameters.h"
#include "joint_prediction_manager.cuh"
#include "obstacle_ship.cuh"
#include "tml.cuh"
#include <vector>


class Obstacle_SBMPC
{
private:

	TML::PDMatrix<float, 1, 2 * MAX_N_M> offset_sequence;

	TML::PDMatrix<int, 1, MAX_N_M> offset_sequence_counter;

	TML::PDMatrix<float, 1, MAX_N_M> maneuver_times;
	
	double u_m_last;
	double chi_m_last;

	double min_cost;
	
	bool obstacle_colav_on;

	Obstacle_Ship ownship;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> trajectory;

	__host__ __device__ void assign_data(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void initialize_prediction(Obstacle_Data<Prediction_Obstacle> &data, const int k_0);

	__host__ __device__ void reset_control_behavior();

	__host__ __device__ void increment_control_behavior();

	__host__ __device__ bool determine_colav_active(const Obstacle_Data<Prediction_Obstacle> &data, const int n_static_obst);

	__host__ __device__ bool determine_COLREGS_violation(
		const Eigen::Vector2d &v_A, 
		const double psi_A, 
		const Eigen::Vector2d &v_B,
		const Eigen::Vector2d &L_AB,	
		const double d_AB);

	__host__ __device__ bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const Eigen::Vector2d &L_AB, 
		const double chi_m,
		const Obstacle_Data<Prediction_Obstacle> &data,
		const int i);

	__host__ __device__ double calculate_dynamic_obstacle_cost(const Obstacle_Data<Prediction_Obstacle> &data, const int i);

	__host__ __device__ double calculate_collision_cost(const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2) { return pars.K_coll * (v_1 - v_2).norm(); }

	__host__ __device__ double calculate_ad_hoc_collision_risk(const double d_AB, const double t);

	// Methods dealing with control deviation cost
	__host__ __device__ double calculate_control_deviation_cost();	

	__host__ __device__ double Delta_u(const double u_1, const double u_2) const 		{ return pars.K_du * fabs(u_1 - u_2); }

	__host__ __device__ double K_chi(const double chi) const 							{ if (chi > 0) return pars.K_chi_strb * pow(chi, 2); else return pars.K_chi_port * pow(chi, 2); };

	__host__ __device__ double Delta_chi(const double chi_1, const double chi_2) const 	{ if (chi_1 > 0) return pars.K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return pars.K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	__host__ __device__ double calculate_chattering_cost();

	// Methods dealing with geographical constraints
	__host__ __device__ double calculate_grounding_cost(const Eigen::Matrix<double, 4, -1>& static_obstacles);

    __host__ __device__ int find_triplet_orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);                           

    __host__ __device__ bool determine_if_on_segment(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r);   

    __host__ __device__ bool determine_if_behind(const Eigen::Vector2d &p_1, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2, const double d_to_line);                         

    __host__ __device__ bool determine_if_lines_intersect(const Eigen::Vector2d &p_1, const Eigen::Vector2d &q_1, const Eigen::Vector2d &p_2, const Eigen::Vector2d &q_2);   

    __host__ __device__ double distance_from_point_to_line(const Eigen::Vector2d &p, const Eigen::Vector2d &q_1, const Eigen::Vector2d &q_2);                  

    __host__ __device__ double distance_to_static_obstacle(const Eigen::Vector2d &p, const Eigen::Vector2d &v_1, const Eigen::Vector2d &v_2);

	__host__ __device__ void assign_optimal_trajectory(Eigen::Matrix<double, 4, -1> &optimal_trajectory);

public:

	SBMPC_Parameters pars;

	__host__ __device__ Obstacle_SBMPC();

	__host__ __device__ ~Obstacle_SBMPC();

	__host__ __device__ Obstacle_SBMPC(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ Obstacle_SBMPC& operator=(const Obstacle_SBMPC &o_sbmpc);

	__host__ __device__ void calculate_optimal_offsets(
		double &u_opt, 	
		double &chi_opt, 
		Eigen::Matrix<double, 4, -1> &predicted_trajectory,
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Eigen::Vector4d &ownship_state,
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		Obstacle_Data<Prediction_Obstacle> &data,
		const int k_0);
};

#endif 