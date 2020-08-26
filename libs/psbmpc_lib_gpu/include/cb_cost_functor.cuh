/****************************************************************************************
*
*  File name : cb_cost_functor.cuh
*
*  Function  : Header file for the control behaviour cost evaluation functor. Used in
*			   the thrust framework for GPU calculations.
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

#ifndef _CB_COST_FUNCTOR_H_
#define _CB_COST_FUNCTOR_H_

#include <thrust/device_vector.h>
#include "psbmpc.h"
#include "cml.cuh"
#include "ownship.cuh"
#include "cuda_obstacle.cuh"
#include "cpe.cuh"
#include <vector>

struct CB_Functor_Vars
{
	int n_M, n_a, n_obst;

	CML::MatrixXd maneuver_times;
	CML::MatrixXd control_behaviours;

	double u_d, chi_d;

	double u_m_last;
	double chi_m_last;

	CPE_Method cpe_method;

	Prediction_Method prediction_method;

	Guidance_Method guidance_method;

	double T, T_static, dt;
	double d_safe, d_close, d_init;
	double K_coll;
	double phi_AH, phi_OT, phi_HO, phi_CR;
	double kappa, kappa_TC;
	double K_u, K_du;
	double K_chi_strb, K_dchi_strb;
	double K_chi_port, K_dchi_port; 
	double K_sgn, T_sgn;
	double G;
	double q, p;
	
	bool obstacle_filter_on;

	double T_lost_limit, T_tracked_limit;

	CML::MatrixXd waypoints;

	Ownship ownship;

	CPE cpe;

	CML::MatrixXd trajectory;

	CML::MatrixXd static_obstacles;

	__host__ void assign_master_data(
		const PSBMPC &master, 
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

	__host__ __device__ CB_Functor_Vars() {};
};

// DEVICE methods
// Functor for use in thrust transform
class CB_Cost_Functor
{
private: 
	
	CML::MatrixXi n_ps;

	CB_Functor_Vars *vars;

	CML::MatrixXb obstacle_colav_on;

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	CML::MatrixXb AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0; 

	Cuda_Obstacle *old_obstacles;
	Cuda_Obstacle *new_obstacles;

	__device__ void predict_trajectories_jointly();

	__device__ bool determine_COLREGS_violation(
		const CML::MatrixXd &v_A, 
		const double psi_A, 
		const CML::MatrixXd &v_B,
		const CML::MatrixXd &L_AB, 
		const double d_AB);

	__device__ bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const CML::MatrixXd &L_AB, 
		const int i,
		const double chi_m);

	__device__ bool determine_transitional_cost_indicator(const CML::MatrixXd &xs_A, const CML::MatrixXd &xs_B, const int i, const double chi_m);

	__device__ void calculate_collision_probabilities(CML::MatrixXd &P_c_i, const int i);

	__device__ double calculate_dynamic_obstacle_cost(const CML::MatrixXd &P_c_i, const int i, const CML::MatrixXd &offset_sequence);

	__device__ inline double calculate_collision_cost(const CML::MatrixXd &v_1, const CML::MatrixXd &v_2) { return vars->K_coll * (v_1 - v_2).norm(); };

	__device__ double calculate_ad_hoc_collision_risk(const double d_AB, const double t);

	// Methods dealing with control deviation cost
	__device__ double calculate_control_deviation_cost(const CML::MatrixXd &offset_sequence);

	__device__ inline double Delta_u(const double u_1, const double u_2) 		{ return vars->K_du * fabs(u_1 - u_2); }

	__device__ inline double K_chi(const double chi) 							{ if (chi > 0) return vars->K_chi_strb * pow(chi, 2); else return vars->K_chi_port * pow(chi, 2); };

	__device__ inline double Delta_chi(const double chi_1, const double chi_2) 	{ if (chi_1 > 0) return vars->K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return vars->K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	__device__ double calculate_chattering_cost(const CML::MatrixXd &offset_sequence);

	// Methods dealing with geographical constraints
	__device__ double calculate_grounding_cost();

	__device__ int find_triplet_orientation(const CML::MatrixXd &p, const CML::MatrixXd &q, const CML::MatrixXd &r);                           

	__device__ bool determine_if_on_segment(const CML::MatrixXd &p, const CML::MatrixXd &q, const CML::MatrixXd &r);   

	__device__ bool determine_if_behind(const CML::MatrixXd &p_1, const CML::MatrixXd &v_1, const CML::MatrixXd &v_2, const double d_to_line);                         

	__device__ bool determine_if_lines_intersect(const CML::MatrixXd &p_1, const CML::MatrixXd &q_1, const CML::MatrixXd &p_2, const CML::MatrixXd &q_2);   

	__device__ double distance_from_point_to_line(const CML::MatrixXd &p, const CML::MatrixXd &q_1, const CML::MatrixXd &q_2);                  

	__device__ double distance_to_static_obstacle(const CML::MatrixXd &p, const CML::MatrixXd &v_1, const CML::MatrixXd &v_2);

public: 
	//__host__ CB_Cost_Functor();

	__host__ CB_Cost_Functor(
		const PSBMPC &master, 
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 4, -1> &static_obstacles);

	__host__ __device__ ~CB_Cost_Functor();
	
	__device__ double operator()(const unsigned int cb_index);
	
};
	
#endif 