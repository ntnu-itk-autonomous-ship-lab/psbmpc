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

#include "cb_cost_functor_structures.cuh"

// DEVICE methods
// Functor for use in thrust transform
class CB_Cost_Functor
{
private: 

	CB_Functor_Pars pars;

	CB_Functor_Data *fdata;

	Ownship ownship;

	__device__ void predict_trajectories_jointly();

	__device__ bool determine_COLREGS_violation(
		const CML::Vector2d &v_A, 
		const double psi_A, 
		const CML::Vector2d &v_B,
		const CML::Vector2d &L_AB, 
		const double d_AB);

	__device__ bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const CML::Vector2d &L_AB, 
		const double chi_m,
		const int i);

	__device__ void calculate_collision_probabilities(CML::MatrixXd &P_c_i, const int i);

	__device__ double calculate_dynamic_obstacle_cost(const CML::MatrixXd &P_c_i, const int i, const CML::MatrixXd &offset_sequence);

/* 	__device__ inline double calculate_collision_cost(const CML::Vector2d &v_1, const CML::Vector2d &v_2) { return pars.K_coll * (v_1 - v_2).norm(); };

	__device__ double calculate_ad_hoc_collision_risk(const double d_AB, const double t);

	// Methods dealing with control deviation cost
	__device__ double calculate_control_deviation_cost(const CML::MatrixXd &offset_sequence);

	__device__ inline double Delta_u(const double u_1, const double u_2) 		{ return pars.K_du * fabs(u_1 - u_2); }

	__device__ inline double K_chi(const double chi) 							{ if (chi > 0) return pars.K_chi_strb * pow(chi, 2); else return pars.K_chi_port * pow(chi, 2); };

	__device__ inline double Delta_chi(const double chi_1, const double chi_2) 	{ if (chi_1 > 0) return pars.K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return pars.K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };
 */
	//
	__device__ double calculate_chattering_cost(const CML::MatrixXd &offset_sequence);

	// Methods dealing with geographical constraints
	__device__ double calculate_grounding_cost();

	__device__ int find_triplet_orientation(const CML::Vector2d &p, const CML::Vector2d &q, const CML::Vector2d &r);                           

	__device__ bool determine_if_on_segment(const CML::Vector2d &p, const CML::Vector2d &q, const CML::Vector2d &r);   

	__device__ bool determine_if_behind(const CML::Vector2d &p_1, const CML::Vector2d &v_1, const CML::Vector2d &v_2, const double d_to_line);                         

	__device__ bool determine_if_lines_intersect(const CML::Vector2d &p_1, const CML::Vector2d &q_1, const CML::Vector2d &p_2, const CML::Vector2d &q_2);   

	__device__ double distance_from_point_to_line(const CML::Vector2d &p, const CML::Vector2d &q_1, const CML::Vector2d &q_2);                  

	__device__ double distance_to_static_obstacle(const CML::Vector2d &p, const CML::Vector2d &v_1, const CML::Vector2d &v_2);

public: 
	__host__ CB_Cost_Functor() : fdata(nullptr) {}

	__host__ CB_Cost_Functor(
		const PSBMPC master, 
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> waypoints, 
		const Eigen::Matrix<double, 4, -1> static_obstacles,
		const Obstacle_Data data);

	__host__ __device__ ~CB_Cost_Functor() { cudaFree(fdata); }
	
	__device__ double operator()(const thrust::tuple<const unsigned int, CML::Pseudo_Dynamic_Matrix<double, 20, 1>> cb_tuple);
	
};
	
#endif 