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

#ifndef _CB_COST_FUNCTOR_CUH_
#define _CB_COST_FUNCTOR_CUH_

#include "cb_cost_functor_structures.cuh"

// DEVICE methods
// Functor for use in thrust transform
class CB_Cost_Functor
{
private: 

	CB_Functor_Pars *pars;

	CB_Functor_Data *fdata;

	Cuda_Obstacle *obstacles;

	CPE *cpe;

	__device__ void predict_trajectories_jointly();

	__device__ bool determine_COLREGS_violation(
		const TML::Vector2d &v_A, 
		const double psi_A, 
		const TML::Vector2d &v_B,
		const TML::Vector2d &L_AB, 
		const double d_AB);

	__device__ bool determine_transitional_cost_indicator(
		const double psi_A, 
		const double psi_B, 
		const TML::Vector2d &L_AB, 
		const double chi_m,
		const int i);

	__device__ inline void calculate_collision_probabilities(TML::PDMatrix<double, MAX_N_PS, MAX_N_SAMPLES> &P_c_i, const int i);

	__device__ inline double calculate_dynamic_obstacle_cost(
		const TML::PDMatrix<double, MAX_N_PS, MAX_N_SAMPLES> &P_c_i, 
		const int i, 
		const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence);

	__device__ inline double calculate_dynamic_obstacle_cost(
		const double P_c_i_ps,
		const TML::PDVector6d xs_p,
		const TML::PDVector4d xs_i_p_ps,
		const int i,
		const double chi_m);

	__device__ inline double calculate_collision_cost(const TML::Vector2d &v_1, const TML::Vector2d &v_2) { return pars->K_coll * (v_1 - v_2).norm(); };

	__device__ inline double calculate_ad_hoc_collision_risk(const double d_AB, const double t);

	// Methods dealing with control deviation cost
	__device__ inline double calculate_control_deviation_cost(const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence);

	__device__ inline double Delta_u(const double u_1, const double u_2) 		{ return pars->K_du * fabs(u_1 - u_2); }

	__device__ inline double K_chi(const double chi) 							{ if (chi > 0) return pars->K_chi_strb * pow(chi, 2); else return pars->K_chi_port * pow(chi, 2); };

	__device__ inline double Delta_chi(const double chi_1, const double chi_2) 	{ if (chi_1 > 0) return pars->K_dchi_strb * pow(fabs(chi_1 - chi_2), 2); else return pars->K_dchi_port * pow(fabs(chi_1 - chi_2), 2); };

	//
	__device__ double calculate_chattering_cost(const TML::PDMatrix<double, 2 * MAX_N_M, 1> &offset_sequence);

	// Methods dealing with geographical constraints
	__device__ double calculate_grounding_cost();

	__device__ int find_triplet_orientation(const TML::Vector2d &p, const TML::Vector2d &q, const TML::Vector2d &r);                           

	__device__ bool determine_if_on_segment(const TML::Vector2d &p, const TML::Vector2d &q, const TML::Vector2d &r);   

	__device__ bool determine_if_behind(const TML::Vector2d &p_1, const TML::Vector2d &v_1, const TML::Vector2d &v_2, const double d_to_line);                         

	__device__ bool determine_if_lines_intersect(const TML::Vector2d &p_1, const TML::Vector2d &q_1, const TML::Vector2d &p_2, const TML::Vector2d &q_2);   

	__device__ double distance_from_point_to_line(const TML::Vector2d &p, const TML::Vector2d &q_1, const TML::Vector2d &q_2);                  

	__device__ double distance_to_static_obstacle(const TML::Vector2d &p, const TML::Vector2d &v_1, const TML::Vector2d &v_2);

public: 
	__host__ CB_Cost_Functor() : fdata(nullptr), obstacles(nullptr), cpe(nullptr) {}

	__host__ CB_Cost_Functor(CB_Functor_Pars *pars, CB_Functor_Data *fdata, Cuda_Obstacle *obstacles, CPE *cpe);

	__host__ __device__ ~CB_Cost_Functor() { fdata = nullptr; obstacles = nullptr; cpe = nullptr; }
	
	__device__ double operator()(const thrust::tuple<const unsigned int, TML::PDMatrix<double, 2 * MAX_N_M, 1>> &cb_tuple);
	
};
	
#endif 