/****************************************************************************************
*
*  File name : cpe.cuh
*
*  Function  : Header file for the Collision Probability Estimator (CPE).
*			   The module estimates the collision probability wrt all nearby
*			   obstacles. The module assumes that the own-ship uncertainty is negligible
*  			   compared to that of the obstacles. If this is not the case, then a
*			   linear transformationcan be used to "gather" both vessel's 
*			   uncertainty in one RV.
* 			   NOTE: This version only considers 1 obstacle at the time, tailor-made for
* 			   running in a CUDA thread/kernel.
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

#ifndef _CPE_CUH_
#define _CPE_CUH_

#include "psbmpc_defines.h"
#include "curand_kernel.h"
#include <thrust/device_vector.h>
#include "tml.cuh"

// NOTE: If you want standalone use of this module, define the enum CPE_Method below
/* // See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" and/or 
// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
enum CPE_Method 
{
	CE,														// Consider positional uncertainty only
	MCSKF4D													// Consider uncertainty in both position and velocity along piece-wise linear segments 
}; */
// Otherwise, for usage with the PSB-MPC, include "psbmpc_parameters.h":
#include "psbmpc_parameters.h"


class CPE
{
private:

	// Active CPE method
	CPE_Method method;

	// Number of samples for each method
	int n_CE, n_MCSKF;

	// PRNG-related
	curandState prng_state;

	// CE-method parameters and internal states
	double sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	TML::Vector2d mu_CE_last;
	TML::Matrix2d P_CE_last;

	int N_e, e_count;
	TML::PDMatrix<double, 2, MAX_N_CPE_SAMPLES> elite_samples;

	// MCSKF4D-method parameters and internal states
	double q, r, dt_seg; 
	
	double P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Common internal sample variables
	TML::PDMatrix<double, 4, MAX_N_CPE_SAMPLES> samples;
	TML::PDMatrix<double, 1, MAX_N_CPE_SAMPLES> valid;
	
	// Safety zone parameters
	double d_safe;

	// Cholesky decomposition matrix
	TML::PDMatrix4d L;

	__host__ __device__ void resize_matrices();

	__device__ inline void update_L(const TML::PDMatrix4d &in);

	__device__ inline void norm_pdf_log(TML::PDMatrix<double, 1, MAX_N_CPE_SAMPLES> &result, const TML::PDVector4d &mu, const TML::PDMatrix4d &Sigma);

	__device__ inline void generate_norm_dist_samples(const TML::PDVector4d &mu, const TML::PDMatrix4d &Sigma);

	__device__ void calculate_roots_2nd_order(TML::Vector2d &r, bool &is_complex, const double A, const double B, const double C);

	__device__ double produce_MCS_estimate(
		const TML::Vector4d &xs_i, 
		const TML::Matrix4d &P_i, 
		const TML::Vector2d &p_os_cpa,
		const double t_cpa);

	__device__ void determine_sample_validity_4D(
		const TML::Vector2d &p_os_cpa, 
		const double t_cpa);

	__device__ void determine_sample_validity_2D(
		const TML::Vector2d &p_os);

	__device__ void determine_best_performing_samples(
		const TML::Vector2d &p_os, 
		const TML::Vector2d &p_i, 
		const TML::Matrix2d &P_i);

public:
	
	__host__ __device__ CPE() {}

	__host__ __device__ CPE(const CPE_Method cpe_method, const double dt);

	__device__ inline double get_segment_discretization_time() const { return dt_seg; }

	__device__ inline void seed_prng(const unsigned int seed) { curand_init(seed, 0, 0, &prng_state); }

	__device__ void initialize(
		const TML::PDVector6d &xs_os, 
		const TML::PDVector4d &xs_i, 
		const TML::PDVector16d &P_i,
		const double d_safe_i);

	__device__ double MCSKF4D_estimate(
		const TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> &xs_os,  
		const TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> &xs_i, 
		const TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> &P_i);	

	__device__ double CE_estimate(
		const TML::Vector2d &p_os, 
		const TML::Vector2d &p_i, 
		const TML::Matrix2d &P_i);
	
	__device__ double estimate(
		const TML::PDMatrix<double, 6, MAX_N_SEG_SAMPLES> &xs_os,
		const TML::PDMatrix<double, 4, MAX_N_SEG_SAMPLES> &xs_i,
		const TML::PDMatrix<double, 16, MAX_N_SEG_SAMPLES> &P_i);

	__device__ void estimate_over_trajectories(
		TML::PDMatrix<double, 1, MAX_N_SAMPLES> &P_c_i,
		const TML::PDMatrix<double, 6, MAX_N_SAMPLES> &xs_p,
		const TML::PDMatrix<double, 4, MAX_N_SAMPLES> &xs_i_p,
		const TML::PDMatrix<double, 16, MAX_N_SAMPLES> &P_i_p,
		const double d_safe_i,
		const double dt);
};

#endif