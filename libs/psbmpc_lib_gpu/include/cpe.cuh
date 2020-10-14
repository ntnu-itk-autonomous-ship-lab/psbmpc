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
	float sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	TML::Vector2f mu_CE_last;
	TML::Matrix2f P_CE_last;

	int N_e, e_count;
	TML::PDMatrix<float, 2, MAX_N_CPE_SAMPLES> elite_samples;

	// MCSKF4D-method parameters and internal states
	float q, r, dt_seg; 
	
	float P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Common internal sample variables
	TML::PDMatrix<float, 4, MAX_N_CPE_SAMPLES> samples;
	TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> valid;
	
	// Safety zone parameters
	float d_safe;

	// Cholesky decomposition matrix
	TML::PDMatrix4f L;

	__host__ __device__ void resize_matrices();

	__device__ inline void update_L(const TML::PDMatrix4f &in);

	__device__ inline void norm_pdf_log(TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> &result, const TML::PDVector4f &mu, const TML::PDMatrix4f &Sigma);

	__device__ inline void generate_norm_dist_samples(const TML::PDVector4f &mu, const TML::PDMatrix4f &Sigma);

	__device__ void calculate_roots_2nd_order(TML::Vector2f &r, bool &is_complex, const float A, const float B, const float C);

	__device__ float produce_MCS_estimate(
		const TML::Vector4f &xs_i, 
		const TML::Matrix4f &P_i, 
		const TML::Vector2f &p_os_cpa,
		const float t_cpa);

	__device__ void determine_sample_validity_4D(
		const TML::Vector2f &p_os_cpa, 
		const float t_cpa);

	__device__ void determine_sample_validity_2D(
		const TML::Vector2f &p_os);

	__device__ void determine_best_performing_samples(
		const TML::Vector2f &p_os, 
		const TML::Vector2f &p_i, 
		const TML::Matrix2f &P_i);

public:
	
	__host__ __device__ CPE() {}

	__host__ __device__ CPE(const CPE_Method cpe_method, const float dt);

	__device__ inline float get_segment_discretization_time() const { return dt_seg; }

	__device__ inline void seed_prng(const unsigned int seed) { curand_init(seed, 0, 0, &prng_state); }

	__device__ void initialize(
		const TML::PDVector6f &xs_os, 
		const TML::PDVector4f &xs_i, 
		const TML::PDVector16f &P_i,
		const float d_safe_i);

	__device__ float MCSKF4D_estimate(
		const TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> &xs_os,  
		const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_i, 
		const TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> &P_i);	

	__device__ float CE_estimate(
		const TML::Vector2f &p_os, 
		const TML::Vector2f &p_i, 
		const TML::Matrix2f &P_i);
	
	__device__ float estimate(
		const TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> &xs_os,
		const TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> &xs_i,
		const TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> &P_i);

	__device__ void estimate_over_trajectories(
		TML::PDMatrix<float, 1, MAX_N_SAMPLES> &P_c_i,
		const TML::PDMatrix<float, 6, MAX_N_SAMPLES> &xs_p,
		const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &xs_i_p,
		const TML::PDMatrix<float, 16, MAX_N_SAMPLES> &P_i_p,
		const float d_safe_i,
		const float dt);
};

#endif