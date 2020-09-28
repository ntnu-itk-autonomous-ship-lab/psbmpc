/****************************************************************************************
*
*  File name : cpe.cuh
*
*  Function  : Header file for the Collision Probability Estimator (CPE).
*			   The module estimates the collision probability wrt all nearby
*			   obstacles. The module assumes that the own-ship uncertainty is negligible
*  			   compared to that of the obstacles. If this is not the case, then a
*			   linear transformationcan be used to "gather" both vessel's 
*			   uncertainty in one RV
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

#include "curand_kernel.h"
#include <thrust/device_vector.h>
#include "cml.cuh"

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

	// Number of obstacles to consider in estimation
	int n_obst; // Ad hoc max number of obstacles: 50

	// Number of samples drawn
	int n_CE, n_MCSKF;

	// PRNG-related
	curandState prng_state;

	// CE-method parameters and internal states
	double sigma_inject, alpha_n, gate, rho, max_it;
	
	bool converged_last;

	CML::MatrixXd *mu_CE_last;
	CML::MatrixXd *P_CE_last;

	int N_e, e_count;
	CML::MatrixXd elite_samples;

	// MCSKF4D-method parameters and internal states
	double q, r, dt_seg; 
	
	CML::MatrixXd P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Common internal sample variables
	CML::MatrixXd samples;
	CML::MatrixXd valid;
	
	// Safety zone parameters
	double d_safe;

	// Cholesky decomposition matrix
	CML::MatrixXd L;

	__host__ __device__ void assign_data(const CPE &cpe);

	__host__ __device__ void resize_matrices();

	__device__ inline void update_L(const CML::MatrixXd &in);

	__device__ inline void norm_pdf_log(CML::MatrixXd &result, const CML::MatrixXd &mu, const CML::MatrixXd &Sigma);

	__device__ inline void generate_norm_dist_samples(const CML::MatrixXd &mu, const CML::MatrixXd &Sigma);

	__device__ void calculate_roots_2nd_order(CML::MatrixXd &r, bool &is_complex, const double A, const double B, const double C);

	__device__ double produce_MCS_estimate(
		const CML::MatrixXd &xs_i, 
		const CML::MatrixXd &P_i, 
		const CML::MatrixXd &p_os_cpa,
		const double t_cpa);

	__device__ void determine_sample_validity_4D(
		const CML::MatrixXd &p_os_cpa, 
		const double t_cpa);

	__device__ double MCSKF4D_estimation(
		const CML::MatrixXd &xs_os,  
		const CML::MatrixXd &xs_i, 
		const CML::MatrixXd &P_i,
		const int i);	

	__device__ void determine_sample_validity_2D(
		const CML::MatrixXd &p_os);

	__device__ void determine_best_performing_samples(
		const CML::MatrixXd &p_os, 
		const CML::MatrixXd &p_i, 
		const CML::MatrixXd &P_i);

	__device__ double CE_estimation(
		const CML::MatrixXd &p_os, 
		const CML::MatrixXd &p_i, 
		const CML::MatrixXd &P_i,
		const int i);

public:
	
	__host__ __device__ CPE() : mu_CE_last(nullptr), P_CE_last(nullptr) {}

	__host__ __device__ CPE(const CPE_Method cpe_method, const int n_CE, const int n_MCSKF, const int n_obst, const double dt);

	__host__ __device__ CPE(const CPE &cpe);

	__host__ __device__ ~CPE();

	__host__ __device__ CPE& operator=(const CPE &cpe);

	__host__ __device__ void clean();

	__host__ __device__ inline void set_method(const CPE_Method cpe_method) 
	{ if (cpe_method >= CE && cpe_method <= MCSKF4D) { method = cpe_method;  resize_matrices(); }}

	__host__ __device__ void set_number_of_obstacles(const int n_obst);

	__device__ inline double get_segment_discretization_time() const { return dt_seg; }

	__device__ inline void seed_prng(const unsigned int seed) { curand_init(seed, 0, 0, &prng_state); }

	__device__ void initialize(
		const CML::MatrixXd &xs_os, 
		const CML::MatrixXd &xs_i, 
		const CML::MatrixXd &P_i,
		const double d_safe_i, 
		const int i);
	
	__device__ double estimate(
		const CML::MatrixXd &xs_os,
		const CML::MatrixXd &xs_i,
		const CML::MatrixXd &P_i,
		const int i);

	__device__ void estimate_over_trajectories(
		CML::MatrixXd &P_c_i,
		const CML::MatrixXd &xs_p,
		const CML::MatrixXd &xs_i_p,
		const CML::MatrixXd &P_i_p,
		const double d_safe_i,
		const int i,
		const double dt);
};

#endif