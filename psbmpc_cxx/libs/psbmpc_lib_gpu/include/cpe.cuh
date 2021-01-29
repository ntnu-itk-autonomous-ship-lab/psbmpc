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
* 			   Versions for CPU and GPU, different implementations due to different
*			   platforms/hardware. 
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

#include "xoshiro.hpp"
#include <Eigen/Dense>
#include <random>
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


class CPE_CPU
{
private:

	// Active CPE method
	CPE_Method method;

	// Number of samples drawn
	int n_CE, n_MCSKF;

	// PRNG-related
	// IMPORTANT NOTE: Some compilers might implement this 
	// random device using a PRNG, and if a non-deterministic device is not available
	// => the same sequence is produced every time, this should be checked before
	// real-time testing to ensure proper functionality.
	std::random_device seed;

	xoshiro256plus64 generator;

	std::normal_distribution<double> std_norm_pdf;

	//====================================
	// CE-method parameters, internal states and temporaries
	int max_it;
	double sigma_inject, alpha_n, gate, rho;
	
	bool converged_last;

	Eigen::Vector2d mu_CE_last;
	Eigen::Matrix2d P_CE_last;

	int N_e, e_count;
	Eigen::MatrixXd elite_samples;

	// Temporaries between dashed lines
	//--------------------------------------------
	Eigen::Vector2d mu_CE_prev, mu_CE;
	Eigen::Matrix2d P_CE_prev, P_CE;
	Eigen::Matrix2d P_i_inv;

	double d_0i, var_P_i_largest;
	bool inside_safety_zone, inside_alpha_p_confidence_ellipse;

	Eigen::VectorXd weights, integrand, importance;
	//-------------------------------------------
	//====================================

	//====================================
	// MCSKF4D-method parameters, internal states and temporaries
	double q, r, dt_seg; 
	double P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Temporaries between dashed lines
	//--------------------------------------------
	double y_P_c_i; // Collision probability "measurement" from MCS, temporary var.
	
	double t_cpa, d_cpa, K;
	Eigen::Vector2d p_os_cpa;

	int n_seg_samples;

	// Speed and course/heading for the vessels along their linear segments
    double U_os_sl, U_i_sl, psi_os_sl, psi_i_sl;

	Eigen::Vector4d xs_os_sl, xs_i_sl;
	Eigen::Matrix4d P_i_sl;

	bool complex_roots;

    Eigen::Vector2d roots, p_i_sample, v_i_sample;
    double d, A, B, C, constant;
	//--------------------------------------------
	//====================================

	// Common internal sample variables
	Eigen::MatrixXd samples;
	Eigen::VectorXd valid;
	
	// Safety zone parameters
	double d_safe;

	// Cholesky decomposition matrix
	Eigen::MatrixXd L;

	//====================================
	// Other pre-allocated temporaries:
	double P_c_est, P_c_CE, y_P_c, sum;
	int n, n_samples, n_samples_traj, n_cols, k_j, k_j_, sample_count;	
	Eigen::MatrixXd Sigma_inv;

	double exp_val, log_val;

	Eigen::MatrixXd xs_os_seg, xs_i_seg, P_i_seg;
	Eigen::Matrix2d P_i_2D;
	Eigen::Vector2d v_os_prev, v_i_prev;
	//====================================
	void assign_data(const CPE_CPU &cpe);

	void resize_matrices();

	inline void update_L(const Eigen::MatrixXd &in);

	inline void norm_pdf_log(Eigen::VectorXd &result, const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	inline void generate_norm_dist_samples(const Eigen::VectorXd &mu, const Eigen::MatrixXd &Sigma);

	void calculate_roots_2nd_order();

	double produce_MCS_estimate(
		const Eigen::Vector4d &xs_i, 
		const Eigen::Matrix4d &P_i, 
		const Eigen::Vector2d &p_os_cpa,
		const double t_cpa);

	void determine_sample_validity_4D(
		const Eigen::Vector2d &p_os_cpa, 
		const double t_cpa);

	double MCSKF4D_estimation(
		const Eigen::MatrixXd &xs_os,  
		const Eigen::MatrixXd &xs_i, 
		const Eigen::MatrixXd &P_i);	

	void determine_sample_validity_2D(
		const Eigen::Vector2d &p_os);

	void determine_best_performing_samples(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i);

	void update_importance_density();

	double CE_estimation(
		const Eigen::Vector2d &p_os, 
		const Eigen::Vector2d &p_i, 
		const Eigen::Matrix2d &P_i,
		const Eigen::Vector2d &v_os_prev,
    	const Eigen::Vector2d &v_i_prev, 
		const double dt);

public:

	CPE_CPU() {}

	CPE_CPU(const CPE_Method cpe_method, const double dt);

	CPE_CPU(const CPE_CPU &other);

	CPE_CPU& operator=(const CPE_CPU &rhs);

	void set_method(const CPE_Method cpe_method) { if (cpe_method >= CE && cpe_method <= MCSKF4D) { method = cpe_method;  resize_matrices(); }};

	void set_segment_discretization_time(const double dt_seg) { this->dt_seg = dt_seg; };

	double get_segment_discretization_time() const { return dt_seg; };

	void initialize(
		const Eigen::Matrix<double, 6, 1> &xs_os, 
		const Eigen::Vector4d &xs_i, 
		const Eigen::VectorXd &P_i,
		const double d_safe_i);

	void estimate_over_trajectories(
		Eigen::Matrix<double, 1, -1> &P_c_i,
		const Eigen::Matrix<double, 6, -1> &xs_p,
		const Eigen::Matrix<double, 4, -1> &xs_i_p,
		const Eigen::Matrix<double, 16, -1> &P_i_p,
		const double d_safe_i,
		const double dt,
		const int p_step);
};

class CPE_GPU
{
private:

	// Active CPE method
	CPE_Method method;

	// Number of samples for each method
	int n_CE, n_MCSKF;

	// PRNG-related
	curandState prng_state;

	//====================================
	// CE-method parameters, internal states and temporaries
	int max_it;
	float rho, sigma_inject, alpha_n;
	double gate;
	
	bool converged_last;

	TML::Vector2f mu_CE_last;
	TML::Matrix2f P_CE_last;

	// Temporaries:
	int N_e, e_count;
	TML::Vector2d elite_sample_innovation;
	TML::PDMatrix<float, 2, MAX_N_CPE_SAMPLES> elite_samples;

	TML::Vector2f mu_CE_prev, mu_CE;
	TML::Matrix2f P_CE_prev, P_CE;
	TML::Matrix2d P_i_inv;

	float d_0i, var_P_i_largest;
	bool inside_safety_zone, inside_alpha_p_confidence_ellipse;

	TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> weights, integrand, importance;
	TML::Vector2f v_os_prev, v_i_prev;
	//====================================

	//====================================
	// MCSKF4D-method parameters, internal states and temporaries
	float dt_seg, q, r; 
	float P_c_p, var_P_c_p, P_c_upd, var_P_c_upd; 

	// Temporaries
	float y_P_c_i; // Collision probability "measurement" from MCS, temporary var.
	
	float t_cpa, d_cpa, K;
	TML::Vector2f p_os_cpa;

	int n_seg_samples;

	// Speed and course/heading for the vessels along their linear segments
    float U_os_sl, U_i_sl, psi_os_sl, psi_i_sl;

	TML::Vector4f xs_os_sl, xs_i_sl;
	TML::Matrix4f P_i_sl;

	bool complex_roots;

	// These parameters are double because the calculations
	// involved in "determine_sample_validity_4D" were found to be
	// very sensitive to change in floating point number precision
    TML::Vector2d roots, p_i_sample, v_i_sample;
    double d, A, B, C, constant;
	//====================================

	// Common internal sample variables
	TML::PDMatrix<float, 4, MAX_N_CPE_SAMPLES> samples;
	TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> valid;
	
	// Safety zone parameter
	double d_safe;

	// Cholesky decomposition matrix
	float sum;
	TML::PDMatrix4f L; 
	
	//====================================
	// Other pre-allocated temporaries:
	float P_c_est, P_c_CE, y_P_c;
	int n, n_samples;	

	// These temps are also double due to being
	// involved in number precision requiring calculations
	TML::Vector2d sample_innovation;
	double exp_val, log_val;
	TML::Matrix2d Sigma_2D_inv;
	TML::Matrix4d Sigma_4D_inv;
	//====================================
	__host__ __device__ void resize_matrices();

	__host__ __device__ void update_L(const TML::Matrix2f &in);
	__host__ __device__ void update_L(const TML::Matrix4f &in);

	__device__ inline void norm_pdf_log(TML::PDMatrix<float, 1, MAX_N_CPE_SAMPLES> &result, const TML::Vector2d &mu, const TML::Matrix2d &Sigma);

	__device__ inline void generate_norm_dist_samples(const TML::Vector2f &mu, const TML::Matrix2f &Sigma);
	__device__ inline void generate_norm_dist_samples(const TML::Vector4f &mu, const TML::Matrix4f &Sigma);

	__host__ __device__ void calculate_roots_2nd_order();

	__device__ float produce_MCS_estimate(
		const TML::Vector4f &xs_i, 
		const TML::Matrix4f &P_i, 
		const TML::Vector2f &p_os_cpa,
		const float t_cpa);

	__host__ __device__ void determine_sample_validity_4D(
		const TML::Vector2d &p_os_cpa, 
		const double t_cpa);

	__host__ __device__ void determine_sample_validity_2D(
		const TML::Vector2d &p_os);

	__host__ __device__ void determine_best_performing_samples(
		const TML::Vector2d &p_os, 
		const TML::Vector2d &p_i, 
		const TML::Matrix2d &P_i_inv);

	__device__ void update_importance_density(
		TML::Vector2f &mu_CE,
		TML::Matrix2f &P_CE, 
		TML::Vector2f &mu_CE_prev, 
		TML::Matrix2f &P_CE_prev);

public:

	// Temporary
	__host__ __device__ void set_samples(const TML::PDMatrix<float, 4, MAX_N_CPE_SAMPLES> &samples) { this->samples = samples; }
	//

	__host__ __device__ CPE_GPU() {}

	__host__ __device__ CPE_GPU(const CPE_Method cpe_method, const float dt);

	__host__ __device__ CPE_GPU(const CPE_CPU &cpe);

	__host__ __device__ void set_method(const CPE_Method cpe_method) { method = cpe_method; resize_matrices(); }

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
		const TML::Matrix2f &P_i,
		const TML::Vector2f &v_os_prev,
		const TML::Vector2f &v_i_prev,
		const float dt);

	__device__ void estimate_over_trajectories(
		TML::PDMatrix<float, 1, MAX_N_SAMPLES> &P_c_i,
		const TML::PDMatrix<float, 6, MAX_N_SAMPLES> &xs_p,
		const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &xs_i_p,
		const TML::PDMatrix<float, 16, MAX_N_SAMPLES> &P_i_p,
		const float d_safe_i,
		const float dt);
};

#endif