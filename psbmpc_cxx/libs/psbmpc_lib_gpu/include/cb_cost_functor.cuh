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
#include "mpc_cost.cuh"

// DEVICE methods
// Functor for use in thrust transform
class CB_Cost_Functor
{
private: 

	CB_Functor_Pars *pars;

	CB_Functor_Data *fdata;

	Cuda_Obstacle *obstacles;

	Prediction_Obstacle *pobstacles;

	CPE_GPU *cpe;

	TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory;

	//==============================================
	// Pre-allocated temporaries
	//==============================================
	MPC_Cost<CB_Functor_Pars> mpc_cost;

	Ownship ownship;

	unsigned int cb_index;
	TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence;

	int n_samples, n_seg_samples;

	float cost_cb, cost_ps, cost_cd, cost_ch, delta_t, cost_g, ahcr;

	float d_safe_i, chi_m, Pr_CC_i;

	// Allocate vectors for keeping track of max cost wrt obstacle i in prediction scenario ps, 
	// collision probabilities and obstacle COLREGS violation indicator
	TML::PDMatrix<float, MAX_N_OBST, 1> cost_i;
	TML::PDMatrix<float, MAX_N_PS, 1> P_c_i, max_cost_ps, weights_ps;
	TML::PDMatrix<bool, MAX_N_PS, 1> mu_i;
	TML::PDMatrix<Intention, 1, MAX_N_PS> ps_ordering;

	// Allocate vectors for the obstacle intention weighted cost, and intention probability vector
	TML::Vector3f ps_intention_count, cost_a_weight_sums, cost_a, Pr_a;
	Intention a_i_ps;
	bool mu_i_ps;

	// Allocate predicted ownship state and predicted obstacle i state and covariance for their prediction scenarios (ps)
	// Only keeps n_seg_samples at a time, sliding window. Minimum 2
	// If cpe_method = MCSKF, then dt_seg must be equal to dt;
	// If cpe_method = CE, then only the first column in these matrices are used (only the current predicted time is considered)
	TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> xs_p; 
	TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_i_p;
	TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> P_i_p;

	// For the CE-method:
	TML::Vector2f p_os, p_i, v_os_prev, v_i_prev;
    TML::Matrix2f P_i_2D;
	//==============================================
	//==============================================

	__device__ void predict_trajectories_jointly();

public: 
	__host__ CB_Cost_Functor() : fdata(nullptr), obstacles(nullptr), cpe(nullptr) {}

	__host__ CB_Cost_Functor(
		CB_Functor_Pars *pars, 
		CB_Functor_Data *fdata, 
		Cuda_Obstacle *obstacles, 
		Prediction_Obstacle *pobstacles,
		CPE_GPU *cpe,
		TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory,
		const int wp_c_0);

	__host__ __device__ ~CB_Cost_Functor() { fdata = nullptr; obstacles = nullptr; pobstacles = nullptr; cpe = nullptr; trajectory = nullptr; }
	
	__device__ float operator()(const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple);
	
};
	
#endif 