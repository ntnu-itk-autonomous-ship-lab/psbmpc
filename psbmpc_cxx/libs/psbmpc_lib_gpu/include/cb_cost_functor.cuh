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

#include "tml.cuh"
#include "obstacle_manager.cuh"

class CB_Functor_Pars;
class CB_Functor_Data;
class Cuda_Obstacle;
class Prediction_Obstacle;
class CPE_GPU;
class Ownship;
class Obstacle_Ship;
class Obstacle_SBMPC;
template <typename Parameters> class MPC_Cost;

// DEVICE methods
// Functor for use in thrust transform
class CB_Cost_Functor
{
private: 

	//==============================================
	// Members allocated in global device memory
	//==============================================
	CB_Functor_Pars *pars;

	CB_Functor_Data *fdata;

	Cuda_Obstacle *obstacles;

	Prediction_Obstacle *pobstacles;

	CPE_GPU *cpe;

	Ownship *ownship;

	TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory;

	Obstacle_Ship *obstacle_ship;

	Obstacle_SBMPC *obstacle_sbmpc;

	MPC_Cost<CB_Functor_Pars> *mpc_cost;

	//==============================================
	// Pre-allocated temporaries (local to the thread stack)
	//==============================================
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
	TML::PDMatrix<Intention, MAX_N_PS, 1> ps_ordering;

	// Allocate vectors for the obstacle intention weighted cost, and intention probability vector
	TML::Vector3f ps_intention_count, cost_a_weight_sums, cost_a, Pr_a;
	Intention a_i_ps;
	bool mu_i_ps;

	// Allocate predicted ownship state and predicted obstacle i state and covariance for their prediction scenarios (ps)
	// Only keeps n_seg_samples at a time, sliding window. Minimum 2
	// If cpe_method = MCSKF, then dt_seg must be equal to dt;
	// If cpe_method = CE, then only the first column in these matrices are used (only the current predicted time is considered)
	TML::PDMatrix<float, 6, MAX_N_SEG_SAMPLES> xs_p_seg; 
	TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_i_p_seg;
	TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> P_i_p_seg;

	// For the CE-method:
	TML::Vector2f p_os, p_i, v_os_prev, v_i_prev;
    TML::Matrix2f P_i_2D;

	// Joint prediction related
	TML::PDMatrix<float, MAX_N_OBST, 1> u_d_i, u_opt_last_i, chi_d_i, chi_opt_last_i;

	TML::PDMatrix<float, 7, 1> xs_os_aug_k;
	
	TML::Vector2f v_os_k, v_A, v_B, L_AB, p_A, p_B;
	TML::Vector4f xs_i_p, xs_i_p_transformed;

	float t, chi_i, psi_A, psi_B, d_AB, u_opt_i, chi_opt_i; 

	Obstacle_Data_GPU_Friendly data;

	bool B_is_starboard, A_is_overtaken, B_is_overtaken;
	bool is_close, is_ahead, is_passed, is_head_on, is_crossing;

	int i_count;
	//==============================================
	
	__device__ void determine_situation_type(
		ST& st_A,
		ST& st_B,
		const TML::Vector2f &v_A,
		const float psi_A,
		const TML::Vector2f &v_B,
		const TML::Vector2f &L_AB,
		const float d_AB);

	__device__ void update_conditional_obstacle_data(const int i_caller, const int k);

	__device__ void predict_trajectories_jointly();

public: 
	__host__ CB_Cost_Functor() : 
		pars(nullptr), 
		fdata(nullptr), 
		obstacles(nullptr), 
		pobstacles(nullptr), 
		cpe(nullptr), 
		ownship(nullptr), 
		trajectory(nullptr), 
		obstacle_ship(nullptr), 
		obstacle_sbmpc(nullptr), 
		mpc_cost(nullptr) 
	{}

	__host__ CB_Cost_Functor(
		CB_Functor_Pars *pars, 
		CB_Functor_Data *fdata, 
		Cuda_Obstacle *obstacles, 
		Prediction_Obstacle *pobstacles,
		CPE_GPU *cpe,
		Ownship *ownship,
		TML::PDMatrix<float, 6, MAX_N_SAMPLES> *trajectory,
		Obstacle_Ship *obstacle_ship,
		Obstacle_SBMPC *obstacle_sbmpc,
		MPC_Cost<CB_Functor_Pars> *mpc_cost);

	__host__ __device__ ~CB_Cost_Functor() 
	{ 
		pars = nullptr; 
		fdata = nullptr; 
		obstacles = nullptr; 
		pobstacles = nullptr; 
		cpe = nullptr; 
		trajectory = nullptr; 
		obstacle_ship = nullptr; 
		obstacle_sbmpc = nullptr; 
		mpc_cost = nullptr;
	}
	
	__device__ float operator()(const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple);
	
};
	
#endif 