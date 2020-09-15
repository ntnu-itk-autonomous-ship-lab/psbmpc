/****************************************************************************************
*
*  File name : cb_cost_functor_structures.cuh
*
*  Function  : Header file for the data/parameter structures used in the control behaviour 
*			   cost functor. Used in the thrust framework for GPU calculations.
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

#ifndef _CB_COST_FUNCTOR_STRUCTURES_H_
#define _CB_COST_FUNCTOR_STRUCTURES_H_

#include <thrust/device_vector.h>
#include "psbmpc.cuh"
#include "psbmpc_parameters.h"
#include "cml.cuh"
#include "ownship.cuh"
#include "cuda_obstacle.cuh"
#include "cpe.cuh"

struct CB_Functor_Pars
{
	int n_M;

	CPE_Method cpe_method;

	Prediction_Method prediction_method;

	Guidance_Method guidance_method;

	double T, T_static, dt, p_step;
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
	
	bool obstacle_colav_on;

	__host__ __device__ CB_Functor_Pars() {}

	__host__ __device__ CB_Functor_Pars(const PSBMPC_Parameters &pars)
	{
		this->n_M = pars.n_M;

		this->cpe_method = pars.cpe_method;

		this->prediction_method = pars.prediction_method;

		this->guidance_method = pars.guidance_method;

		this->T = pars.T; this->T_static = pars.T_static; this->dt = pars.dt; this->p_step = pars.p_step; 

		this->d_safe = pars.d_safe; this->d_close = pars.d_close; this->d_init = pars.d_init;

		this->K_coll = pars.K_coll;

		this->phi_AH = pars.phi_AH; this->phi_OT = pars.phi_OT; this->phi_HO = pars.phi_HO; this->phi_CR = pars.phi_CR;

		this->kappa = pars.kappa; this->kappa_TC = pars.kappa_TC;

		this->K_u = pars.K_u; this->K_du = pars.K_du;

		this->K_chi_strb = pars.K_chi_strb; this->K_dchi_strb = pars.K_dchi_strb;
		this->K_chi_port = pars.K_chi_port; this->K_dchi_port = pars.K_dchi_port;

		this->K_sgn = pars.K_sgn; this->T_sgn = pars.T_sgn;

		this->G = pars.G;

		this->q = pars.q; this->p = pars.p;

		this->obstacle_colav_on = pars.obstacle_colav_on;
	}
};

class CB_Functor_Data
{
public:

	CML::MatrixXd maneuver_times;
	CML::MatrixXd control_behaviours;

	double u_d, chi_d;

	double u_m_last;
	double chi_m_last;

	CML::MatrixXd waypoints;

	CML::MatrixXd trajectory;

	CML::MatrixXd static_obstacles;

	int n_obst;

	CML::MatrixXi n_ps;

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	CML::MatrixXb AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0; 

	Cuda_Obstacle *obstacles;

	__host__ void assign_master_data(
		const PSBMPC &master, 
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		const Obstacle_Data &odata);

	__host__ __device__ CB_Functor_Data() : obstacles(nullptr) {}

	__host__ __device__ ~CB_Functor_Data() { delete[] obstacles; }
};
	
#endif 