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


/****************************************************************************************
*  Name     : CB_Functor_Pars
*  Function : Struct containing subset of the PSB-MPC parameters for use in the GPU
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
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

/****************************************************************************************
*  Name     : CB_Functor_Data
*  Function : Struct containing data/information needed to evaluate the cost of one
*			  PSB-MPC control behaviour. Two versions for testing..
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
struct CB_Functor_Data
{
	CML::Pseudo_Dynamic_Matrix<double, 30, 1> maneuver_times;

	double u_d, chi_d;

	double u_m_last;
	double chi_m_last;

	CML::Pseudo_Dynamic_Matrix<double, 2, 100> waypoints;

	CML::Pseudo_Dynamic_Matrix<double, 6, 1000> trajectory;

	CML::Pseudo_Dynamic_Matrix<double, 4, 100> static_obstacles;

	int n_obst;

	// Number of prediction scenarios for each obstacle (max nr. for n_obst set to 50 here)
	CML::Pseudo_Dynamic_Matrix<int, 50, 1> n_ps;

	// Transitional indicator variables at the current time in addition to <obstacle ahead> (AH_0)
	// and <obstacle is passed> (IP_0) indicators
	CML::Pseudo_Dynamic_Matrix<bool, 50, 1> AH_0, S_TC_0, S_i_TC_0, O_TC_0, Q_TC_0, IP_0, H_TC_0, X_TC_0; 

	Cuda_Obstacle *obstacles;


	__host__ __device__ CB_Functor_Data(
		const PSBMPC &master, 
		const double u_d, 
		const double chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 4, -1> &static_obstacles,
		const Obstacle_Data &odata)
	{
		this->n_obst = odata.obstacles.size();

		this->u_d = u_d;
		this->chi_d = chi_d;

		this->u_m_last = master.u_m_last;
		this->chi_m_last = master.chi_m_last;

		// Assign Eigen objects to CML objects
		CML::assign_eigen_object(this->maneuver_times, master.maneuver_times);

		CML::assign_eigen_object(this->trajectory, master.trajectory);

		CML::assign_eigen_object(this->waypoints, waypoints);

		CML::assign_eigen_object(this->static_obstacles, static_obstacles);

		CML::assign_eigen_object(this->AH_0, odata.AH_0); 			CML::assign_eigen_object(this->S_TC_0, odata.S_TC_0);
		CML::assign_eigen_object(this->S_i_TC_0, odata.S_i_TC_0); 	CML::assign_eigen_object(this->O_TC_0, odata.O_TC_0);
		CML::assign_eigen_object(this->Q_TC_0, odata.Q_TC_0); 		CML::assign_eigen_object(this->IP_0, odata.IP_0);
		CML::assign_eigen_object(this->H_TC_0, odata.H_TC_0); 		CML::assign_eigen_object(this->X_TC_0, odata.X_TC_0);

		n_ps.resize(n_obst, 1);

		cudaMalloc(&obstacles, n_obst * sizeof(Cuda_Obstacle));

		Cuda_Obstacle* temp_obstacles = new Cuda_Obstacle[n_obst];
		for (int i = 0; i < this->n_obst; i++)
		{
			this->n_ps[i] = master.n_ps[i];

			temp_obstacles[i] = odata.obstacles[i];

			cudaMemcpy(&obstacles[i], &temp_obstacles[i], n_obst, cudaMemcpyHostToDevice);
		}
	}

	__host__ __device__ ~CB_Functor_Data() { cudaFree(obstacles); }
};
	
#endif 