/****************************************************************************************
*
*  File name : sbmpc_parameters.h
*
*  Function  : Header file for the SB-MPC parameter class.
*
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

#ifndef _SBMPC_PARAMETERS_H_
#define _SBMPC_PARAMETERS_H_

#include "psbmpc_index.h"
#include "psbmpc_parameters.h"
#include "Eigen/Dense"
#include <vector>

class SBMPC;
template <typename Parameters> class MPC_Cost;
class Obstacle_SBMPC;
class Obstacle_Manager;

class SBMPC_Parameters
{
private:

	friend class SBMPC;
	friend class MPC_Cost<SBMPC_Parameters>;
	friend class Obstacle_SBMPC;
	friend class Obstacle_Manager;

	int n_cbs, n_M;

	std::vector<Eigen::VectorXd> u_offsets;
	std::vector<Eigen::VectorXd> chi_offsets;

	Eigen::VectorXd dpar_low, dpar_high;
	Eigen::VectorXd ipar_low, ipar_high;

	Prediction_Method prediction_method;

	Guidance_Method guidance_method;

	double T, T_static, dt, p_step;
	double t_ts;
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

	void initialize_par_limits();

	void initialize_pars(const bool is_obstacle_sbmpc);

public:

	SBMPC_Parameters(const bool is_obstacle_sbmpc) { initialize_pars(is_obstacle_sbmpc); initialize_par_limits(); }

	SBMPC_Parameters(std::string tuning_file); // Not implemented yet

	void set_par(const int index, const int value);

	void set_par(const int index, const double value);

	void set_par(const int index, const std::vector<Eigen::VectorXd> &value);

	inline void set_prediction_method(const Prediction_Method prediction_method)  	{ if (prediction_method >= Linear && prediction_method <= ERK4) this->prediction_method = prediction_method; }

	inline void set_guidance_method(const Guidance_Method guidance_method) 		 	{ if (guidance_method >= LOS && guidance_method <= CH) this->guidance_method = guidance_method; }

	int get_ipar(const int index) const;
	
	double get_dpar(const int index) const;

	std::vector<Eigen::VectorXd> get_opar(const int index) const;

	inline Prediction_Method get_prediction_method() const { return prediction_method; };

	inline Guidance_Method get_guidance_method() const { return guidance_method; };

};

#endif 