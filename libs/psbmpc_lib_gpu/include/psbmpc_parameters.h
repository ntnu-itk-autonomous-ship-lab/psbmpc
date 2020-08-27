/****************************************************************************************
*
*  File name : psbmpc.h
*
*  Function  : Header file for the PSB-MPC parameter struct.
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

#ifndef _PSBMPC_PARAMETERS_H_
#define _PSBMPC_PARAMETERS_H_

#include "psbmpc_index.h"
#include "ownship.cuh"
#include "cpe.cuh"
#include "Eigen/Dense"
#include <vector>

class PSBMPC;

enum Par_Type 
{
	BPAR,					// Boolean type parameter
	IPAR,					// Integer type parameter
	DPAR,					// Double type parameter
	OPAR,					// Offset/control behaviour related parameter
	EVPAR,					// Eigen::Vector parameter
	CPEM,					// CPE_Method parameter
	PREDM,					// Prediction_Method parameter
	GUIDM					// Guidance_Method parameter
};

struct PSBMPC_Parameters
{
private:

	friend class PSBMPC;

	int n_cbs, n_M;

	std::vector<Eigen::VectorXd> u_offsets;
	std::vector<Eigen::VectorXd> chi_offsets;

	Eigen::MatrixXd control_behaviours;

	Eigen::VectorXd obstacle_course_changes;

	Eigen::VectorXd dpar_low, dpar_high;
	Eigen::VectorXd ipar_low, ipar_high;

	CPE_Method cpe_method;

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
	
	bool obstacle_filter_on;
	bool obstacle_colav_on;

	double T_lost_limit, T_tracked_limit;

	void initialize_par_limits();

	void initialize_pars();

	void map_offset_sequences();

	void reset_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

	void increment_control_behaviour(Eigen::VectorXd &offset_sequence_counter, Eigen::VectorXd &offset_sequence);

public:

	PSBMPC_Parameters() { initialize_pars(); initialize_par_limits(); }

	void set_par(const int index, const bool value);

	void set_par(const int index, const int value);

	void set_par(const int index, const double value);

	void set_par(const int index, const std::vector<Eigen::VectorXd> &value);

	void set_par(const int index, const Eigen::VectorXd &value);

	void set_cpe_method(CPE_Method cpe_method) 						{ if (cpe_method >= CE && cpe_method <= MCSKF4D) this->cpe_method = cpe_method; };

	void set_prediction_method(Prediction_Method prediction_method) { if (prediction_method >= Linear && prediction_method <= ERK4) this->prediction_method = prediction_method; };

	void set_guidance_method(Guidance_Method guidance_method) 		{ if (guidance_method >= LOS && guidance_method <= CH) this->guidance_method = guidance_method; };

	bool get_bpar(const int index) const;  

	int get_ipar(const int index) const;
	
	double get_dpar(const int index) const;

	std::vector<Eigen::VectorXd> get_opar(const int index) const;

	Eigen::VectorXd get_evpar(const int index) const;

	CPE_Method get_cpe_method() const { return cpe_method; }; 

	Prediction_Method get_prediction_method() const { return prediction_method; };

	Guidance_Method get_guidance_method() const { return guidance_method; };

};

#endif 