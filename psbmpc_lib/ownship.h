/****************************************************************************************
*
*  File name : ownship.h
*
*  Function  : header file for the ownship class. Modified and extended version of the
*			   "Ship_Model" class created for SBMPC by Inger Berge Hagen and Giorgio D. 
*			   Kwame Minde Kufoalor through the Autosea project. Facilitates Guidance,
*			   Navigation and Control (GNC) of a surface vessel 
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


#ifndef _Ownship_H_
#define _Ownship_H_

#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include <string>

class Ownship
{
	private:

	Eigen::Matrix<double, 6, -1> trajectory;

	Eigen::Vector3d tau;
	Eigen::Matrix3d M_inv;
	Eigen::Vector3d Cvv;
	Eigen::Vector3d Dvv;

	// Model Parameters
	double l_r;
	double m; 	// [kg]
	double I_z; // [kg/m2]

	// Added mass terms
	double X_udot;
	double Y_vdot, Y_rdot;
	double N_vdot, N_rdot;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	double X_u;
	double Y_v, Y_r;
	double N_v, N_r;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	double X_uu;
	double Y_vv;
	double N_rr;
	double X_uuu;
	double Y_vvv;
	double N_rrr;

	//Force limits
	double Fx_min;
	double Fx_max;
	double Fy_min;
	double Fy_max; 

	// Guidance parameters
	double e_int, e_int_max; 
	double R_a;
	double LOS_LA, LOS_K_i;

	int wp_counter;

	// Controller parameters
	double Kp_u;
	double Kp_psi;
	double Kd_psi;
	double Kp_r;
	
	double r_max; 
	double psi_rf; // required fraction of psi_d to be achieved before adapting psi_d.

	double A, B, C, D, l, w;
	double x_offset, y_offset;

	// Calculates the offsets according to the position of the GPS receiver
	void calculate_position_offsets();

	inline void Cvv(const Eigen::Vector3d v);

	inline void Dvv(const Eigen::Vector3d v);

	void update_guidance_references(
		double &u_d, 
		double &psi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 6, 1> &xs,
		const int k,
		const double dt,
		const Guidance_Method guidance_method);

	void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

	public:

	Ownship();

	Eigen::VectorXd predict(const Eigen::Matrix<double, 6, 1> &xs_old, const double dt, const Prediction_Method prediction_method);

	void resize_trajectory(const int n_samples);

	void predict_trajectory(
		const Eigen::VectorXd offset_sequence, 
		const double u_d, 
		const double psi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const double T,
		const double dt, 
		const Guidance_Method guidance_method);

	// Must assess how to parameterize this trajectory, should it be public or private? 
	Eigen::Matrix<double, 6, -1> get_trajectory() const { return trajectory; };

	double get_length() const { return l};

	double get_width() const { return w};

};

#endif
