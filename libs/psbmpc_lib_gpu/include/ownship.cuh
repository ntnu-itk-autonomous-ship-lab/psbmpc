/****************************************************************************************
*
*  File name : ownship.cuh
*
*  Function  : Header file for the ownship class. Modified and extended version of the
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


#ifndef _Ownship_CUH_
#define _Ownship_CUH_

#include <thrust/device_vector.h>
#include "Eigen/Dense"

enum Prediction_Method {
	Linear,													// Linear prediction
	ERK1, 													// Explicit Runge Kutta 1 = Eulers method
	ERK4 													// Explicit Runge Kutta of fourth order, not implemented.
};


enum Guidance_Method {
	LOS, 													// Line-of-sight		
	WPP,													// WP-Pursuit
	CH 														// Course Hold
};

class Ownship
{
private:

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
	double LOS_LD, LOS_K_i;

	// Counter variables to keep track of the active WP segment at the current 
	// time and predicted time
	int wp_c_0, wp_c_p;

	// Controller parameters
	double Kp_u;
	double Kp_psi;
	double Kd_psi;
	double Kp_r;
	
	double r_max; 

	double A, B, C, D, l, w;
	double x_offset, y_offset;

	// Calculates the offsets according to the position of the GPS receiver
	__host__ __device__ void calculate_position_offsets() { x_offset = A - B; y_offset = D - C; };

	__host__ __device__ void update_Cvv(const Eigen::Vector3d &nu);

	__host__ __device__ void update_Dvv(const Eigen::Vector3d &nu);

public:

	__host__ __device__ Ownship();

	__host__ __device__ void determine_active_waypoint_segment(const Eigen::Matrix<double, 2, -1> &waypoints, const Eigen::Matrix<double, 6, 1> &xs);

	__host__ __device__ void update_guidance_references(
		double &u_d, 
		double &chi_d, 
		const Eigen::Matrix<double, 2, -1> &waypoints, 
		const Eigen::Matrix<double, 6, 1> &xs,
		const double dt,
		const Guidance_Method guidance_method);

	__host__ __device__ void update_ctrl_input(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);

	__host__ __device__ Eigen::Matrix<double, 6, 1> predict(
		const Eigen::Matrix<double, 6, 1> &xs_old, 
		const double dt, 
		const Prediction_Method prediction_method);

	__host__ __device__ void predict_trajectory(
		Eigen::Matrix<double, 6, -1> &trajectory,
		const Eigen::VectorXd &offset_sequence,
		const Eigen::VectorXd &maneuver_times,
		const double u_d,
		const double chi_d,
		const Eigen::Matrix<double, 2, -1> &waypoints,
		const Prediction_Method prediction_method,
		const Guidance_Method guidance_method,
		const double T,
		const double dt
	);

	__host__ __device__ double get_length() const { return l; };

	__host__ __device__ double get_width() const { return w; };

};

#endif
