/*
 *    This file is part of SB-MPC Library.
 *
 *    SB-MPC -- Scenario-Based MPC for Maritime Collision Avoidance.
 *    Copyright (C) 2016-2019 Inger Berge Hagen, Giorgio D. Kwame Minde Kufoalor, 
 *    NTNU Trondheim.
 *    Developed within the Autosea Project (Sensor fusion and collision avoidance for 
 *    autonomous surface vehicles) under the supervision of Tor Arne Johansen. 
 *    All rights reserved.
 *
 *    SB-MPC Library is software used according to the conditions of the Autosea Consortium.
 *    <https://www.ntnu.edu/autosea>
 */


/**
 *    \file   ship_model.h
 *    \brief  Declares the shipModel class.
 *    \author Inger Berge Hagen, Giorgio D. K. M. Kufoalor
 */


#ifndef SHIP_MODEL_H_
#define SHIP_MODEL_H_

#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include <string>

class shipModel
{
	public:

	/// Constructor
	shipModel(double T, double dt);

	/// Destructor
	~shipModel();

	//void eulersMethod(const Eigen::Matrix<double,6,1>& state, double u_d, double psi_d, double r_limit, const Eigen::Matrix<double,2,1>& next_wp);
	
	void eulersMethod(const Eigen::Matrix<double,6,1>& state, double u_d, double psi_d, const Eigen::Matrix<double,-1,2>&waypoints, double Chi_ca, int course_change_point, int guidance_strategy, double R, double de, double Ki);

	void linearPrediction(const Eigen::Matrix<double,6,1>& state, double u_d, double psi_d, const Eigen::Matrix<double,-1,2>&waypoints, double Chi_ca, int course_change_point, int guidance_strategy, double R, double de, double Ki);

	Eigen::VectorXd getX();
	Eigen::VectorXd getY();
	Eigen::VectorXd getPsi();
	Eigen::VectorXd getU();
	Eigen::VectorXd getV();
	Eigen::VectorXd getR();

	double getA();
	double getB();
	double getC();
	double getD();

	double getL();
	double getW();

	double getT();
	double getDT();
	double getNsamp();

	void setT(double T);
	void setDT(double DT);
	void setNsamp(int n_samp);

	void setA(double A);
	void setB(double B);
	void setC(double C);
	void setD(double D);

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	Eigen::VectorXd psi;
	Eigen::VectorXd u;
	Eigen::VectorXd v;
	Eigen::VectorXd r;

	double A_, B_, C_, D_, l, w;
	double os_x, os_y;


	private:

	// Calculates the offsets according to the position of the GPS receiver
	void calculate_position_offsets();

	// Assures that the numerical difference is at most PI
	double normalize_angle_diff(double angle, double angle_ref);

	// Assures that angle is between [-PI, PI)
	double normalize_angle(double angle);

	void updateCtrlInput(double u_d, double psi_d, int i);
	
	// Computes angle difference
	inline double angle_diff(double a,double b);

	Eigen::Vector3d tau;
	Eigen::Matrix3d Minv;
	Eigen::Vector3d Cvv;
	Eigen::Vector3d Dvv;

	// Model Parameters
	double rudder_d;
	double M; 	// [kg]
	double I_z; // [kg/m2]

	// Added mass terms
	double X_udot;
	double Y_vdot;
	double Y_rdot;
	double N_vdot;
	double N_rdot;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	double X_u;
	double Y_v;
	double Y_r;
	double N_v;
	double N_r;

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

	// Simulation parameters
	double DT_;
	double T_;
	//const int n_samp_;
	int n_samp_; // possibility to set from sb_mpc

	// Controller parameters
	double Kp_u;
	double Kp_psi;
	double Kd_psi;
	double Kp_r;
	
	
	double r_max; // [deg/s] max yaw rate
	double psi_response_factor; // required fraction of psi_d to be achieved before adapting psi_d.

};

#endif /* SHIP_MODEL_H_ */
