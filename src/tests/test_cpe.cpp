/****************************************************************************************
*
*  File name : test_cpe.cpp
*
*  Function  : Test file for the CPE class for PSB-MPC, using Matlab for 
*			   visualization 
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


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utilities.h"
#include "cpe.h"
#include "mrou.h"
#include "ownship.h"
#include <iostream>
#include <variant>
#include <vector>
#include "Eigen/StdVector"
#include "Eigen/Core"
#include "engine.h"

#define BUFSIZE 1000000

int main(){
	// Matlab engine setup
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1];
	
	//*****************************************************************************************************************
	// Own-ship prediction setup
	//*****************************************************************************************************************

	Eigen::Matrix<double, 6, 1> xs;
	xs << 0, 0, 0, 6, 0, 0;

	double T = 300; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	std::vector<Eigen::VectorXd> chi_offsets, value;
	chi_offsets.resize(3);
	chi_offsets[0].resize(13); chi_offsets[1].resize(7); chi_offsets[2].resize(5);
	Eigen::VectorXd v1, v2, v3;
	v1.resize(13); v2.resize(7); v3.resize(5);
	v1 << -90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90;
	v2 << -90, -75, -45, 0, 45, 75, 90;
	v3 << -90, -45, 0, 45, 90;
	chi_offsets[0] << v1;
	chi_offsets[1] << v2;
	chi_offsets[2] << v3;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;

	
	for (std::vector<Eigen::VectorXd>::iterator it = chi_offsets.begin(); it != chi_offsets.end(); it++){
		std::cout << "chi_offsets(j) = " << *it << " | ";
	}
	std::cout << std::endl;

	Ownship* asv = new Ownship(); 

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	int n_samples = std::round(T / dt);
	trajectory.resize(6, n_samples);
	trajectory.block<6, 1>(0, 0) << xs;
	std::cout << "traj init = [" << trajectory(0, 0) << ", " << trajectory(1, 0) << ", " << trajectory(2, 0) << ", " \
	<< trajectory(3, 0) << ", " << trajectory(4, 0) << ", " << trajectory(5, 0) << "]" << std::endl;

	waypoints.resize(2, 7); 
	waypoints << 0, 200, 200, 0,    0, 300, 1000,
				 0, -50,  -200, -200,  0, 300, 0;

	mxArray *traj_os = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *ptraj_os = mxGetPr(traj_os);
	double *pwps = mxGetPr(wps);

	//*****************************************************************************************************************
	// Obstacle prediction setup
	//*****************************************************************************************************************

	double sigma_x(0.8), sigma_xy(0),sigma_y(0.8), gamma_x(0.1), gamma_y(0.1);

	Eigen::Vector4d xs_0;
	xs_0 << 0, 0, 2, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
			0, 100, 0, 0,
			0, 0, 0.025, 0,
			0, 0, 0, 0.025;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> v_p;

	// Predicted covariance for each prediction scenario: n_ps x n*n x n_samples, i.e. the covariance is flattened for each time step
	std::vector<Eigen::MatrixXd> P_p; 

	MROU *mrou = new MROU(sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y);

	// n_ps = 1
	xs_p.resize(1); xs_p[0].resize(4, n_samples);
	xs_p[0].col(0) = xs_0;
	P_p.resize(1); P_p[0].resize(16, n_samples);
	P_p[0].col(0) = Utilities::flatten(P_0);

	v_p.resize(1); v_p[0].resize(2, n_samples);
	
	int n_ps = xs_p.size();

	Eigen::Vector2d v_0, v;
	v_0 << 4, 0;
	v_p[0].col(0) = v_0;

	Eigen::VectorXd turn_times(1);
	turn_times << 100;

	int tt_count = 0;
	double chi = 0; 
	v = v_0;
	for(int k = 0; k < n_samples; k++)
	{
		if (k == turn_times(tt_count))
		{
			chi = atan2(v(1), v(0));
			v(0) = v.norm() * cos(chi + 30 * M_PI / 180.0);
			v(1) = v.norm() * sin(chi + 30 * M_PI / 180.0);
			if (tt_count < turn_times.size() - 1) tt_count += 1;
		}
		if (k < n_samples - 1)	v_p[0].col(k + 1) = v;
	}

	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *vtraj_i = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *pvtraj_i = mxGetPr(vtraj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);

	//*****************************************************************************************************************
	// Collision Probability Estimator setup
	//*****************************************************************************************************************
	mxArray *Pcoll_CE = mxCreateDoubleMatrix(1, n_samples, mxREAL);
	mxArray *Pcoll_MCSKF = mxCreateDoubleMatrix(1, n_samples, mxREAL);

	double *p_CE = mxGetPr(Pcoll_CE);
	double *p_MCSKF = mxGetPr(Pcoll_MCSKF);

	double d_safe = 50;

	std::vector<Eigen::VectorXd> P_c_i_CE, P_c_i_MCSKF;
	P_c_i_CE.resize(1); P_c_i_MCSKF.resize(1);
	P_c_i_CE[0].resize(n_samples);
	P_c_i_MCSKF[0].resize(n_samples);

	CPE *cpe = new CPE(CE, 1000, 100, d_safe, dt);

	//*****************************************************************************************************************
	// Prediction
	//*****************************************************************************************************************

	cpe->set_number_of_obstacles(1);
	cpe->initialize(trajectory.col(0), xs_p[0].col(0), Utilities::reshape(P_p[0].col(0), 4, 4), 0);

	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, ERK1, LOS, T, dt);

	double t = 0;
	Eigen::Vector4d xs = xs_0;
	Eigen::Matrix4d P = P_0;
	for (int ps = 0; ps < n_ps; ps++)
	{
		cpe->set_method(CE);
		for (int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;
			xs = mrou->predict_state(xs, v_p[ps].col(k), dt);
			P = mrou->predict_covariance(P_0, t);

			if (k < n_samples - 1)
			{
				xs_p[ps].col(k + 1) = xs;
				P_p[ps].col(k + 1) = Utilities::flatten(P);
			}

			// Collision probability estimation using CE
			P_c_i_CE[ps](k) = cpe->estimate(trajectory.col(k), xs_p[ps].col(k), Utilities::reshape(P_p[ps].col(k), 4, 4), 0);
		}
		
		cpe->set_method(MCSKF4D);
		cpe->initialize(trajectory.col(0), xs_p[ps].col(0), Utilities::reshape(P_p[ps].col(0), 4, 4), 0);
		for (int k = 0; k < n_samples; k++)
		{
			// Collision probability estimation using MCSKF4D
			P_c_i_CE[ps](k) = cpe->estimate(trajectory.col(k), xs_p[ps].col(k), Utilities::reshape(P_p[ps].col(k), 4, 4), 0);
		}
	}

	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************

	Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, n_samples);
	map_traj_os = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(pwps, 2, 7);
	map_wps = waypoints;

	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	map_traj_i = xs_p[0];

	Eigen::Map<Eigen::MatrixXd> map_vtraj_i(pvtraj_i, 2, n_samples);
	map_vtraj_i = v_p[0];

	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);
	map_P_traj_i = P_p[0];

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_os);
	engPutVariable(ep, "WPs", wps);

	engPutVariable(ep, "X_i", traj_i);
	engPutVariable(ep, "v", vtraj_i);
	engPutVariable(ep, "P_flat", P_traj_i);
	engEvalString(ep, "test_cpe_plot");

	printf("%s", buffer);
	mxDestroyArray(traj_os);
	mxDestroyArray(wps);

	mxDestroyArray(traj_i);
	mxDestroyArray(vtraj_i);
	mxDestroyArray(P_traj_i);
	engClose(ep);

	return 0;
}