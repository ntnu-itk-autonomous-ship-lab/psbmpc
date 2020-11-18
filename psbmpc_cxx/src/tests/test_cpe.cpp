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


#include "cpe.h"
#include "utilities.h"
#include "mrou.h"
#include "ownship.h"
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include "Eigen/Dense"
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

	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 6, 0, 0;

	double T = 100; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;

	std::unique_ptr<Ownship> asv(new Ownship()); 

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	int n_samples = std::round(T / dt);
	std::cout << "n_samples = " << n_samples << std::endl;

	trajectory.resize(6, n_samples);
	trajectory.block<6, 1>(0, 0) << xs_os_0;

	waypoints.resize(2, 2); 
	//waypoints << 0, 200, 200, 0,    0, 300, 1000,
	//			 0, -50,  -200, -200,  0, 300, 0;
	waypoints << 0, 1000,
				 0, 0;

	//*****************************************************************************************************************
	// Obstacle prediction setup
	//*****************************************************************************************************************

	double sigma_x(0.8), sigma_xy(0),sigma_y(0.8), gamma_x(0.1), gamma_y(0.1);

	Eigen::Vector4d xs_0;
	xs_0 << 75, 75, -2, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
			0, 100, 0, 0,
			0, 0, 0.025, 0,
			0, 0, 0, 0.025;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_p;

	// Mean predicted velocity for the obstacle (MROU): n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> v_p;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step
	Eigen::MatrixXd P_p; 

	std::unique_ptr<MROU> mrou(new MROU(sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y));

	// n_ps = 1
	xs_p.resize(1); xs_p[0].resize(4, n_samples);
	xs_p[0].col(0) = xs_0;
	P_p.resize(16, n_samples);
	P_p.col(0) = flatten(P_0);

	v_p.resize(1); v_p[0].resize(2, n_samples);
	
	int n_ps = xs_p.size();

	Eigen::Vector2d v_0, v;
	v_0 << -2, 0;
	v_p[0].col(0) = v_0;

	Eigen::VectorXd turn_times(1);
	turn_times << 300;

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
			v(0) = -2.0; v(1) = 1.0;
			if (tt_count < turn_times.size() - 1) tt_count += 1;
		}
		if (k < n_samples - 1)	v_p[0].col(k + 1) = v;
	}

	//*****************************************************************************************************************
	// Collision Probability Estimator setup
	//*****************************************************************************************************************
	double d_safe = 50;

	Eigen::MatrixXd P_c_i_CE(n_ps, n_samples), P_c_i_MCSKF(n_ps, n_samples);
	Eigen::Matrix<double, 1, -1> P_c_i_temp(1, n_samples);
	std::unique_ptr<CPE> cpe(new CPE(CE, dt));

	//*****************************************************************************************************************
	// Prediction
	//*****************************************************************************************************************

	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, ERK1, LOS, T, dt);

	double t = 0;

	for (int ps = 0; ps < n_ps; ps++)
	{
		//=======================================================================================
		// MROU Prediction
		//=======================================================================================
		for (int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;
			if (k < n_samples - 1)
			{
				xs_p[ps].col(k + 1) = mrou->predict_state(xs_p[ps].col(k), v_p[ps].col(k), dt);
				P_p.col(k + 1) = flatten(mrou->predict_covariance(P_0, t));
			}
		}
		//=======================================================================================
		// CE Estimation
		//=======================================================================================

		cpe->set_method(CE);

		auto start = std::chrono::system_clock::now();

		cpe->estimate_over_trajectories(P_c_i_temp, trajectory, xs_p[ps], P_p, d_safe, dt);
		P_c_i_CE.row(ps) = P_c_i_temp;

		auto end = std::chrono::system_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		std::cout << "CE time usage one trajectory pair : " << elapsed.count() << " milliseconds" << std::endl;
		
		//=======================================================================================
		// MCSKF4D Estimation
		//=======================================================================================
		cpe->set_method(MCSKF4D);

		start = std::chrono::system_clock::now();
		
		cpe->estimate_over_trajectories(P_c_i_temp, trajectory, xs_p[ps], P_p, d_safe, dt);
		P_c_i_MCSKF.row(ps) = P_c_i_temp;

		end = std::chrono::system_clock::now();
    	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		std::cout << "MCSKF4D time usage one trajectory pair : " << elapsed.count() << " milliseconds" << std::endl;
	}

	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *p_traj_os = mxGetPr(traj_os_mx);
	double *p_wps = mxGetPr(wps_mx);

	mxArray *traj_i_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *v_traj_i_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_i_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_traj_i = mxGetPr(traj_i_mx);
	double *p_v_traj_i = mxGetPr(v_traj_i_mx);
	double *p_P_traj_i = mxGetPr(P_traj_i_mx);

	mxArray *Pcoll_CE = mxCreateDoubleMatrix(1, n_samples, mxREAL);
	mxArray *Pcoll_MCSKF = mxCreateDoubleMatrix(1, n_samples, mxREAL);

	double *p_CE = mxGetPr(Pcoll_CE);
	double *p_MCSKF = mxGetPr(Pcoll_MCSKF);

	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, 6, n_samples);
	map_traj_os = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps, 2, 2);
	map_wps = waypoints;

	Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i, 4, n_samples);
	map_traj_i = xs_p[0];

	Eigen::Map<Eigen::MatrixXd> map_v_traj_i(p_v_traj_i, 2, n_samples);
	map_v_traj_i = v_p[0];

	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);
	map_P_traj_i = P_p;

	Eigen::Map<Eigen::MatrixXd> map_Pcoll_CE(p_CE, 1, n_samples);
	map_Pcoll_CE = P_c_i_CE;

	Eigen::Map<Eigen::MatrixXd> map_Pcoll_MCSKF(p_MCSKF, 1, n_samples);
	map_Pcoll_MCSKF = P_c_i_MCSKF;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "WPs", wps_mx);

	engPutVariable(ep, "X_i", traj_i_mx);
	engPutVariable(ep, "v", v_traj_i_mx);
	engPutVariable(ep, "P_flat", P_traj_i_mx);

	engPutVariable(ep, "P_c_CE", Pcoll_CE);
	engPutVariable(ep, "P_c_MCSKF", Pcoll_MCSKF);
	engEvalString(ep, "test_cpe_plot");
	
	//save_matrix_to_file(P_c_i_CE[0]);
	//save_matrix_to_file(P_c_i_MCSKF[0]);

	printf("%s", buffer);
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_mx);

	mxDestroyArray(traj_i_mx);
	mxDestroyArray(v_traj_i_mx);
	mxDestroyArray(P_traj_i_mx);

	mxDestroyArray(Pcoll_CE);
	mxDestroyArray(Pcoll_MCSKF);
	engClose(ep);

	return 0;
}