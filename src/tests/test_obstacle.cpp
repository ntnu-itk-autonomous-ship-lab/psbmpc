/****************************************************************************************
*
*  File name : test_ownship.cpp
*
*  Function  : Test file for the Ownship class for PSB-MPC, using Matlab for 
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

#include "obstacle.h"
#include "ownship.h"
#include "utilities.h"
#include <iostream>
#include <vector>
#include <string>
#include "Eigen/StdVector"
#include "Eigen/Core"
#include "engine.h"

#define BUFSIZE 1000000

int main(){
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

	double T = 80; double dt = 0.5;
	int n_samples = std::round(T / dt);
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
	trajectory.block<6, 1>(0, 0) << xs_os_0;
	std::cout << "traj init = [" << trajectory(0, 0) << ", " << trajectory(1, 0) << ", " << trajectory(2, 0) << ", " \
	<< trajectory(3, 0) << ", " << trajectory(4, 0) << ", " << trajectory(5, 0) << "]" << std::endl;

	waypoints.resize(2, 2); 
	//waypoints << 0, 200, 200, 0,    0, 300, 1000,
	//			 0, -50,  -200, -200,  0, 300, 0;
	waypoints << 0, 1000,
				 0, 0;

	mxArray *traj_os = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *ptraj_os = mxGetPr(traj_os);
	double *pwps = mxGetPr(wps);
	
	//*****************************************************************************************************************
	// Obstacle setup
	//*****************************************************************************************************************

	Eigen::VectorXd xs_aug(9);
	xs_aug << 100, 0, -2, 0, 5, 5, 5, 5, 0;

	Eigen::MatrixXd P(4, 4);
	P << 100, 0, 0, 0,
	     0, 100, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	Eigen::VectorXd Pr_a(3);
	Pr_a << 1, 1, 1;
	Pr_a = Pr_a.normalized();

	double Pr_cc = 0.9;

	bool filter_on = true, colav_on = false;

	Obstacle *obstacle = new Obstacle(xs_aug, P, Pr_a, Pr_cc, filter_on, colav_on, T, dt);

	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);

	//*****************************************************************************************************************
	// Test obstacle functionality
	//*****************************************************************************************************************
	double d_close = 500, d_safe = 300; 
	double phi_AH = 68.5 * DEG2RAD, phi_OT = 68.5 * DEG2RAD, phi_HO = 22.5 * DEG2RAD, phi_CR = 68.5 * DEG2RAD;	
	
	int n_ps = 5; 	// KCC, two alternative maneuvers to starboard and port, only one course change of 45 deg
	std::vector<Intention> ps_ordering;
	Eigen::VectorXd ps_course_changes, ps_weights, ps_maneuver_times;
	ps_ordering.resize(n_ps); ps_course_changes.resize(n_ps);
	ps_weights.resize(n_ps); ps_maneuver_times.resize(n_ps);
	
	double Pr_CC_i_test = obstacle->get_a_priori_CC_probability();
	std::cout << "Pr_CC = " << Pr_CC_i_test << std::endl;

	for (int ps = 0; ps < n_ps; ps++)
	{
		if (ps == 0) { ps_ordering[ps] = KCC; }
		else if (ps > 0 && ps < (n_ps - 1) / 2 + 1) { ps_ordering[ps] = SM; }
		else { ps_ordering[ps] = PM; }
	}
	// Lets say a head-on situation, assign weights accordingly:
	ps_maneuver_times << 0, 0, 50, 0, 50;
	ps_course_changes << 0, 45 * DEG2RAD, 45 * DEG2RAD, - 45 * DEG2RAD, - 45 * DEG2RAD;
	ps_weights << (1 - Pr_CC_i_test), Pr_CC_i_test, Pr_CC_i_test, (1 - Pr_CC_i_test), (1 - Pr_CC_i_test);

	obstacle->initialize_independent_prediction(ps_ordering, ps_course_changes, ps_weights, ps_maneuver_times);

	obstacle->predict_independent_trajectories(T, dt, trajectory.col(0), phi_AH, phi_CR, phi_HO, phi_OT, d_close, d_safe);

	std::vector<Eigen::MatrixXd> xs_p = obstacle->get_independent_trajectories();

	std::vector<Eigen::MatrixXd> P_p = obstacle->get_independent_trajectory_covariances();

	std::vector<bool> mu = obstacle->get_COLREGS_violation_indicator();
	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, n_samples);
	map_traj_os = trajectory;
	
	Eigen::Map<Eigen::MatrixXd> map_wps(pwps, 2, 2);
	map_wps = waypoints;
	
	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);

	engPutVariable(ep, "X", traj_os);
	engPutVariable(ep, "WPs", wps);

	engEvalString(ep, "test_ownship_plot");

	mxArray *ps_count = mxCreateDoubleScalar(1);
	double *ps_ptr = mxGetPr(ps_count);

	for (int ps = 0; ps < n_ps; ps++)
	{
		map_traj_i = xs_p[ps];
		engPutVariable(ep, "X_i", traj_i);

		map_P_traj_i = P_p[ps];
		engPutVariable(ep, "P_flat_i", P_traj_i);

		engPutVariable(ep, "ps_counter", ps_count);
		*ps_ptr++;
		
		engEvalString(ep, "test_obstacle_plot");
	}
	
	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	printf("%s", buffer);

	mxDestroyArray(traj_os);
	mxDestroyArray(wps);

	mxDestroyArray(traj_i);
	mxDestroyArray(P_traj_i);
	engClose(ep);

	return 0;
}