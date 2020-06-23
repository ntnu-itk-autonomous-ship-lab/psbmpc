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

#include "ownship.h"
#include <iostream>
#include <variant>
#include <vector>
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

	mxArray *traj = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *ptraj = mxGetPr(traj);
	double *pwps = mxGetPr(wps);

	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, ERK1, LOS, T, dt);

	Eigen::Map<Eigen::MatrixXd> map_traj(ptraj, 6, n_samples);
	map_traj = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(pwps, 2, 7);
	map_wps = waypoints;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj);
	engPutVariable(ep, "WPs", wps);
	engEvalString(ep, "whos");
	engEvalString(ep, "testPlot");

	
	printf("%s", buffer);
	mxDestroyArray(traj);
	mxDestroyArray(wps);
	engClose(ep);

	return 0;
}