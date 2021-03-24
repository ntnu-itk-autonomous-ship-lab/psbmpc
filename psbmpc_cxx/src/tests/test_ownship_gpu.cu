/****************************************************************************************
*
*  File name : test_ownship.cu
*
*  Function  : Test file for the Ownship class for GPU PSB-MPC, using Matlab for 
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

#if OWNSHIP_TYPE == 0
	#include "gpu/kinematic_ship_models_gpu.cuh"
#else 
	#include "gpu/kinetic_ship_models_gpu.cuh"
#endif
#include <iostream>
#include <memory>
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
	Eigen::Matrix<double, 6, 1> xs;
	xs << 0, 0, 0, 6, 0, 0;

	double T = 200; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;
	
	std::unique_ptr<PSBMPC_LIB::GPU::Ownship> asv(new PSBMPC_LIB::GPU::Ownship()); 

	int n_samples = std::round(T / dt);
	Eigen::MatrixXd trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	#if OWNSHIP_TYPE == 0
		trajectory.resize(4, n_samples);
		trajectory.col(0) = xs.block<4, 1>(0, 0);
	#else
		trajectory.resize(6, n_samples);
		trajectory.col(0) = xs;
	#endif

	waypoints.resize(2, 7); 
	waypoints << 0, 200, 200, 0,    0, 300, 1000,
				 0, -50,  -200, -200,  0, 300, 0;

	//*****************************************************************************************************************
	// Prediction
	//*****************************************************************************************************************
	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T, dt);

	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	mxArray *traj_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
	mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *p_traj_mx = mxGetPr(traj_mx);
	double *p_wps_mx = mxGetPr(wps_mx);

	Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_mx, trajectory.rows(), n_samples);
	map_traj = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_mx, 2, 7);
	map_wps = waypoints;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_mx);
	engPutVariable(ep, "WPs", wps_mx);

	engEvalString(ep, "test_ownship_plot");
	printf("%s", buffer);
	mxDestroyArray(traj_mx);
	mxDestroyArray(wps_mx);
	engClose(ep);

	return 0;
}