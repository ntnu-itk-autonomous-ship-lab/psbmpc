/****************************************************************************************
*
*  File name : test_obstacle_ship.cu
*
*  Function  : Test file for the Obstacle_Ship class for PSB-MPC, using Matlab for 
*			   visualization. Not complete yet. 
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


#include "gpu/obstacle_ship_gpu.cuh"
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
	Eigen::Matrix<double, 4, 1> xs;
	xs << 0, 0, 0, 6;

	double T = 200; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;
	
	std::unique_ptr<PSBMPC_LIB::GPU::Obstacle_Ship> obstacle_ship(new PSBMPC_LIB::GPU::Obstacle_Ship()); 

	Eigen::Matrix<double, 4, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	int n_samples = std::round(T / dt);
	trajectory.resize(4, n_samples);
	trajectory.block<4, 1>(0, 0) << xs;

	int n_wps = 7;
	waypoints.resize(2, n_wps); 
	waypoints << 0, 200, 200, 0,    0, 300, 1000,
				 0, -50,  -200, -200,  0, 300, 0;

	waypoints << 0, 200, 200, 0,    0, 300, 1000,
				 0, 50,  200, 200,  0, -300, 0;

	//*****************************************************************************************************************
	// Prediction
	//*****************************************************************************************************************
	obstacle_ship->determine_active_waypoint_segment(waypoints, trajectory.col(0));

	obstacle_ship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T, dt);

	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);
	mxArray *T_sim_mx = mxCreateDoubleScalar(T);
	mxArray *dt_sim_mx = mxCreateDoubleScalar(dt);

	double *p_traj_mx = mxGetPr(traj_mx);
	double *p_wps_mx = mxGetPr(wps_mx);

	Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_mx, 4, n_samples);
	map_traj = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_mx, 2, n_wps);
	map_wps = waypoints;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_mx);
	engPutVariable(ep, "WPs", wps_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);

	engEvalString(ep, "test_obstacle_ship_plot");
	
	printf("%s", buffer);
	mxDestroyArray(traj_mx);
	mxDestroyArray(wps_mx);
	engClose(ep);

	return 0;
}