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
#include "utilities.h"
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
	
	//*****************************************************************************************************************
	// Obstacle setup
	//*****************************************************************************************************************
	double T = 80; double dt = 0.5;
	int n_samples = std::round(T / dt);

	Eigen::VectorXd xs_aug(9);
	xs_aug << 0, 0, 2, 0, 5, 5, 5, 5, 0;

	Eigen::MatrixXd P(4, 4);
	P << 100, 0, 0, 0,
	     0, 100, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	Eigen::VectorXd Pr_a(3);
	Pr_a << 1, 1, 1;
	Pr_a = Pr_a.normalized();

	double Pr_cc = 0.5;

	bool filter_on = true, colav_on = false;

	Obstacle *obstacle = new Obstacle(xs_aug, P, Pr_a, Pr_cc, filter_on, colav_on, T, dt);


	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *vtraj_i = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *pvtraj_i = mxGetPr(vtraj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);

	//*****************************************************************************************************************
	// Test obstacle functionality
	//*****************************************************************************************************************

	
	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************

	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	//map_traj_i = xs_p[0];

	Eigen::Map<Eigen::MatrixXd> map_vtraj_i(pvtraj_i, 2, n_samples);
	//map_vtraj_i = v_p[0];

	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);
	//map_P_traj_i = P_p[0];

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X_i", traj_i);
	engPutVariable(ep, "v", vtraj_i);
	engPutVariable(ep, "P_flat", P_traj_i);

	engEvalString(ep, "test_obstacle_plot");

	printf("%s", buffer);

	mxDestroyArray(traj_i);
	mxDestroyArray(vtraj_i);
	mxDestroyArray(P_traj_i);
	engClose(ep);

	return 0;
}