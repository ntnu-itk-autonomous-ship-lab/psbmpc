/****************************************************************************************
*
*  File name : test_psbmpc_cost_class.cpp
*
*  Function  : Test file for the Probabilistic Scenario-based Model Predictive Control 
*			   cost function class.
*			   
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengsedal NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengsedal
*
*  Modified  : 
*
*****************************************************************************************/


#include "cpu/psbmpc_cpu.hpp"
#include "gpu/mpc_cost_gpu.cuh"
#include "cpu/utilities_cpu.hpp"
#include "grounding_hazard_manager.hpp"
#include "engine.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <string>


#define BUFSIZE 1000000

//*****************************************************************************************************************
// Main program:
//*****************************************************************************************************************
int main(){
	// Matlab engine setup
 	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
    
	char buffer[BUFSIZE+1]; 

//*****************************************************************************************************************
// Simulation setup
//*****************************************************************************************************************
	double T_sim = 500; double dt = 0.5;
	int N = std::round(T_sim / dt);

//*****************************************************************************************************************
// Static Obstacles Setup
//*****************************************************************************************************************
	char buffer1[256];
	char *val = getcwd(buffer1, sizeof(buffer1));
	if (val) {
		std::cout << buffer1 << std::endl;
	}
	// Input the path to the land data
    std::string filename = "src/tests/grounding_hazard_data/charts/land/land.shp";
    
   	PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager(filename);
	std::vector<polygon_2D> polygons = grounding_hazard_manager.get_polygons();

    //Make matlab polygons type friendly array:
    Eigen::Matrix<double, -1, 2> polygon_matrix;
    int n_static_obst = 0;
    BOOST_FOREACH(polygon_2D const &poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			n_static_obst += 1;
		}
		n_static_obst += 1;
    }
    polygon_matrix.resize(n_static_obst, 2); 

    /*format polygon_matrix array for matlab plotting*/
    int pcount = 0; 
    BOOST_FOREACH(polygon_2D const& poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			polygon_matrix(pcount, 0) = boost::geometry::get<0>(*it);
			polygon_matrix(pcount, 1) = boost::geometry::get<1>(*it);
			
			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		polygon_matrix(pcount, 0) = -1;
		polygon_matrix(pcount, 1) = -1;
		pcount += 1;
    }
    
    mxArray *polygon_matrix_mx = mxCreateDoubleMatrix(n_static_obst, 2, mxREAL);
    double *p_polygon_matrix = mxGetPr(polygon_matrix_mx);
    Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, n_static_obst, 2);
	map_polygon_matrix = polygon_matrix;

	engPutVariable(ep, "P", polygon_matrix_mx);
//*****************************************************************************************************************
// Own-ship sim setup
//*****************************************************************************************************************

	/*coordinates are given in wgs-84 use https://finnposisjon.test.geonorge.no/ */
	Eigen::Matrix<double, 6, 1> xs_os_0;
	// xs_os_0 << 7042320, 269475, 180 * DEG2RAD, 1, 0, 0; // utforbi skansen
	xs_os_0 << 7042020, 269575, 130 * DEG2RAD, 1, 0, 0; // "i" skansen
	double u_d = 2.0, chi_d, u_c, chi_c;
	
	PSBMPC_LIB::CPU::Ownship asv_sim;

	Eigen::MatrixXd trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	#if OWNSHIP_TYPE == 0
		trajectory.resize(4, N);
		trajectory.col(0) = xs_os_0.block<4, 1>(0, 0);
	#else
		trajectory.resize(6, N);
		trajectory.col(0) = xs_os_0;
	#endif

	int n_wps_os = 2;
	waypoints.resize(2, n_wps_os); 
	/* waypoints << 0, 200, 200, 400, 600,  300, 500,
				 0, 0,   200, 200,  0,  0, -200; */
	waypoints << xs_os_0(0), 7042350,
				 xs_os_0(1), 270575;
	

//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	PSBMPC_LIB::CPU::PSBMPC psbmpc;
	PSBMPC_LIB::GPU::MPC_Cost<PSBMPC_LIB::PSBMPC_Parameters> mpc_cost(psbmpc.pars);
	double u_opt(u_d), chi_opt(0.0);

	double V_w = 0.0;
	Eigen::Vector2d wind_direction; wind_direction(0) = 1.0; wind_direction(1) = 0.0;

	//=========================================================
	// Matlab plot setup
	//=========================================================
	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);

	double *ptraj_os = mxGetPr(traj_os_mx); 
	double *p_wps_os = mxGetPr(wps_os_mx); 
	mxArray *T_sim_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim);

	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "WPs", wps_os_mx);

	Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, N);
	map_traj_os = trajectory;

	engPutVariable(ep, "X", traj_os_mx);

	engEvalString(ep, "test_psbmpc_cost_class_plot");

	// TODO: PLOT DISTANCE TO ALL POLYGONS IN MATLAB; VERIFY THAT IT IS CORRECT IN CPU/GPU MPC COST CLASSES
	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_os_mx);
	mxDestroyArray(T_sim_mx);


	engClose(ep);  

	return 0;
}