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
#include <Eigen/Dense>
#include <engine.h>
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
// Own-ship sim setup
//*****************************************************************************************************************

	/*coordinates are given in wgs-84 use https://finnposisjon.test.geonorge.no/ */
	Eigen::Matrix<double, 6, 1> xs_os_0;
	// xs_os_0 << 7042320, 269475, 180 * DEG2RAD, 1, 0, 0; // utforbi skansen
	xs_os_0 << 7042020, 269575, 130 * DEG2RAD, 1, 0, 0; // "i" skansen
	double u_d = 2.0, chi_d(0.0);
	
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
	
	Eigen::VectorXd offset_sequence(2), maneuver_times(1); 
	maneuver_times(0) = 0;
	offset_sequence << 1.0, 0.0;
	asv_sim.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T_sim, dt);
//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	PSBMPC_LIB::CPU::PSBMPC psbmpc;
	PSBMPC_LIB::CPU::MPC_Cost<PSBMPC_LIB::PSBMPC_Parameters> mpc_cost_cpu(psbmpc.pars);
	PSBMPC_LIB::GPU::MPC_Cost<PSBMPC_LIB::PSBMPC_Parameters> mpc_cost_gpu(psbmpc.pars);

	//double V_w = 0.0;
	//Eigen::Vector2d wind_direction; wind_direction(0) = 1.0; wind_direction(1) = 0.0;

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
    
   	PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager(filename, psbmpc);
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
			polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it);
			polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it);
			
			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		polygon_matrix(pcount, 1) = -1;
		polygon_matrix(pcount, 0) = -1;
		pcount += 1;
    }
    
	mxArray *map_origin_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
	double *p_map_origin = mxGetPr(map_origin_mx);
	Eigen::Map<Eigen::Vector2d> map_map_origin(p_map_origin, 2, 1);
	map_map_origin = grounding_hazard_manager.get_map_origin();

    mxArray *polygon_matrix_mx = mxCreateDoubleMatrix(n_static_obst, 2, mxREAL);
    double *p_polygon_matrix = mxGetPr(polygon_matrix_mx);
    Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, n_static_obst, 2);
	map_polygon_matrix = polygon_matrix;

	engPutVariable(ep, "map_origin", map_origin_mx);
	engPutVariable(ep, "P", polygon_matrix_mx);

//*****************************************************************************************************************
//	Testing
//*****************************************************************************************************************
	
	// Use positions relative to the map origin
	Eigen::Vector2d map_origin = grounding_hazard_manager.get_map_origin();
	xs_os_0.block<2, 1>(0, 0) -= map_origin;
	trajectory.block<2, 1>(0, 0) -= map_origin;
	for (int l = 0; l < n_wps_os; l++)
	{
		waypoints.col(l) -= map_origin;
	}
	
	// Test distance to polygon function
	Eigen::MatrixXd d2poly_cpu(2, polygons.size()), d2poly_gpu(2, polygons.size());
	TML::Vector2f v, p_os;
	Eigen::Vector2d e;
	PSBMPC_LIB::GPU::Basic_Polygon poly;
	point_2D p(xs_os_0(0), xs_os_0(1));
	p_os(0) = xs_os_0(0); p_os(1) = xs_os_0(1);
	n_static_obst = polygons.size();
	for (int j = 0; j < n_static_obst; j++)
	{
		std::cout << "Boost dist to poly: " << boost::geometry::distance(p, polygons[j]) << std::endl;
		d2poly_cpu.col(j) = mpc_cost_cpu.distance_to_polygon(xs_os_0.block<2, 1>(0, 0), polygons[j]);
		std::cout << "CPU dist to poly: " << d2poly_cpu.col(j).norm() << std::endl;

		poly = polygons[j];
		v = mpc_cost_gpu.distance_to_polygon(p_os, poly);
		TML::assign_tml_object(e, v);
		d2poly_gpu.col(j) = e;
		std::cout << "GPU dist to poly: " << d2poly_gpu.col(j).norm() << std::endl;
	}
	std::vector<polygon_2D> selected_polygons(1);
	selected_polygons[0] = polygons[3];
	Eigen::MatrixXd selected_polygon_matrix(1, 2);
	for(auto it = boost::begin(boost::geometry::exterior_ring(selected_polygons[0])); it != boost::end(boost::geometry::exterior_ring(selected_polygons[0])); ++it)
	{
		selected_polygon_matrix(0, 1) = boost::geometry::get<0>(*it);
		selected_polygon_matrix(0, 0) = boost::geometry::get<1>(*it);
	}

	//=========================================================
	// Matlab plot setup
	//=========================================================
	mxArray *selected_polygon_matrix_mx = mxCreateDoubleMatrix(1, 2, mxREAL);
    double *p_selected_polygon_matrix = mxGetPr(selected_polygon_matrix_mx);
    Eigen::Map<Eigen::MatrixXd> map_selected_polygon_matrix(p_selected_polygon_matrix, 1, 2);
	map_selected_polygon_matrix = selected_polygon_matrix;

	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);

	mxArray *d2poly_cpu_mx = mxCreateDoubleMatrix(2, n_static_obst, mxREAL);
	mxArray *d2poly_gpu_mx = mxCreateDoubleMatrix(2, n_static_obst, mxREAL);

	double *ptraj_os = mxGetPr(traj_os_mx); 
	double *p_wps_os = mxGetPr(wps_os_mx); 
	double *p_d2poly_cpu = mxGetPr(d2poly_cpu_mx);
	double *p_d2poly_gpu = mxGetPr(d2poly_gpu_mx);

	mxArray *T_sim_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim);

	Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, N);
	Eigen::Map<Eigen::MatrixXd> map_wps_os(p_wps_os, 2, n_wps_os);
	Eigen::Map<Eigen::MatrixXd> map_d2poly_cpu(p_d2poly_cpu, 2, n_static_obst);
	Eigen::Map<Eigen::MatrixXd> map_d2poly_gpu(p_d2poly_gpu, 2, n_static_obst);
	map_traj_os = trajectory;
	map_wps_os = waypoints;
	map_d2poly_cpu = d2poly_cpu;
	map_d2poly_gpu = d2poly_gpu;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "selected_polygon_matrix", selected_polygon_matrix_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "WPs", wps_os_mx);
	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "d2poly_cpu", d2poly_cpu_mx);
	engPutVariable(ep, "d2poly_gpu", d2poly_gpu_mx);

	engEvalString(ep, "test_psbmpc_cost_class_plot");

	printf("%s", buffer);

	mxDestroyArray(T_sim_mx);
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_os_mx);
	mxDestroyArray(selected_polygon_matrix_mx);
	mxDestroyArray(d2poly_cpu_mx);
	mxDestroyArray(d2poly_gpu_mx);
	mxDestroyArray(map_origin_mx);
	mxDestroyArray(polygon_matrix_mx);

	engClose(ep);  

	return 0;
}