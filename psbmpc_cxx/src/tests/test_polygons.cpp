/****************************************************************************************
*
*  File name : test_polygons.cpp
*
*  Function  : Test file for the grounding hazard managers polygon handling.
*			   
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengsedal NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengsedal
*
*  Modified  : 
*
*****************************************************************************************/

#include <iostream>
#include <list>

#include "cpu/psbmpc_cpu.hpp"
#include "gpu/psbmpc_gpu.cuh"
#include "cpu/utilities_cpu.hpp"
#include "grounding_hazard_manager.hpp"

#define BUFSIZE 1000000

int main()
{    
//*****************************************************************************************************************
// Own-ship position
//*****************************************************************************************************************

	/*coordinates are given in wgs-84 use https://finnposisjon.test.geonorge.no/ */
	Eigen::Matrix<double, 6, 1> xs_os_0;
	// xs_os_0 << 7042320, 269475, 180 * DEG2RAD, 1, 0, 0; // utforbi skansen
	xs_os_0 << 7042020, 269575, 130 * DEG2RAD, 1.5, 0, 0; // "i" skansen

//*****************************************************************************************************************
// Static Obstacles Setup
//*****************************************************************************************************************
	char buffer1[256];
	char *val = getcwd(buffer1, sizeof(buffer1)); // either release or debug
	if (val) {
		std::cout << buffer1 << std::endl;
	}
	// Input the path to the land data
    std::string filename = "src/tests/grounding_hazard_data/charts/land/land.shp";
    
    PSBMPC_LIB::CPU::PSBMPC psbmpc;
   	PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager(filename, psbmpc);
	std::vector<polygon_2D> polygons = grounding_hazard_manager.get_polygons();
	std::vector<polygon_2D> simplified_polygons = grounding_hazard_manager.get_simplified_polygons();

	//=================================
	// POLYGONS
	//=================================
    //Make matlab polygons type friendly array:
    Eigen::Matrix<double, -1, 2> polygon_matrix, simplified_polygon_matrix;
    int n_total_vertices = 0;
    BOOST_FOREACH(polygon_2D const &poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; ++it)
		{
			n_total_vertices += 1;
		}
		n_total_vertices += 1;
    }
    polygon_matrix.resize(n_total_vertices, 2); 

    /*format polygon_matrix array for matlab plotting*/
    int pcount = 0; 
    BOOST_FOREACH(polygon_2D const& poly, polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; ++it)
		{
			polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it); // east 
			polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it); // north format for matlab
			
			pcount += 1;
		}
		// each polygon is separated with (-1e6, -1e6)
		polygon_matrix(pcount, 1) = -1e6;
		polygon_matrix(pcount, 0) = -1e6;
		pcount += 1;
    }

	//PSBMPC_LIB::CPU::save_matrix_to_file(polygon_matrix);
	//=================================
	// SIMPLIFIED POLYGONS
	//=================================
	int n_total_vertices_simplified = 0;
    BOOST_FOREACH(polygon_2D const &poly, simplified_polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; ++it)
		{
			n_total_vertices_simplified += 1;
		}
		n_total_vertices_simplified += 1;
    }
    simplified_polygon_matrix.resize(n_total_vertices_simplified, 2); 

    /*format polygon_matrix array for matlab plotting*/
    pcount = 0; 
    BOOST_FOREACH(polygon_2D const& poly, simplified_polygons)
	{
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; ++it)
		{
			simplified_polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it); // east 
			simplified_polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it); // north format for matlab
			
			pcount += 1;
		}
		// each polygon is separated with (-1e6, -1e6)
		simplified_polygon_matrix(pcount, 1) = -1e6;
		simplified_polygon_matrix(pcount, 0) = -1e6;
		pcount += 1;
    }

//*****************************************************************************************************************
// Matlab plotting stuff
//*****************************************************************************************************************
    // Matlab engine setup
 	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
    char buffer[BUFSIZE+1]; 
    buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

    mxArray *map_origin_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
    mxArray *polygon_matrix_mx = mxCreateDoubleMatrix(n_total_vertices, 2, mxREAL);
	mxArray *simplified_polygon_matrix_mx = mxCreateDoubleMatrix(n_total_vertices_simplified, 2, mxREAL);

	double *p_map_origin = mxGetPr(map_origin_mx);
    double *p_polygon_matrix = mxGetPr(polygon_matrix_mx);
	double *p_simplified_polygon_matrix = mxGetPr(simplified_polygon_matrix_mx);

	Eigen::Map<Eigen::Vector2d> map_map_origin(p_map_origin, 2, 1);
    Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, n_total_vertices, 2);
	Eigen::Map<Eigen::MatrixXd> map_simplified_polygon_matrix(p_simplified_polygon_matrix, n_total_vertices_simplified, 2);

	map_map_origin = grounding_hazard_manager.get_map_origin();
	map_polygon_matrix = polygon_matrix;
	map_simplified_polygon_matrix = simplified_polygon_matrix;

	engPutVariable(ep, "map_origin", map_origin_mx);
	engPutVariable(ep, "P", polygon_matrix_mx);
	engPutVariable(ep, "P_simplified", simplified_polygon_matrix_mx);
    engEvalString(ep, "test_polygons_plot");

    printf("%s\n", buffer);

    mxDestroyArray(map_origin_mx);
    mxDestroyArray(polygon_matrix_mx);
    mxDestroyArray(simplified_polygon_matrix_mx);

    return 0;
}