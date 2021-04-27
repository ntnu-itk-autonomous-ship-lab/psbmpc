/****************************************************************************************
*
*  File name : test_psbmpc.cpp
*
*  Function  : Test file for the Probabilistic Scenario-based Model Predictive Control with anti-grounding
*			   using Matlab for visualization 
*			   
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Tom Daniel Grande, Trym Tengsedal NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Tom Daniel Grande, Trym Tengsedal
*
*  Modified  : 
*
*****************************************************************************************/


#include "cpu/psbmpc_cpu.hpp"
#include "cpu/utilities_cpu.hpp"
#include "engine.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <string>


#define BUFSIZE 1000000

template <typename T, typename F>
void read_shapefile(const std::string &filename, std::vector<T> &polygons, F functor)
{
    try
    {
        SHPHandle handle = SHPOpen(filename.c_str(), "rb");
        if (handle <= (SHPHandle)0)
        {
            throw std::string("File " + filename + " not found");
        }

        int nShapeType, nEntities;
        double adfMinBound[4], adfMaxBound[4];
        SHPGetInfo(handle, &nEntities, &nShapeType, adfMinBound, adfMaxBound );

        for (int i = 0; i < nEntities; i++)
        {
            SHPObject* psShape = SHPReadObject(handle, i );

            // Read only polygons, and only those without holes
            if (psShape->nSHPType == SHPT_POLYGON && psShape->nParts == 1)
            {
                T polygon;
                functor(psShape, polygon);
                polygons.push_back(polygon);
            }
            SHPDestroyObject( psShape );
        }
        SHPClose(handle);
    }
    catch(const std::string &s)
    {
        throw s;
    }
    catch(...)
    {
        throw std::string("Other exception");
    }
}


template <typename T>
void convert(SHPObject *psShape, T &polygon)
{
    double* x = psShape->padfX;
    double* y = psShape->padfY;
    for (int v = 0; v < psShape->nVertices; v++)
    {
        typename boost::geometry::point_type<T>::type point;
        boost::geometry::assign_values(point, x[v], y[v]);
        boost::geometry::append(polygon, point);
    }
}

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
	double T_sim = 1000; double dt = 0.5;
	int N = std::round(T_sim / dt);


//*****************************************************************************************************************
// Static Obstacles Setup
//*****************************************************************************************************************
	// Input the path to the land data
    std::string filename = "/home/admin/Desktop/ENC (copy)/data/charts/land/land.shp";
    std::vector<polygon_2D> polygons;

    try
    {
        read_shapefile(filename, polygons, convert<polygon_2D>);
    }
    catch(const std::string& s)
    {
        std::cout << s << std::endl;
        return 1;
    }

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
    polygon_matrix.resize(n_static_obst,2); 

    /*format polygon_matrix array for matlab plotting*/
    int pcount = 0 ; 
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
	//remove polygons which are irrelevantly far away for code speed. 
	point_2D p1(316868,7087612);
	point_2D p2(318807,7087612);
	point_2D p3(318807,7087552);
	
	int d_irrelevant = 2000; 		//The polygons are so far away we can remove them from SB-MPC calculations. 
	int iterator = 0;
	bool isdone = false;
	bool isbreak =false;
	while (isdone == false)
	{
		isdone = true;
		BOOST_FOREACH(polygon_2D const& poly, polygons)
		{
			isbreak = false;
			double distance_to_point_1 = boost::geometry::distance(p1, poly);
			double distance_to_point_2 = boost::geometry::distance(p2, poly);
			double distance_to_point_3 = boost::geometry::distance(p3, poly);

			if ((distance_to_point_1 > d_irrelevant) && (distance_to_point_2 > d_irrelevant) && (distance_to_point_3 > d_irrelevant))
			{
				polygons.erase(polygons.begin() + iterator);
				iterator=0;
				isbreak = true;
				break;
			}
			else
			{
				iterator += 1;
				isbreak = false;
			}
		}
		if(isbreak==true)
		{
			isdone = false;
			iterator = 0;
		}
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
	xs_os_0 << 7087612, 316868, 0, 9, 0, 0;
	double u_d = 9, chi_d, u_c, chi_c;
	
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
	waypoints << 7087612,7087552,
				 316868, 318807;
	
//*****************************************************************************************************************
// Obstacle sim setup
//*****************************************************************************************************************
	int n_obst = 1;
	std::vector<int> ID(n_obst);

	std::vector<Eigen::VectorXd> xs_i_0(n_obst);

	// Use constant obstacle uncertainty throughout the simulation, for simplicity
	Eigen::MatrixXd P_0(4, 4);
	P_0 << 25, 0, 0, 0,
	     0, 25, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	double A = 5, B = 5, C = 5, D = 5; 
	
	// Use constant equal intention probability and a priori CC probability  for simplicity
	std::vector<Eigen::VectorXd> Pr_a(n_obst);

	std::vector<double> Pr_CC(n_obst);

	// Simulate obstacles using an ownship model
	PSBMPC_LIB::CPU::Ownship obstacle_sim;

	std::vector<double> u_d_i(n_obst);
	std::vector<double> chi_d_i(n_obst);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_obst);

	std::vector<Eigen::VectorXd> maneuver_times_i(n_obst);

	std::vector<Eigen::MatrixXd> trajectory_i(n_obst); 
	std::vector<Eigen::Matrix<double, 16, -1>> trajectory_covariances_i(n_obst);
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_obst);

	//=====================================================================
	// Matlab array setup for the ownship and obstacle, ++
	//=====================================================================
	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);

	double *ptraj_os = mxGetPr(traj_os_mx); 
	double *p_wps_os = mxGetPr(wps_os_mx); 

	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_os, 2, n_wps_os);
	map_wps_i = waypoints;

	std::vector<mxArray*> traj_i_mx(n_obst); 
	std::vector<mxArray*> P_traj_i_mx(n_obst); 
	std::vector<mxArray*> wps_i_mx(n_obst);

	double* ptraj_i; 
	double* p_P_traj_i; 
	double* p_wps_i;
	int n_wps_i;

	for (int i = 0; i < n_obst; i++)
	{
		
		ID[i] = i;

		u_d_i[i] = 3.0; chi_d_i[i] = 0.0;

		#if OWNSHIP_TYPE == 0
			xs_i_0[0].resize(4);
			xs_i_0[0] << 7087569, 317514, -60 * DEG2RAD, 5;
			trajectory_i[i].resize(4, N);
			trajectory_i[i].col(0) = xs_i_0[i];
		#else
			xs_i_0[0].resize(6);
			xs_i_0[0] << 7087569, 317514, -60 * DEG2RAD, 5, 0, 0;
			trajectory_i[i].resize(6, N);
			trajectory_i[i].col(0) = xs_i_0[i];
		#endif
		

		trajectory_covariances_i[i].resize(16, 1);
		trajectory_covariances_i[i].col(0) = PSBMPC_LIB::CPU::flatten(P_0);

		/* Pr_a[i].resize(3);
		Pr_a[i] << 1, 1, 1;
		Pr_a[i] = Pr_a[0] / Pr_a[0].sum(); */
		Pr_a[i].resize(1);
		Pr_a[i] << 1;

		Pr_CC[i] = 1;

		n_wps_i = 2;
		waypoints_i[i].resize(2, n_wps_i); 
		waypoints_i[i] << 7087569, 7087662,
					317514, 316773;
		
		offset_sequence_i[i].resize(6);
		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

		maneuver_times_i[i].resize(3);
		maneuver_times_i[i] << 0, 100, 150;

		// Simulate obstacle trajectory independent on the ownship
		obstacle_sim.predict_trajectory(trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T_sim, dt);
	}

//*****************************************************************************************************************
// Obstacle Manager setup
//*****************************************************************************************************************	
	PSBMPC_LIB::Obstacle_Manager obstacle_manager;
//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	PSBMPC_LIB::CPU::PSBMPC psbmpc;
	double u_opt, chi_opt;

	Eigen::Matrix<double, 2, -1> predicted_trajectory; 

	Eigen::Matrix<double,-1,-1> obstacle_status; 				
	Eigen::Matrix<double,-1, 1> colav_status; 

	Eigen::Matrix<double, 9, -1> obstacle_states;
	obstacle_states.resize(9, n_obst);

	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	obstacle_covariances.resize(16, n_obst);

	Eigen::MatrixXd obstacle_intention_probabilities;
	obstacle_intention_probabilities.resize(1, n_obst);

	Eigen::VectorXd obstacle_a_priori_CC_probabilities(n_obst);

//*****************************************************************************************************************
// Simulation
//*****************************************************************************************************************	
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	//=========================================================
	// Matlab plot setup
	//=========================================================

	for (int i = 0; i < n_obst; i++)
	{
		wps_i_mx[i] = mxCreateDoubleMatrix(2, n_wps_i, mxREAL);
		traj_i_mx[i] = mxCreateDoubleMatrix(trajectory_i[i].rows(), N, mxREAL);
		P_traj_i_mx[i] = mxCreateDoubleMatrix(16, 1, mxREAL);
	}
	
	mxArray *T_sim_mx, *n_obst_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim);
	n_obst_mx = mxCreateDoubleScalar(n_obst);

	mxArray *pred_traj_mx;
	double *p_pred_traj;


	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "WPs", wps_os_mx);

	engEvalString(ep, "init_psbmpc_plotting_grounding");
	mxArray *i_mx, *k_s_mx;

	for (int i = 0; i < n_obst; i++)
	{
		p_wps_i = mxGetPr(wps_i_mx[i]);

		Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i);
		map_wps_i = waypoints_i[i];

		engPutVariable(ep, "WPs_i", wps_i_mx[i]);

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		engEvalString(ep, "init_obstacle_plot");
	}
	//=========================================================
	
	Eigen::Vector4d xs_i_k;
	Eigen::VectorXd xs_aug(9);
	double mean_t = 0, t(0.0);
	for (int k = 0; k < N; k++)
	{
		t = k * dt;

		// Aquire obstacle information
		for (int i = 0; i < n_obst; i++)
		{
			if (trajectory_i[i].rows() == 4)
			{
				xs_i_k = trajectory_i[i].col(k);
			}
			else
			{
				xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
				xs_i_k.block<2, 1>(2, 0) = PSBMPC_LIB::CPU::rotate_vector_2D(trajectory_i[i].block<2, 1>(3, k), trajectory_i[i](2, k));
			}
			obstacle_states.col(i) << xs_i_k, A, B, C, D, ID[i];

			obstacle_covariances.col(i) = PSBMPC_LIB::CPU::flatten(P_0);

			obstacle_intention_probabilities.col(i) = Pr_a[i];
			obstacle_a_priori_CC_probabilities(i) = Pr_CC[i];
		}

		obstacle_manager.operator()(
			psbmpc.pars, 
			trajectory.col(k), 
			asv_sim.get_length(),
			obstacle_states, 
			obstacle_covariances, 
			obstacle_intention_probabilities, 
			obstacle_a_priori_CC_probabilities);

		asv_sim.update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, PSBMPC_LIB::LOS);

		if (fmod(t, 5) == 0)
		{
			start = std::chrono::system_clock::now();		

			psbmpc.calculate_optimal_offsets(
				u_opt,
				chi_opt, 
				predicted_trajectory,
				u_d,
				chi_d,
				waypoints,
				trajectory.col(k),
				polygons,
				obstacle_manager.get_data());

			end = std::chrono::system_clock::now();
			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

			mean_t = elapsed.count();

			std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;

			obstacle_manager.update_obstacle_status(trajectory.col(k));
			obstacle_manager.display_obstacle_information();
		
		}
		u_c = u_d * u_opt; chi_c = chi_d + chi_opt;
		
		if (k < N - 1) { trajectory.col(k + 1) = asv_sim.predict(trajectory.col(k), u_c, chi_c, dt, PSBMPC_LIB::ERK1); }

		//===========================================
		// Send trajectory data to matlab
		//===========================================
		buffer[BUFSIZE] = '\0';
		engOutputBuffer(ep, buffer, BUFSIZE);

		k_s_mx = mxCreateDoubleScalar(k + 1);
		engPutVariable(ep, "k", k_s_mx);
		
		pred_traj_mx = mxCreateDoubleMatrix(predicted_trajectory.rows(), predicted_trajectory.cols(), mxREAL);
		p_pred_traj = mxGetPr(pred_traj_mx);

		Eigen::Map<Eigen::MatrixXd> map_pred_traj_os(p_pred_traj, 2, predicted_trajectory.cols());
		map_pred_traj_os = predicted_trajectory;

		Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, 6, N);
		map_traj_os = trajectory;

		engPutVariable(ep, "X_pred", pred_traj_mx);
		engPutVariable(ep, "X", traj_os_mx);

		engEvalString(ep, "update_ownship_plot");

		for(int i = 0; i < n_obst; i++)
		{
			ptraj_i = mxGetPr(traj_i_mx[i]);
			p_P_traj_i = mxGetPr(P_traj_i_mx[i]);

			Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 6, N);
			Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, 1);
			
			map_traj_i = trajectory_i[i];
			map_P_traj_i = trajectory_covariances_i[i];
			
			engPutVariable(ep, "X_i", traj_i_mx[i]);
			engPutVariable(ep, "P_i", P_traj_i_mx[i]);

			i_mx = mxCreateDoubleScalar(i + 1);
			engPutVariable(ep, "i", i_mx);

			engEvalString(ep, "update_obstacle_plot");
		}
		//======================================================
		
	}

	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_os_mx);
	mxDestroyArray(pred_traj_mx);
	mxDestroyArray(i_mx);
	mxDestroyArray(k_s_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(n_obst_mx);
	mxDestroyArray(pred_traj_mx);
	for (int i = 0; i < n_obst; i++)
	{
		mxDestroyArray(traj_i_mx[i]);
		mxDestroyArray(P_traj_i_mx[i]);
		mxDestroyArray(wps_i_mx[i]);
	}
	engClose(ep);  

	return 0;
}