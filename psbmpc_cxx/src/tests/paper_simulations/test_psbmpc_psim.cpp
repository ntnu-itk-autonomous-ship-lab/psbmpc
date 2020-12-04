/****************************************************************************************
*
*  File name : test_psbmpc.cpp
*
*  Function  : Test file for the Probabilistic Scenario-based Model Predictive Control
*			   using Matlab for visualization, for paper simulations
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

#include "obstacle_ship.h"
#include "psbmpc.h"
#include "sbmpc.h"
#include "utilities.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include "Eigen/Dense"
#include "engine.h"
#include "xoshiro.hpp"


#define BUFFSIZE 1000000

int main(){
	// Matlab engine setup
 	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "Engine start failed!" << std::endl;
	}
	char buffer[BUFFSIZE+1]; 

//*****************************************************************************************************************
// Simulation setup
//*****************************************************************************************************************
	double T_sim = 115; double dt = 0.5;
	int N = std::round(T_sim / dt);

	std::random_device seed;

	std::mt19937_64 eng1(seed()); 

	std::normal_distribution<double> std_norm_pdf(0.0, 1.0);


//*****************************************************************************************************************
// Own-ship data allocation
//*****************************************************************************************************************
	Eigen::Matrix<double, 6, 1> xs_os_0;
	
	double u_d, u_d_pr = u_d, chi_d, chi_d_pr, u_c, u_c_pr, chi_c, chi_c_pr;
	
	Ownship os, os_pr;

	Eigen::Matrix<double, 4, -1> aux_data_os, aux_data_os_pr;		// Contains autopilot references u_d, chi_d, and modifications u_m, chi_m from the COLAV system
	Eigen::Matrix<double, 6, -1> trajectory, trajectory_pr; 
	Eigen::Matrix<double, 2, -1> waypoints;
	int n_wps_os = 2;
	waypoints.resize(2, n_wps_os); 
	/* waypoints << 0, 200, 200, 400, 600,  300, 500,
				0, 0,   200, 200,  0,  0, -200; */
	waypoints << 0, 1000,
				0, 0;

	aux_data_os.resize(4, N);
	aux_data_os_pr = aux_data_os;

	trajectory.resize(6, N);
	
//*****************************************************************************************************************
// Obstacle data setup
//*****************************************************************************************************************
	int n_obst = 2, n_obst_s = 2;
	std::vector<int> ID(n_obst);

	// Initial obstacle gt state estimates, a priori state estimates and uncertainty for the KF's
	std::vector<Eigen::Vector4d> xs_i_0(n_obst);
	std::vector<Eigen::Vector4d> xs_tr_i_0(n_obst);
	Eigen::MatrixXd P_0(4, 4);

	// Obstacle dimensions
	double A = 10, B = 10, C = 2.5, D = 2.5;  // (length = 20 m, width = 5 m)
	
	// Measurement generating (mg) Covariance matrix
	Eigen::Matrix2d R_mg;
	Eigen::Matrix2d L; 
	Eigen::MatrixXd aux_data_i(32, n_obst); // Consist of R_KF and Q_KF
	
	// Set up tracking filters for each obstacle
	std::vector<KF> kf_i(n_obst);
	// KF measurement and process noise covariance matrices (CV model)
	double sigma_a;
	Eigen::Matrix4d R_KF, Q_KF;
	double dt_KF;

	double T_U, T_chi, R_a, LOS_LD;
	Obstacle_Ship obstacle_ship;

	std::vector<double> u_d_i(n_obst);
	std::vector<double> chi_d_i(n_obst);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_obst);
	std::vector<Eigen::VectorXd> maneuver_times_i(n_obst);

	// Use constant equal intention probability and a priori CC probability for simplicity
	std::vector<Eigen::MatrixXd> Pr_a(n_obst);
	std::vector<Eigen::Matrix<double, 1, -1>> Pr_CC(n_obst);

	std::vector<Eigen::Matrix<double, 2, -1>> measurement_i(n_obst);					// Measurement sequence generated for obstacles
	std::vector<Eigen::Matrix<double, 4, -1>> gt_trajectory_i(n_obst);					// Ground truth obstacle trajectories
	std::vector<Eigen::Matrix<double, 8, -1>> tr_trajectory_i(n_obst); 					// Tracked obstacle trajectories, consisting of [xs_i_p; xs_i_upd], thus 8 elements
	std::vector<Eigen::Matrix<double, 32, -1>> tr_trajectory_covariances_i(n_obst);		// Covariances associated with tracked obstacle trajectories, consisting of [flatten(P_i_p; flatten(P_i_upd)], thus 32 elements
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_obst);						// Waypoints that the obstacles are simulated to follow

//*****************************************************************************************************************
// PSB-MPC setup
//*****************************************************************************************************************	
	Obstacle_Manager sbmpc_obstacle_manager, psbmpc_obstacle_manager;
	PSBMPC psbmpc;
	SBMPC sbmpc;
	double u_opt, u_opt_pr, chi_opt, chi_opt_pr;

	Eigen::Matrix<double, 2, -1> predicted_trajectory, predicted_trajectory_pr; 

	Eigen::Matrix<double,-1,-1> obstacle_status; 				
	Eigen::Matrix<double,-1, 1> colav_status; 

	Eigen::Matrix<double, 9, -1> obstacle_states;
	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	Eigen::MatrixXd obstacle_intention_probabilities;
	Eigen::Matrix<double, 1, -1> obstacle_a_priori_CC_probabilities(n_obst);

	int n_static_obst = 0;
	Eigen::Matrix<double, 4, -1> static_obstacles;

	obstacle_states.resize(9, n_obst);	
	obstacle_covariances.resize(16, n_obst);	
	obstacle_intention_probabilities.resize(3, n_obst);

	// Format of each column: v_0, v_1, where v_0 and v_1 are the (x,y) coordinates of the start and end
	// of the static obstacle no-go zone
	static_obstacles.resize(4, n_static_obst);
    //static_obstacles.col(0) << 500.0, 300.0, 1000.0, 50.0;

//=====================================================================
// Matlab data setup for the ownship and obstacle data, ++
//=====================================================================
	// Ownship matlab arrays
	mxArray *aux_data_os_mx = mxCreateDoubleMatrix(4, N, mxREAL);
	mxArray *aux_data_os_pr_mx = mxCreateDoubleMatrix(4, N, mxREAL);
	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *traj_os_pr_mx = mxCreateDoubleMatrix(6, N, mxREAL);
	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);
	mxArray *pred_traj_os_mx;
	mxArray *pred_traj_os_pr_mx;
	// Ownship pointers to matlab arrays
	double *p_aux_data_os = mxGetPr(aux_data_os_mx);
	double *p_aux_data_os_pr = mxGetPr(aux_data_os_pr_mx);
	double *p_traj_os = mxGetPr(traj_os_mx); 
	double *p_traj_os_pr = mxGetPr(traj_os_pr_mx); 
	double *p_wps_os = mxGetPr(wps_os_mx); 
	double *p_pred_traj_os(nullptr);
	double *p_pred_traj_os_pr(nullptr);

	Eigen::Map<Eigen::MatrixXd> map_wps_os(p_wps_os, 2, n_wps_os);
	map_wps_os = waypoints;

	// R_cov mxArray
	mxArray *R_mg_mx = mxCreateDoubleMatrix(2, 2, mxREAL);
	double* p_R_mg = mxGetPr(R_mg_mx);
	Eigen::Map<Eigen::Matrix2d> map_R_mg(p_R_mg, 2, 2);

	// Obstacle matlab arrays
	mxArray* aux_data_i_mx = mxCreateDoubleMatrix(32, n_obst, mxREAL);
	std::vector<mxArray*> y_m_i_mx(n_obst);
	std::vector<mxArray*> gt_traj_i_mx(n_obst); 
	std::vector<mxArray*> tr_traj_i_mx(n_obst); 
	std::vector<mxArray*> tr_P_traj_i_mx(n_obst); 
	std::vector<mxArray*> wps_i_mx(n_obst);
	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, n_static_obst, mxREAL);
	std::vector<mxArray*> Pr_a_i_mx(n_obst);
	std::vector<mxArray*> Pr_CC_i_mx(n_obst);
	
	// Obstacle pointers to matlab arrays
	double* p_aux_data_i = mxGetPr(aux_data_i_mx);
	double* p_y_m_i(nullptr);
	double* p_gt_traj_i(nullptr);
	double* p_tr_traj_i(nullptr); 
	double* p_tr_P_traj_i(nullptr); 
	double* p_wps_i(nullptr);
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 
	double *p_Pr_a_i(nullptr);
	double *p_Pr_CC_i(nullptr);

	std::vector<int> n_wps_i(n_obst);

	// Other matlab objects
	mxArray *k_pt_mx(nullptr), *k_MC_mx(nullptr), *T_sim_mx, *dt_sim_mx, *n_obst_mx(nullptr), *n_static_obst_mx(nullptr), 
			*stype_mx(nullptr), *i_mx(nullptr), *k_s_mx(nullptr), *d_safe_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim); dt_sim_mx = mxCreateDoubleScalar(dt);
	d_safe_mx = mxCreateDoubleScalar(psbmpc.pars.get_dpar(i_dpar_d_safe));

	engPutVariable(ep, "d_safe", d_safe_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);

	// Eigen Maps for use in transfering Eigen matrices to the matlab mxArrays
	// There is no default constructor, so all these maps are empty with this 
	// initialization
	Eigen::Map<Eigen::MatrixXd> map_aux_data_os(p_aux_data_os, 4, N);
	Eigen::Map<Eigen::MatrixXd> map_aux_data_os_pr(p_aux_data_os_pr, 4, N);
	Eigen::Map<Eigen::MatrixXd> map_pred_traj_os(p_pred_traj_os, 2, N);
	Eigen::Map<Eigen::MatrixXd> map_pred_traj_os_pr(p_pred_traj_os_pr, 2, N);
	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, 6, N);
	Eigen::Map<Eigen::MatrixXd> map_traj_os_pr(p_traj_os_pr, 6, N);

	Eigen::Map<Eigen::MatrixXd> map_aux_data_i(p_aux_data_i, 32, n_obst);
	Eigen::Map<Eigen::MatrixXd> map_y_m_i(p_y_m_i, 2, N);
	Eigen::Map<Eigen::MatrixXd> map_gt_traj_i(p_gt_traj_i, 4, N);
	Eigen::Map<Eigen::MatrixXd> map_tr_traj_i(p_tr_traj_i, 8, N);
	Eigen::Map<Eigen::MatrixXd> map_tr_P_traj_i(p_tr_P_traj_i, 32, N);
	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i[0]);
	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, n_static_obst);
	Eigen::Map<Eigen::MatrixXd> map_Pr_a_i(p_Pr_a_i, 3, N);
	Eigen::Map<Eigen::Matrix<double, 1, -1>> map_Pr_CC_i(p_Pr_CC_i, 1, N);

	// Temporary data for simulation
	Eigen::Vector2d y_i, z, mu; mu.setZero();
	Eigen::Vector4d xs_i_k;
	Eigen::VectorXd xs_aug(9);
	double mean_t(0.0), t(0.0);
	
//*****************************************************************************************************************
// Simulation
//*****************************************************************************************************************	
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	
	Eigen::VectorXi scenario_type(3); scenario_type << 1, 2, 3;
	//==============================================================================================================
	// Simulation for each parameter test
	//==============================================================================================================
	
	Eigen::VectorXd pars(1);
	pars << 2.5;
	int N_par_tests = 1; //kappa_pars.size();
	int N_MCS = 50;

	for (int k_pt = 0; k_pt < N_par_tests; k_pt++)
	{
		k_pt_mx = mxCreateDoubleScalar(k_pt + 1);
		engPutVariable(ep, "k_pt", k_pt_mx);
		
		std::cout << "Parameter test: " << k_pt + 1 << std::endl;

		psbmpc.pars.set_par(i_dpar_K_coll, pars(k_pt));
		sbmpc.pars.set_par(i_dpar_K_coll, pars(k_pt));

		//==============================================================================================================
		// Simulation for each MCS. Each MCS runs all 3 scenarios.
		//==============================================================================================================
		
		for (int k_MC = 0; k_MC < N_MCS; k_MC++)
		{			
			std::cout << "MC run: " << k_MC + 1 << std::endl;
			
			k_MC_mx = mxCreateDoubleScalar(k_MC + 1);
			engPutVariable(ep, "k_MC", k_MC_mx);

			eng1.seed(k_MC);

			//=========================================================
			// Data initialization for an MC run of all scenario types
			//=========================================================
			
			for (int s = 2; s < scenario_type.size(); s++)
			{
				std::cout << "Scenario: " << scenario_type(s) << std::endl;

				stype_mx = mxCreateDoubleScalar(scenario_type(s));
				engPutVariable(ep, "stype", stype_mx);

				//============================================
				// Crossing collision scenario, conservative KF
				//============================================
				if (scenario_type(s) == 1)
				{
					n_obst_s = 1;	// Number of obstacles in this scenario
					n_wps_i.resize(n_obst_s);
					obstacle_states.resize(9, n_obst_s);	
					obstacle_covariances.resize(16, n_obst_s);	
					obstacle_intention_probabilities.resize(3, n_obst_s);
					obstacle_a_priori_CC_probabilities.resize(n_obst_s);

					// Ownship data initialization
					xs_os_0 << 0, 0, 0, 9, 0, 0;
					trajectory.col(0) = xs_os_0;	trajectory_pr = trajectory;

					u_d = 9.0; u_d_pr = u_d;

					// Obstacle data initialization
					P_0 << 	25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0.025, 0,
							0, 0, 0, 0.025;

					// Measurement generation covariance
					R_mg << 25, 0,
							0, 25;
					L = R_mg.llt().matrixL();

					// KF tuning matrices, conservative filter
					sigma_a =  0.5;		dt_KF = 2.5;
					R_KF << 25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0, 0,
							0, 0, 0, 0;
					R_KF = 5 * R_KF;
					Q_KF << pow(dt_KF, 3) / 3.0, 			0, 				pow(dt_KF, 2) / 2.0, 			0,
									0,		  		pow(dt_KF, 3) / 3, 				0, 					pow(dt_KF, 2) / 2.0,
							pow(dt_KF, 2) / 2.0, 			0, 					dt_KF, 						0,
									0, 				pow(dt_KF, 2) / 2.0, 			0, 						dt_KF;
					Q_KF = sigma_a * Q_KF;

					// Simulate obstacles using a simple kinematic model based ship
					T_U = 10; T_chi = 7.5; R_a = 30.0; LOS_LD = 200.0;
					obstacle_ship = Obstacle_Ship(T_U, T_chi, R_a, LOS_LD, 0); // xs = [x, y, chi, U]^T for this ship
					for (int i = 0; i < n_obst; i++)
					{
						ID[i] = i;

						aux_data_i.block<16, 1>(0, i) = flatten(R_KF);
						aux_data_i.block<16, 1>(16, i) = flatten(Q_KF);

						// desired course will be set by the guidance in the obstacle ship prediction
						u_d_i[i] = 9.0; chi_d_i[i] = 0.0;

						// --GT initialization--
						xs_i_0[i] << 300, 275, -90 * DEG2RAD, u_d_i[i];

						gt_trajectory_i[i].resize(4, N);	gt_trajectory_i[i].col(0) = xs_i_0[i];
						//--

						n_wps_i[i] = 4;
						waypoints_i[i].resize(2, n_wps_i[i]); 
						waypoints_i[i] << 	xs_i_0[i](0), 300, 290, 0,
											xs_i_0[i](1), 125,  80, 25;
						
						offset_sequence_i[i].resize(6);		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

						maneuver_times_i[i].resize(3);	maneuver_times_i[i] << 0, 100, 150;

						// Simulate obstacle trajectory independent on the ownship
						obstacle_ship.predict_trajectory(gt_trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], ERK1, LOS, T_sim, dt);

						Pr_a[i].resize(3, N);
						Pr_CC[i].resize(N);
						
						measurement_i[i].resize(2, N);
						for (int k = 0; k < N; k++)
						{	
							Pr_a[i](0, k) = 1; Pr_a[i](1, k) = 1; Pr_a[i](2, k) = 1;
							Pr_a[i].col(k) = Pr_a[i].col(k) / Pr_a[i].col(k).sum();
							Pr_CC[i](k) = 0.5;

							// Generate measurement vector for obstacle i by adding measurement noise to the trajectory, with covariance R_cov
							z(0) = std_norm_pdf(eng1); z(1) = std_norm_pdf(eng1);
							measurement_i[i].col(k) = gt_trajectory_i[i].block<2, 1>(0, k) + L * z;
						}
						// GT initialization
						xs_tr_i_0[i] << xs_i_0[i].block<2, 1>(0, 0), xs_i_0[i](3) * cos(xs_i_0[i](2)), xs_i_0[i](3) * sin(xs_i_0[i](2)); 

						// Single-point initialization
						//xs_tr_i_0[i] << measurement_i[i].col(0), 0, 0;

						tr_trajectory_i[i].resize(8, N);	tr_trajectory_covariances_i[i].resize(32, N);	tr_trajectory_covariances_i[i].block<16, 1>(0, 0) = flatten(P_0);

						kf_i[i] = KF(xs_tr_i_0[i], P_0, ID[i], dt_KF, 0.0, Q_KF, R_KF);
					}
				}
				//============================================
				// Head-on collision scenario, conservative KF
				//============================================
				else if(scenario_type(s) == 2)
				{
					n_obst_s = 1;	// Number of obstacles in this scenario
					n_wps_i.resize(n_obst_s);
					obstacle_states.resize(9, n_obst_s);	
					obstacle_covariances.resize(16, n_obst_s);	
					obstacle_intention_probabilities.resize(3, n_obst_s);
					obstacle_a_priori_CC_probabilities.resize(n_obst_s);

					// Ownship data initialization
					xs_os_0 << 0, 0, 0, 9, 0, 0;
					trajectory.col(0) = xs_os_0;	trajectory_pr = trajectory;

					u_d = 9.0; u_d_pr = u_d;

					// Obstacle data initialization
					P_0 << 	25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0.025, 0,
							0, 0, 0, 0.025;

					// Measurement generation covariance
					R_mg << 25, 0,
							0, 25;
					L = R_mg.llt().matrixL();

					// KF tuning matrices
					sigma_a =  0.5;		dt_KF = 2.5;
					R_KF << 25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0, 0,
							0, 0, 0, 0;
					R_KF = 5 * R_KF;
					Q_KF << pow(dt_KF, 3) / 3.0, 			0, 				pow(dt_KF, 2) / 2.0, 			0,
									0,		  		pow(dt_KF, 3) / 3, 				0, 					pow(dt_KF, 2) / 2.0,
							pow(dt_KF, 2) / 2.0, 			0, 					dt_KF, 						0,
									0, 				pow(dt_KF, 2) / 2.0, 			0, 						dt_KF;
					Q_KF = sigma_a * Q_KF;

					// Simulate obstacles using a simple kinematic model based ship
					T_U = 10; T_chi = 7.5; R_a = 30.0; LOS_LD = 200.0;
					obstacle_ship = Obstacle_Ship(T_U, T_chi, R_a, LOS_LD, 0); // xs = [x, y, chi, U]^T for this ship
					for (int i = 0; i < n_obst; i++)
					{
						ID[i] = i;

						aux_data_i.block<16, 1>(0, i) = flatten(R_KF);
						aux_data_i.block<16, 1>(16, i) = flatten(Q_KF);

						// desired course will be set by the guidance in the obstacle ship prediction
						u_d_i[i] = 9.0; chi_d_i[i] = 0.0;

						// --GT initialization--
						xs_i_0[i] << 475, 0, -180 * DEG2RAD, u_d_i[i];

						gt_trajectory_i[i].resize(4, N);	gt_trajectory_i[i].col(0) = xs_i_0[i];
						//--

						n_wps_i[i] = 3;
						waypoints_i[i].resize(2, n_wps_i[i]); 
						waypoints_i[i] << 	xs_i_0[i](0), 325,-100,
											xs_i_0[i](1), 0, 400;
						
						offset_sequence_i[i].resize(6);		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

						maneuver_times_i[i].resize(3);	maneuver_times_i[i] << 0, 100, 150;

						// Simulate obstacle trajectory independent on the ownship
						obstacle_ship.predict_trajectory(gt_trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], ERK1, LOS, T_sim, dt);

						Pr_a[i].resize(3, N);
						Pr_CC[i].resize(N);
						
						measurement_i[i].resize(2, N);
						for (int k = 0; k < N; k++)
						{	
							Pr_a[i](0, k) = 1; Pr_a[i](1, k) = 1; Pr_a[i](2, k) = 1;
							Pr_a[i].col(k) = Pr_a[i].col(k) / Pr_a[i].col(k).sum();
							Pr_CC[i](k) = 0.5;

							// Generate measurement vector for obstacle i by adding measurement noise to the trajectory, with covariance R_cov
							z(0) = std_norm_pdf(eng1); z(1) = std_norm_pdf(eng1);
							measurement_i[i].col(k) = gt_trajectory_i[i].block<2, 1>(0, k) + L * z;
						}
						// GT initialization
						xs_tr_i_0[i] << xs_i_0[i].block<2, 1>(0, 0), xs_i_0[i](3) * cos(xs_i_0[i](2)), xs_i_0[i](3) * sin(xs_i_0[i](2)); 

						// Single-point initialization
						//xs_tr_i_0[i] << measurement_i[i].col(0), 0, 0;

						tr_trajectory_i[i].resize(8, N);	tr_trajectory_covariances_i[i].resize(32, N);	tr_trajectory_covariances_i[i].block<16, 1>(0, 0) = flatten(P_0);

						kf_i[i] = KF(xs_tr_i_0[i], P_0, ID[i], dt_KF, 0.0, Q_KF, R_KF);
					}
				}
				//============================================
				// Combined Crossing and overtaking scenario, 
				// conservative KF
				//============================================
				else if(scenario_type(s) == 3)
				{
					n_obst_s = 2;	// Number of obstacles in this scenario
					n_wps_i.resize(n_obst_s);
					obstacle_states.resize(9, n_obst_s);	
					obstacle_covariances.resize(16, n_obst_s);	
					obstacle_intention_probabilities.resize(3, n_obst_s);
					obstacle_a_priori_CC_probabilities.resize(n_obst_s);

					// Ownship data initialization
					xs_os_0 << 200, 0, 0, 6, 0, 0;
					trajectory.col(0) = xs_os_0;	trajectory_pr = trajectory;

					u_d = 6.0; u_d_pr = u_d;

					// Obstacle data initialization
					P_0 << 	25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0.025, 0,
							0, 0, 0, 0.025;

					// Measurement generation covariance
					R_mg << 25, 0,
							0, 25;
					L = R_mg.llt().matrixL();

					// KF tuning matrices
					sigma_a =  0.5;		dt_KF = 2.5;
					R_KF << 25, 0, 0, 0,
							0, 25, 0, 0,
							0, 0, 0, 0,
							0, 0, 0, 0;
					R_KF = 5 * R_KF;
					Q_KF << pow(dt_KF, 3) / 3.0, 			0, 				pow(dt_KF, 2) / 2.0, 			0,
									0,		  		pow(dt_KF, 3) / 3, 				0, 					pow(dt_KF, 2) / 2.0,
							pow(dt_KF, 2) / 2.0, 			0, 					dt_KF, 						0,
									0, 				pow(dt_KF, 2) / 2.0, 			0, 						dt_KF;
					Q_KF = sigma_a * Q_KF;

					// Simulate obstacles using a simple kinematic model based ship
					T_U = 10; T_chi = 7.5; R_a = 30.0; LOS_LD = 200.0;
					obstacle_ship = Obstacle_Ship(T_U, T_chi, R_a, LOS_LD, 0); // xs = [x, y, chi, U]^T for this ship
					for (int i = 0; i < n_obst; i++)
					{
						ID[i] = i;

						aux_data_i.block<16, 1>(0, i) = flatten(R_KF);
						aux_data_i.block<16, 1>(16, i) = flatten(Q_KF);

						// --GT initialization--
						if (i == 0)
						{
							// desired course will be set by the guidance in the obstacle ship prediction
							u_d_i[i] = 12.0; chi_d_i[i] = 0.0;
							xs_i_0[i] << 0, 0, 0 * DEG2RAD, u_d_i[i];

							n_wps_i[i] = 6;
							waypoints_i[i].resize(2, n_wps_i[i]); 
							waypoints_i[i] << 	xs_i_0[i](0), 100, 200, 400, 500, 9000, 
												xs_i_0[i](1), 45, 60, 50,  0,	0;
						}
						else if(i == 1)
						{
							// desired course will be set by the guidance in the obstacle ship prediction
							u_d_i[i] = 6.0; chi_d_i[i] = 0.0;
							xs_i_0[i] << 400, -300, 90 * DEG2RAD, u_d_i[i];
							n_wps_i[i] = 5;
							waypoints_i[i].resize(2, n_wps_i[i]); 
							waypoints_i[i] << 	xs_i_0[i](0), xs_i_0[i](0), 400, xs_i_0[i](0), xs_i_0[i](0),
												xs_i_0[i](1), -150, 0, 100, 300;
						}
						
						gt_trajectory_i[i].resize(4, N);	gt_trajectory_i[i].col(0) = xs_i_0[i];
						//--

						offset_sequence_i[i].resize(6);		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

						maneuver_times_i[i].resize(3);	maneuver_times_i[i] << 0, 100, 150;

						// Simulate obstacle trajectory independent on the ownship
						obstacle_ship.predict_trajectory(gt_trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], ERK1, LOS, T_sim, dt);

						Pr_a[i].resize(3, N);
						Pr_CC[i].resize(N);
						
						measurement_i[i].resize(2, N);
						for (int k = 0; k < N; k++)
						{	
							if (i == 0)
							{
								//Pr_a[i](0, k) = 0.1; Pr_a[i](1, k) = 0.45; Pr_a[i](2, k) = 0.45;
								Pr_a[i](0, k) = 1; Pr_a[i](1, k) = 1; Pr_a[i](2, k) = 1;
								Pr_CC[i](k) = 0.5;
							}
							else if (i == 1)
							{
								Pr_a[i](0, k) = 1; Pr_a[i](1, k) = 1; Pr_a[i](2, k) = 1;
								Pr_CC[i](k) = 0.5;
							}
							
							Pr_a[i].col(k) = Pr_a[i].col(k) / Pr_a[i].col(k).sum();
							

							// Generate measurement vector for obstacle i by adding measurement noise to the trajectory, with covariance R_cov
							z(0) = std_norm_pdf(eng1); z(1) = std_norm_pdf(eng1);
							measurement_i[i].col(k) = gt_trajectory_i[i].block<2, 1>(0, k) + L * z;
						}
						// GT initialization
						xs_tr_i_0[i] << xs_i_0[i].block<2, 1>(0, 0), xs_i_0[i](3) * cos(xs_i_0[i](2)), xs_i_0[i](3) * sin(xs_i_0[i](2)); 

						// Single-point initialization
						//xs_tr_i_0[i] << measurement_i[i].col(0), 0, 0;

						tr_trajectory_i[i].resize(8, N);	tr_trajectory_covariances_i[i].resize(32, N);	tr_trajectory_covariances_i[i].block<16, 1>(0, 0) = flatten(P_0);

						kf_i[i] = KF(xs_tr_i_0[i], P_0, ID[i], dt_KF, 0.0, Q_KF, R_KF);
					}
				}
				
				
				//=========================================================

				//=========================================================
				// Initialize plotting for an MC run
				//=========================================================
				n_obst_mx = mxCreateDoubleScalar(n_obst_s);
				n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);
				
				map_static_obst = static_obstacles;

				for (int i = 0; i < n_obst_s; i++)
				{
					wps_i_mx[i] = mxCreateDoubleMatrix(2, n_wps_i[i], mxREAL);

					y_m_i_mx[i] = mxCreateDoubleMatrix(2, N, mxREAL);
					gt_traj_i_mx[i] = mxCreateDoubleMatrix(4, N, mxREAL);
					tr_traj_i_mx[i] = mxCreateDoubleMatrix(8, N, mxREAL);
					tr_P_traj_i_mx[i] = mxCreateDoubleMatrix(32, N, mxREAL);

					Pr_a_i_mx[i] = mxCreateDoubleMatrix(3, N, mxREAL);
					Pr_CC_i_mx[i] = mxCreateDoubleMatrix(1, N, mxREAL);
				}

				engPutVariable(ep, "X_static", static_obst_mx);
				engPutVariable(ep, "n_static_obst", n_static_obst_mx);
				engPutVariable(ep, "n_obst", n_obst_mx);
				
				engPutVariable(ep, "WPs", wps_os_mx);

				engEvalString(ep, "init_psbmpc_plotting_for_paper");

				for (int i = 0; i < n_obst_s; i++)
				{
					p_wps_i = mxGetPr(wps_i_mx[i]);

					new (&map_wps_i) Eigen::Map<Eigen::MatrixXd>(p_wps_i, 2, n_wps_i[i]);
					map_wps_i = waypoints_i[i];

					engPutVariable(ep, "WPs_i", wps_i_mx[i]);

					i_mx = mxCreateDoubleScalar(i + 1);
					engPutVariable(ep, "i", i_mx);

					/* p_gt_traj_i = mxGetPr(gt_traj_i_mx[i]);
					new (&map_gt_traj_i) Eigen::Map<Eigen::MatrixXd>(p_gt_traj_i, 4, N);
					map_gt_traj_i = gt_trajectory_i[i];
					
					engPutVariable(ep, "X_i", gt_traj_i_mx[i]); */

					engEvalString(ep, "init_obstacle_plotting_for_paper");
				}
				//=========================================================

				//=========================================================
				// Simulate MCS Run of Scenario
				//=========================================================
				for (int k = 0; k < N; k++)
				{
					t = k * dt;

					for (int i = 0; i < n_obst_s; i++)
					{
						tr_trajectory_i[i].block<4, 1>(0, k) = kf_i[i].get_predicted_state();
						tr_trajectory_covariances_i[i].block<16, 1>(0, k) = flatten(kf_i[i].get_predicted_covariance());

						/* std::cout << "xs_i_p = " << tr_trajectory_i[i].block<4, 1>(0, k).transpose() << std::endl;
						std::cout << "P_i_p : " << std::endl;
						std::cout << reshape(tr_trajectory_covariances_i[i].block<16, 1>(0, k), 4, 4) << std::endl; */

						// Track Obstacles using KF's, with measurement every dt_KF = 2.5 s (i.e  approx 0.4 Hz)
						if (fmod(t, dt_KF) == 0)
						{
							kf_i[i].update(measurement_i[i].col(k), dt_KF, false);
							tr_trajectory_i[i].block<4, 1>(4, k) = kf_i[i].get_state();
							tr_trajectory_covariances_i[i].block<16, 1>(16, k) = flatten(kf_i[i].get_covariance());
						}
						else // dead reckoning
						{
							kf_i[i].update(measurement_i[i].col(k), dt_KF, true); // measurement is now a dont-care argument
							tr_trajectory_i[i].block<4, 1>(4, k) = kf_i[i].get_predicted_state();
							tr_trajectory_covariances_i[i].block<16, 1>(16, k) = flatten(kf_i[i].get_predicted_covariance());
						}
						
						kf_i[i].predict(dt);
						
						
						/* std::cout << "xs_i_upd = " << tr_trajectory_i[i].block<4, 1>(4, k).transpose() << std::endl;
						std::cout << "P_i_upd : " << std::endl;
						std::cout << reshape(tr_trajectory_covariances_i[i].block<16, 1>(16, k), 4, 4) << std::endl; */

						//then aquire obstacle information for the COLAV systems
						/* xs_i_k.block<2, 1>(0, 0) = gt_trajectory_i[i].block<2, 1>(0, k);
						xs_i_k(2) = gt_trajectory_i[i](3) * cos(gt_trajectory_i[i](2));
						xs_i_k(3) = gt_trajectory_i[i](3) * sin(gt_trajectory_i[i](2)); */
						xs_i_k = tr_trajectory_i[i].block<4, 1>(4, k);
						obstacle_states.col(i) << xs_i_k, A, B, C, D, ID[i];

						obstacle_covariances.col(i) = flatten(P_0);

						obstacle_intention_probabilities.col(i) = Pr_a[i].col(k);
						obstacle_a_priori_CC_probabilities(i) = Pr_CC[i](k);
					}
					
					//============================================================================================
					// COLAV using SB-MPC
					//============================================================================================
					sbmpc_obstacle_manager.operator()<SBMPC_Parameters>(
						sbmpc.pars, 
						trajectory.col(k), 
						os.get_length(),
						obstacle_states, 
						obstacle_covariances, 
						obstacle_intention_probabilities, 
						obstacle_a_priori_CC_probabilities);

					os.update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, LOS);

					if (fmod(t, 5) == 0)
					{
						start = std::chrono::system_clock::now();		

						sbmpc.calculate_optimal_offsets(
							u_opt,
							chi_opt, 
							predicted_trajectory,
							u_d,
							chi_d,
							waypoints,
							trajectory.col(k),
							static_obstacles,
							sbmpc_obstacle_manager.get_data());

						end = std::chrono::system_clock::now();
						elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

						mean_t = elapsed.count();

						std::cout << "SBMPC time usage : " << mean_t << " milliseconds" << std::endl;

						sbmpc_obstacle_manager.update_obstacle_status(trajectory.col(k));
						//sbmpc_obstacle_manager.display_obstacle_information();
					
					}
					u_c = u_d * u_opt; chi_c = chi_d + chi_opt;
					os.update_ctrl_input(u_c, chi_c, trajectory.col(k));
					
					if (k < N - 1) { trajectory.col(k + 1) = os.predict(trajectory.col(k), dt, ERK1); }

					aux_data_os(0, k) = u_d; aux_data_os(1, k) = u_opt; aux_data_os(2, k) = chi_d; aux_data_os(3, k) = chi_opt;
					//============================================================================================
					// COLAV using PSB-MPC
					//============================================================================================
					psbmpc_obstacle_manager.operator()<PSBMPC_Parameters>(
						psbmpc.pars, 
						trajectory_pr.col(k), 
						os_pr.get_length(),
						obstacle_states, 
						obstacle_covariances, 
						obstacle_intention_probabilities, 
						obstacle_a_priori_CC_probabilities);

					os_pr.update_guidance_references(u_d_pr, chi_d_pr, waypoints, trajectory_pr.col(k), dt, LOS);

					if (fmod(t, 5) == 0)
					{
						start = std::chrono::system_clock::now();		

						psbmpc.calculate_optimal_offsets(
							u_opt_pr,
							chi_opt_pr, 
							predicted_trajectory_pr,
							u_d_pr,
							chi_d_pr,
							waypoints,
							trajectory_pr.col(k),
							static_obstacles,
							psbmpc_obstacle_manager.get_data());

						end = std::chrono::system_clock::now();
						elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

						mean_t = elapsed.count();

						std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;

						psbmpc_obstacle_manager.update_obstacle_status(trajectory.col(k));
						//psbmpc_obstacle_manager.display_obstacle_information();
					
					}
					u_c_pr = u_d_pr * u_opt_pr; chi_c_pr = chi_d_pr + chi_opt_pr;
					os_pr.update_ctrl_input(u_c_pr, chi_c_pr, trajectory_pr.col(k));
					
					if (k < N - 1) { trajectory_pr.col(k + 1) = os_pr.predict(trajectory_pr.col(k), dt, ERK1); }

					aux_data_os_pr(0, k) = u_d_pr; aux_data_os_pr(1, k) = u_opt_pr; aux_data_os_pr(2, k) = chi_d_pr; aux_data_os_pr(3, k) = chi_opt_pr;
					//============================================================================================
					// Send data to matlab for live plotting
					//============================================================================================
					buffer[BUFFSIZE] = '\0';
					engOutputBuffer(ep, buffer, BUFFSIZE);

					k_s_mx = mxCreateDoubleScalar(k + 1);
					engPutVariable(ep, "k", k_s_mx);
					
					pred_traj_os_mx = mxCreateDoubleMatrix(predicted_trajectory.rows(), predicted_trajectory.cols(), mxREAL);
					pred_traj_os_pr_mx = mxCreateDoubleMatrix(predicted_trajectory_pr.rows(), predicted_trajectory_pr.cols(), mxREAL);

					p_pred_traj_os = mxGetPr(pred_traj_os_mx);
					p_pred_traj_os_pr = mxGetPr(pred_traj_os_pr_mx);

					new (&map_pred_traj_os) Eigen::Map<Eigen::MatrixXd>(p_pred_traj_os, 2, predicted_trajectory.cols());
					new (&map_pred_traj_os_pr) Eigen::Map<Eigen::MatrixXd>(p_pred_traj_os_pr, 2, predicted_trajectory_pr.cols());
					map_pred_traj_os = predicted_trajectory;
					map_pred_traj_os_pr = predicted_trajectory_pr;

					new (&map_traj_os) Eigen::Map<Eigen::MatrixXd>(p_traj_os, 6, N);
					new (&map_traj_os_pr) Eigen::Map<Eigen::MatrixXd>(p_traj_os_pr, 6, N);
					map_traj_os = trajectory;
					map_traj_os_pr = trajectory_pr;

					engPutVariable(ep, "X_pred", pred_traj_os_mx);
					engPutVariable(ep, "X_pred_pr", pred_traj_os_pr_mx);
					engPutVariable(ep, "X", traj_os_mx);
					engPutVariable(ep, "X_pr", traj_os_pr_mx);

					engEvalString(ep, "update_ownship_plot_for_paper");

					for(int i = 0; i < n_obst_s; i++)
					{
						p_gt_traj_i = mxGetPr(gt_traj_i_mx[i]);
						p_tr_traj_i = mxGetPr(tr_traj_i_mx[i]);
						p_tr_P_traj_i = mxGetPr(tr_P_traj_i_mx[i]);

						new (&map_gt_traj_i) Eigen::Map<Eigen::MatrixXd>(p_gt_traj_i, 4, N);
						new (&map_tr_traj_i) Eigen::Map<Eigen::MatrixXd>(p_tr_traj_i, 8, N);
						new (&map_tr_P_traj_i) Eigen::Map<Eigen::MatrixXd>(p_tr_P_traj_i, 32, N);
						
						map_gt_traj_i = gt_trajectory_i[i];
						map_tr_traj_i = tr_trajectory_i[i];
						map_tr_P_traj_i = tr_trajectory_covariances_i[i];
						
						engPutVariable(ep, "X_i", gt_traj_i_mx[i]);
						engPutVariable(ep, "X_i_tr", tr_traj_i_mx[i]);
						engPutVariable(ep, "P_i_tr", tr_P_traj_i_mx[i]);

						i_mx = mxCreateDoubleScalar(i + 1);
						engPutVariable(ep, "i", i_mx);

						engEvalString(ep, "update_obstacle_plot_for_paper");

						printf("%s", buffer);
					}
					//============================================================================================
					
				}
				//============================================================================================
				// Store data for one Monte Carlo simulation
				//============================================================================================
				map_aux_data_os = aux_data_os;
				map_aux_data_os_pr = aux_data_os_pr;

				map_R_mg = R_mg;
				map_aux_data_i = aux_data_i;
				engPutVariable(ep, "R_mg", R_mg_mx);
				engPutVariable(ep, "aux_data_i", aux_data_i_mx);
				
				engPutVariable(ep, "X_pred", pred_traj_os_mx);
				engPutVariable(ep, "X_pred_pr", pred_traj_os_pr_mx);
				engPutVariable(ep, "X", traj_os_mx);
				engPutVariable(ep, "X_pr", traj_os_pr_mx);

				engPutVariable(ep, "aux_data_os", aux_data_os_mx);
				engPutVariable(ep, "aux_data_os_pr", aux_data_os_pr_mx);

				engEvalString(ep, "store_ownship_data_for_paper");

				for (int i = 0; i < n_obst_s; i++)
				{
					p_Pr_a_i = mxGetPr(Pr_a_i_mx[i]);
					p_Pr_CC_i = mxGetPr(Pr_CC_i_mx[i]);

					new (&map_Pr_a_i) Eigen::Map<Eigen::MatrixXd>(p_Pr_a_i, 3, N);
					map_Pr_a_i = Pr_a[i];

					new (&map_Pr_CC_i) Eigen::Map<Eigen::Matrix<double, 1, -1>>(p_Pr_CC_i, 1, N);
					map_Pr_CC_i = Pr_CC[i];

					p_y_m_i = mxGetPr(y_m_i_mx[i]);
					new (&map_y_m_i) Eigen::Map<Eigen::MatrixXd>(p_y_m_i, 2, N);
					map_y_m_i = measurement_i[i];

					engPutVariable(ep, "Pr_a_i", Pr_a_i_mx[i]);
					engPutVariable(ep, "Pr_CC_i", Pr_CC_i_mx[i]);

					engPutVariable(ep, "y_m_i", y_m_i_mx[i]);
					engPutVariable(ep, "X_i", gt_traj_i_mx[i]);
					engPutVariable(ep, "X_i_tr", tr_traj_i_mx[i]);
					engPutVariable(ep, "P_i_tr", tr_P_traj_i_mx[i]);

					i_mx = mxCreateDoubleScalar(i + 1);
					engPutVariable(ep, "i", i_mx);

					engEvalString(ep, "store_obstacle_data_for_paper");
				}
				//============================================================================================

				//============================================================================================
				// Clean up MC run matlab arrays
				//============================================================================================			
				mxDestroyArray(pred_traj_os_mx);
				mxDestroyArray(pred_traj_os_pr_mx);

				mxDestroyArray(n_obst_mx);
				mxDestroyArray(n_static_obst_mx);
				for (int i = 0; i < n_obst_s; i++)
				{
					mxDestroyArray(Pr_a_i_mx[i]);
					mxDestroyArray(Pr_CC_i_mx[i]);
					mxDestroyArray(y_m_i_mx[i]);
					mxDestroyArray(gt_traj_i_mx[i]);
					mxDestroyArray(tr_traj_i_mx[i]);
					mxDestroyArray(tr_P_traj_i_mx[i]);
					mxDestroyArray(wps_i_mx[i]);
				}
			}
		}
	
	
	}
	//============================================================================================



	//============================================================================================
	// Clean up simulation variables being persistent, plot results and close matlab engine
	//============================================================================================
	mxDestroyArray(aux_data_os_mx);
	mxDestroyArray(aux_data_os_pr_mx);
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(traj_os_pr_mx);
	mxDestroyArray(wps_os_mx);

	mxDestroyArray(aux_data_i_mx);
	mxDestroyArray(static_obst_mx);

	mxDestroyArray(R_mg_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(dt_sim_mx);
	
	mxDestroyArray(i_mx);
	mxDestroyArray(d_safe_mx);
	mxDestroyArray(k_pt_mx);
	mxDestroyArray(k_MC_mx);
	mxDestroyArray(k_s_mx);

	engEvalString(ep, "plot_multiple_results_paper");
	engClose(ep);  

	return 0;
}