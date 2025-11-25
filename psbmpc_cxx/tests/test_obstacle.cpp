#include "tracked_obstacle.hpp"
#include "sbmpc.hpp"
#if OWNSHIP_TYPE == 0
	#include "cpu/kinematic_ship_models_cpu.hpp"
#else 
	#include "cpu/kinetic_ship_models_cpu.hpp"
#endif
#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <engine.h>

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

	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 6, 0, 0;

	double T = 200; double dt = 0.5;
	int n_samples = std::round(T / dt);
	double u_d = 6.0; double chi_d = 0.0;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;

	Eigen::MatrixXd trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	#if OWNSHIP_TYPE == 0
		trajectory.resize(4, n_samples);
		trajectory.col(0) = xs_os_0.block<4, 1>(0, 0);
	#else
		trajectory.resize(6, n_samples);
		trajectory.col(0) = xs_os_0;
	#endif

	waypoints.resize(2, 2); 
	//waypoints << 0, 200, 200, 0,    0, 300, 1000,
	//			 0, -50,  -200, -200,  0, 300, 0;
	waypoints << 0, 1000,
				 0, 0;

	std::unique_ptr<PSBMPC_LIB::CPU::Ownship> ownship(new PSBMPC_LIB::CPU::Ownship());
	
	//*****************************************************************************************************************
	// Obstacle setup
	//*****************************************************************************************************************

	Eigen::VectorXd xs_aug(9);
	xs_aug << 1000, 0, -5, 0, 5, 5, 5, 5, 0;

	Eigen::MatrixXd P(4, 4);
	P << 100, 0, 0, 0,
	     0, 100, 0, 0,
		 0, 0, 0.025, 0,
		 0, 0, 0, 0.025;

	Eigen::VectorXd Pr_a(3);
	Pr_a << 1, 1, 1;
	Pr_a = Pr_a / Pr_a.sum();

	double Pr_CC = 0.9;

	bool filter_on = true;

	std::unique_ptr<PSBMPC_LIB::Tracked_Obstacle> obstacle(new PSBMPC_LIB::Tracked_Obstacle(xs_aug, PSBMPC_LIB::CPU::flatten(P), Pr_a, Pr_CC, filter_on, T, dt));

	//*****************************************************************************************************************
	// Test obstacle functionality
	//*****************************************************************************************************************
	PSBMPC_LIB::SBMPC sbmpc;
	
	int n_ps = 5; 	// KCC, two alternative maneuvers to starboard and port, only one course change of 45 deg
	std::vector<PSBMPC_LIB::Intention> ps_ordering;
	Eigen::VectorXd ps_course_changes, ps_weights, ps_maneuver_times;
	ps_ordering.resize(n_ps); ps_course_changes.resize(n_ps);
	ps_weights.resize(n_ps); ps_maneuver_times.resize(n_ps);
	
	double Pr_CC_i_test = obstacle->get_a_priori_CC_probability();
	std::cout << "Pr_CC = " << Pr_CC_i_test << std::endl;

	for (int ps = 0; ps < n_ps; ps++)
	{
		if (ps == 0) { ps_ordering[ps] = PSBMPC_LIB::KCC; }
		else if (ps > 0 && ps < (n_ps - 1) / 2 + 1) { ps_ordering[ps] = PSBMPC_LIB::SM; }
		else { ps_ordering[ps] = PSBMPC_LIB::PM; }
	}
	// Lets say a head-on situation, assign weights accordingly:
	ps_maneuver_times << 0, 0, 100, 0, 100;
	ps_course_changes << 0, 45 * DEG2RAD, 45 * DEG2RAD, - 45 * DEG2RAD, - 45 * DEG2RAD;
	ps_weights << (1 - Pr_CC_i_test), Pr_CC_i_test, Pr_CC_i_test, (1 - Pr_CC_i_test), (1 - Pr_CC_i_test);

	ownship->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T, dt);

	obstacle->initialize_independent_prediction(ps_ordering, ps_course_changes, ps_maneuver_times);

	obstacle->predict_independent_trajectories(T, dt, trajectory.col(0), sbmpc);

	std::vector<Eigen::MatrixXd> xs_p = obstacle->get_trajectories();

	Eigen::MatrixXd P_p = obstacle->get_trajectory_covariance();

	std::vector<bool> mu = obstacle->get_COLREGS_violation_indicator();

	// Update tracked duration, input new information, print it.
	Eigen::VectorXd xs_aug_new(9);
	xs_aug_new << 900, 0, -6, 0, 5, 5, 5, 5, 0;

	Eigen::MatrixXd P_new(4, 4);
	P_new << 125, 0, 0, 0,
	     0, 125, 0, 0,
		 0, 0, 0.05, 0,
		 0, 0, 0, 0.05;

	Eigen::VectorXd Pr_a_new(3);
	Pr_a_new << 0.1, 0.5, 0.4;

	double Pr_CC_new = 0.1;

	obstacle->increment_duration_tracked(dt);

	obstacle->update(xs_aug_new, PSBMPC_LIB::CPU::flatten(P_new), Pr_a_new, Pr_CC_new, !filter_on, dt);

	std::cout << "xs_upd = " << obstacle->kf->get_state().transpose() << std::endl;

	std::cout << "P_upd = " << std::endl;
	std::cout << obstacle->kf->get_covariance() << std::endl;

	std::cout << "Pr_CC_new = " << obstacle->get_a_priori_CC_probability() << std::endl;

	std::cout << "Pr_a_new = " << obstacle->get_intention_probabilities().transpose() << std::endl;

	std::cout << "Duration tracked = " << obstacle->get_duration_tracked() << std::endl;

	std::cout << "Duration lost = " << obstacle->get_duration_lost() << std::endl;
	
	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
	mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *p_traj_os_mx = mxGetPr(traj_os_mx);
	double *p_wps_mx = mxGetPr(wps_mx);

	mxArray *traj_i_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *P_traj_i_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_traj_i_mx = mxGetPr(traj_i_mx);
	double *p_P_traj_i_mx = mxGetPr(P_traj_i_mx);

	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os_mx, trajectory.rows(), n_samples);
	map_traj_os = trajectory;
	
	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_mx, 2, 2);
	map_wps = waypoints;
	
	Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i_mx, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i_mx, 16, n_samples);

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "WPs", wps_mx);

	engEvalString(ep, "test_ownship_plot");

	mxArray *ps_count(nullptr);

	map_P_traj_i = P_p;
	engPutVariable(ep, "P_i_flat", P_traj_i_mx);

	for (int ps = 0; ps < n_ps; ps++)
	{
		ps_count = mxCreateDoubleScalar(ps + 1);

		std::cout << "Obstacle breaches COLREGS in prediction scenario " << ps << " ? " << mu[ps] << std::endl;
		std::cout << xs_p[ps].col(300).transpose() << std::endl;
		map_traj_i = xs_p[ps];
		engPutVariable(ep, "X_i", traj_i_mx);

		engPutVariable(ep, "ps_counter", ps_count);

		engEvalString(ep, "test_obstacle_plot");

		printf("%s", buffer);
	}
	
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_mx);

	mxDestroyArray(traj_i_mx);
	mxDestroyArray(P_traj_i_mx);
	mxDestroyArray(ps_count);

	engClose(ep); 

	return 0;
}