#include "cpu/psbmpc_cpu.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <engine.h>

#define BUFSIZE 1000000

int main()
{
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE + 1];

	double T = 110;
	double dt = 0.5;
	int n_samples = std::round(T / dt);

	Eigen::Vector4d xs_i_0;
	xs_i_0 << 0, 0, 10, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 0.025, 0,
		0, 0, 0, 0.025;

	Eigen::MatrixXd waypoints_i(2, 2), trajectory_i(4, n_samples);
	waypoints_i << xs_i_0(0), 1000,
		xs_i_0(1), 0;

	trajectory_i(0, 0) = xs_i_0(0);
	trajectory_i(1, 0) = xs_i_0(1);
	trajectory_i(2, 0) = 0.0;
	trajectory_i(3, 0) = xs_i_0(2);

	Eigen::VectorXd maneuver_times(2), offset_sequence(4);
	maneuver_times << 0, 50;
	offset_sequence << 1.0, -90 * DEG2RAD, 1.0, 90 * DEG2RAD;

	PSBMPC_LIB::CPU::Obstacle_Ship oship;
	oship.predict_trajectory(trajectory_i, offset_sequence, maneuver_times, xs_i_0(2), xs_i_0(3), waypoints_i, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T, dt);

	Eigen::VectorXd xs_i_aug(9);
	xs_i_aug << xs_i_0, 5, 5, 5, 5, 0;

	Eigen::VectorXd Pr_s(3);
	Pr_s << 1, 1, 1;
	Pr_s / Pr_s.sum();

	Eigen::VectorXd ownship_state(4);
	ownship_state << 500, 0, 180 * DEG2RAD, 9.0;

	PSBMPC_LIB::Dynamic_Obstacles obstacles;
	obstacles.push_back(PSBMPC_LIB::Tracked_Obstacle(xs_i_aug, PSBMPC_LIB::CPU::flatten(P_0), Pr_s, false, T, dt));

	PSBMPC_LIB::CPU::PSBMPC psbmpc;
	PSBMPC_LIB::Obstacle_Predictor obstacle_predictor;

	//==============================================
	// Matlab plotting init
	//==============================================
	std::vector<Eigen::MatrixXd> v_p(1);
	v_p[0].resize(2, n_samples);

	mxArray *wps_i_mx = mxCreateDoubleMatrix(2, 2, mxREAL);
	mxArray *traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *pred_traj_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *v_traj_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_wps_i = mxGetPr(wps_i_mx);
	double *p_traj = mxGetPr(traj_mx);
	double *p_pred_traj = mxGetPr(pred_traj_mx);
	double *p_v_traj = mxGetPr(v_traj_mx);
	double *p_P_traj = mxGetPr(P_traj_mx);
	mxArray *ps_mx(nullptr), *k_mx(nullptr), *k_p_mx(nullptr), *n_ps_mx(nullptr);
	mxArray *T_sim_mx = mxCreateDoubleScalar(T), *dt_sim_mx = mxCreateDoubleScalar(dt);

	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, 2);
	Eigen::Map<Eigen::MatrixXd> map_traj(p_traj, 4, n_samples);

	map_traj = trajectory_i;
	map_wps_i = waypoints_i;

	n_ps_mx = mxCreateDoubleScalar(obstacle_predictor.get_n_ps_LOS());

	engPutVariable(ep, "X_i", traj_mx);
	engPutVariable(ep, "wps_i", wps_i_mx);
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);
	engEvalString(ep, "test_obstacle_predictor_plot_init");

	std::vector<Eigen::MatrixXd> xs_p;
	Eigen::MatrixXd P_p;
	for (int k = 0; k < n_samples; k++)
	{
		xs_i_aug.block<2, 1>(0, 0) = trajectory_i.block<2, 1>(0, k);
		xs_i_aug(2) = trajectory_i(3, k) * cos(trajectory_i(2, k));
		xs_i_aug(3) = trajectory_i(3, k) * sin(trajectory_i(2, k));
		obstacles[0].update(xs_i_aug, PSBMPC_LIB::CPU::flatten(P_0), Pr_s, false, dt);

		if (k % 20 == 0)
		{
			std::cout << "xs_i = " << xs_i_aug.block<4, 1>(0, 0).transpose() << std::endl;
			//==============================================
			// Setup and predict using the developed class
			//==============================================
			obstacle_predictor(obstacles, ownship_state, psbmpc.pars);
			//==============================================
			xs_p = obstacles[0].get_trajectories();
			P_p = obstacles[0].get_trajectory_covariance();
			int n_ps = xs_p.size();

			k_mx = mxCreateDoubleScalar(k + 1);
			engPutVariable(ep, "k", k_mx);

			engEvalString(ep, "test_obstacle_predictor_plot_update_1");

			for (int k_p = 0; k_p < n_samples; k_p += 10)
			{
				k_p_mx = mxCreateDoubleScalar(k_p + 1);
				engPutVariable(ep, "k_p", k_p_mx);
				for (int ps = 0; ps < n_ps; ps++)
				{
					ps_mx = mxCreateDoubleScalar(ps + 1);
					engPutVariable(ep, "ps", ps_mx);

					Eigen::Map<Eigen::MatrixXd> map_pred_traj(p_pred_traj, xs_p[ps].rows(), xs_p[ps].cols());
					Eigen::Map<Eigen::MatrixXd> map_v_traj(p_v_traj, v_p[0].rows(), v_p[0].cols());
					Eigen::Map<Eigen::MatrixXd> map_P_traj(p_P_traj, P_p.rows(), P_p.cols());

					map_pred_traj = xs_p[ps];
					map_v_traj = v_p[0];
					map_P_traj = P_p;

					buffer[BUFSIZE] = '\0';
					engOutputBuffer(ep, buffer, BUFSIZE);

					engPutVariable(ep, "X_i_ps", pred_traj_mx);
					engPutVariable(ep, "v_i_ps", v_traj_mx);
					engPutVariable(ep, "P_i_ps", P_traj_mx);
					engEvalString(ep, "test_obstacle_predictor_plot_update_2");

					printf("%s", buffer);
				}
			}
		}
	}

	mxDestroyArray(ps_mx);
	mxDestroyArray(k_mx);
	mxDestroyArray(k_p_mx);
	mxDestroyArray(n_ps_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(dt_sim_mx);
	mxDestroyArray(wps_i_mx);
	mxDestroyArray(traj_mx);
	mxDestroyArray(pred_traj_mx);
	mxDestroyArray(v_traj_mx);
	mxDestroyArray(P_traj_mx);
	engClose(ep);

	return 0;
}