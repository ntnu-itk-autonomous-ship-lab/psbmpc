#include "cpu/psbmpc_cpu.hpp"
#if USE_GPU_PSBMPC
#include "gpu/psbmpc_gpu.cuh"
#endif
#include "cpu/utilities_cpu.hpp"
#include "grounding_hazard_manager.hpp"

#include <Eigen/Dense>
#if ENABLE_TEST_FILE_PLOTTING
#include <engine.h>
#endif
#include <iostream>
#include <limits>
#include <vector>
#include <chrono>
#include <memory>
#include <string>

#define BUFSIZE 1000000

//*****************************************************************************************************************
// Main program:
//*****************************************************************************************************************
int main()
{
	// std::cout << std::numeric_limits<long double>::digits10 << std::endl;
	//*****************************************************************************************************************
	//  Simulation setup
	//*****************************************************************************************************************
	double T_sim = 500;
	double dt = 0.5;
	int N = std::round(T_sim / dt);
	//*****************************************************************************************************************
	// Own-ship sim setup
	//*****************************************************************************************************************
	double offset = 0;
	/*coordinates are given in wgs-84 use https://finnposisjon.test.geonorge.no/ */
	Eigen::Matrix<double, 6, 1> xs_os_0;
	// xs_os_0 << 7042320, 269475, 180 * DEG2RAD, 1, 0, 0; // utforbi skansen
	// xs_os_0 << 7042020, 269575, 130 * DEG2RAD, 1.5, 0, 0; // "i" skansen
	/* xs_os_0 << 7042220, 270175, 60 * DEG2RAD, 1.5, 0, 0; // rett nordvest for ravnkloa
	double u_d = 1.5, chi_d, u_c, chi_c; */

	/* xs_os_0 << 280.13 + offset, 960.02 + offset, 23 * DEG2RAD, 1.0, 0, 0; */
	/* xs_os_0 << 456.33, 1295.52, -105.0 * DEG2RAD, 0.0, 0, 0; */
	/* xs_os_0 << 354.0, 1176.0, 0.862, 0.0, 0, 0; */
	xs_os_0 << 250.0, 1190.0, 0.306, 0.0, 0, 0;
	double u_d(1.0), chi_d(0.0), u_c(0.0), chi_c(0.0);

	PSBMPC_LIB::CPU::Ownship ownship;

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

	waypoints << xs_os_0(0), 462.0,
		xs_os_0(1), 1257.0;

	/* waypoints << 280.13 + offset, 477.50 + offset,
		960.02 + offset, 1032.11 + offset; */

	/* waypoints << 280.13 + offset, 477.50 + offset,
		960.02 + offset, 1032.11 + offset; */

	//*****************************************************************************************************************
	// Obstacle sim setup
	//*****************************************************************************************************************
	int n_do = 1;
	std::vector<int> ID(n_do);

	std::vector<Eigen::VectorXd> xs_i_0(n_do);

	// Use constant obstacle uncertainty throughout the simulation, for simplicity
	Eigen::MatrixXd P_0(4, 4);
	P_0 << 1.0, 0, 0, 0,
		0, 1.0, 0, 0,
		0, 0, 0.025, 0,
		0, 0, 0, 0.025;

	double A = 5, B = 5, C = 1.5, D = 1.5;

	// Use constant equal intention probability and a priori CC probability for simplicity
	std::vector<Eigen::VectorXd> Pr_a(n_do);

	std::vector<double> Pr_CC(n_do);

	// Simulate obstacles using an ownship model
	PSBMPC_LIB::CPU::Obstacle_Ship obstacle_sim;

	std::vector<double> u_d_i(n_do);
	std::vector<double> chi_d_i(n_do);

	std::vector<Eigen::VectorXd> offset_sequence_i(n_do);

	std::vector<Eigen::VectorXd> maneuver_times_i(n_do);

	std::vector<Eigen::MatrixXd> trajectory_i(n_do);
	std::vector<Eigen::Matrix<double, 16, -1>> trajectory_covariances_i(n_do);
	std::vector<Eigen::Matrix<double, 2, -1>> waypoints_i(n_do);
	std::vector<int> n_wps_i(n_do);

	//=====================================================================
	// Matlab array setup for the ownship and obstacle, ++
	//=====================================================================

#if ENABLE_TEST_FILE_PLOTTING
	// Matlab engine setup
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}

	char buffer[BUFSIZE + 1];
	mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), N, mxREAL);
	double *ptraj_os = mxGetPr(traj_os_mx);

	std::vector<mxArray *> traj_i_mx(n_do);
	std::vector<mxArray *> P_traj_i_mx(n_do);
	std::vector<mxArray *> wps_i_mx(n_do);

	double *ptraj_i;
	double *p_P_traj_i;
	double *p_wps_i;
#endif

	for (int i = 0; i < n_do; i++)
	{
		ID[i] = i;

		trajectory_covariances_i[i].resize(16, 1);
		trajectory_covariances_i[i].col(0) = PSBMPC_LIB::CPU::flatten(P_0);

		xs_i_0[i].resize(6);
		if (i == 1)
		{
			n_wps_i[i] = 3;
			waypoints_i[i].resize(2, n_wps_i[i]);
			// xs_i_0[i] << 5000, 0, 180 * DEG2RAD, 6, 0, 0;

			xs_i_0[i] << 281.06, 979.00, 23 * DEG2RAD, 2.0, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 377.99, 528.69,
				xs_i_0[i](1), 969.08, 1031.90;

			/* xs_i_0[i] << -224 + offset, -346 + offset, 0.978, 1.0, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), -191 + offset, -180 + offset, -149 + offset, -62 + offset,
				xs_i_0[i](1), -297 + offset, -235 + offset, -218 + offset, -140 + offset; */

			chi_d_i[i] = -90 * DEG2RAD;
			u_d_i[i] = 1.0;
		}
		else if (i == 2)
		{
			n_wps_i[i] = 2;
			waypoints_i[i].resize(2, n_wps_i[i]);
			xs_i_0[i] << 500, -300, 90 * DEG2RAD, 5, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 500,
				xs_i_0[i](1), 300;
			u_d_i[i] = 5.0;
			chi_d_i[i] = 90 * DEG2RAD;
		}
		else
		{
			n_wps_i[i] = 2;
			waypoints_i[i].resize(2, n_wps_i[i]);
			/* xs_i_0[i] << 300, 0, 180 * DEG2RAD, 1.5, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 0,
				xs_i_0[i](1), 0; */

			/* xs_i_0[i] << 493.48, 1031.90, -165 * DEG2RAD, 1.0, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 437.65, 347.55, 293.31,
				xs_i_0[i](1), 1040.24, 1028.86, 973.75; */

			/* xs_i_0[i] << 515.0, 1100.0, 134.0 * DEG2RAD, 1.5, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 300.0,
				xs_i_0[i](1), 1287.0; */

			xs_i_0[i] << 462.0, 1257.0, 3.4471, 1.0, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 250.0,
				xs_i_0[i](1), 1190.0;

			/* xs_i_0[i] << 477.02 + offset, 1032.36 + offset, -2.87973, 1.0, 0, 0;
			waypoints_i[i] << xs_i_0[i](0), 399.16 + offset, 340.55 + offset, 280.13 + offset,
				xs_i_0[i](1), 1040.24 + offset, 1024.49 + offset, 960.02 + offset; */
			u_d_i[i] = 1.0;
			chi_d_i[i] = 180 * DEG2RAD;
		}

#if OWNSHIP_TYPE == 0
		trajectory_i[i].resize(4, N);
		trajectory_i[i].block<2, 1>(0, 0) = xs_i_0[i].block<2, 1>(0, 0);
		trajectory_i[i](2, 0) = xs_i_0[i](2);
		trajectory_i[i](3) = xs_i_0[i].block<2, 1>(3, 0).norm();
#else
		trajectory_i[i].resize(6, N);
		trajectory_i[i].col(0) = xs_i_0[i];
#endif

		/* n_wps_i[i] = 4;
		waypoints_i[i].resize(2, n_wps_i[i]);
		waypoints_i[i] << xs_i_0[i](0), 7042340, 7042120, 7041990,
			xs_i_0[i](1), 270375, 269995, 269675; */

		offset_sequence_i[i].resize(6);
		offset_sequence_i[i] << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;

		maneuver_times_i[i].resize(3);
		maneuver_times_i[i] << 0, 100, 150;

		// Simulate obstacle trajectory independent on the ownship
		obstacle_sim.predict_trajectory(trajectory_i[i], offset_sequence_i[i], maneuver_times_i[i], u_d_i[i], chi_d_i[i], waypoints_i[i], PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T_sim, dt);
	}

	//*****************************************************************************************************************
	// PSB-MPC setup
	//*****************************************************************************************************************
#if (USE_GPU_PSBMPC == 1)
	PSBMPC_LIB::GPU::PSBMPC psbmpc;
#else
	PSBMPC_LIB::CPU::PSBMPC psbmpc;
#endif
	PSBMPC_LIB::Obstacle_Manager obstacle_manager;
	PSBMPC_LIB::Obstacle_Predictor obstacle_predictor(psbmpc.pars);

	double u_opt(u_d), chi_opt(0.0);

	double V_w = 0.0;
	Eigen::Vector2d wind_direction;
	wind_direction(0) = 1.0;
	wind_direction(1) = 0.0;

	Eigen::MatrixXd predicted_trajectory;

	Eigen::Matrix<double, 9, -1> obstacle_states;
	obstacle_states.resize(9, n_do);

	Eigen::Matrix<double, 16, -1> obstacle_covariances;
	obstacle_covariances.resize(16, n_do);

	PSBMPC_LIB::Static_Obstacles relevant_polygons;

	//*****************************************************************************************************************
	// Static Obstacles Setup
	//*****************************************************************************************************************
	char buffer1[256];
	char *val = getcwd(buffer1, sizeof(buffer1)); // either release or debug
	if (val)
	{
		std::cout << buffer1 << std::endl;
	}
	std::string relative_path = buffer1;

	// Input the path to the land data
	// std::string filename = "/tests/grounding_hazard_data/trondheim/old version data/charts/land/land.shp";
	double equatorial_radius(6378137.0), flattening_factor(0.003352810664747);
	int utm_zone(32);
	Eigen::Vector2d lla_origin;
	lla_origin << 63.4389029083, 10.39908278; // piren
	PSBMPC_LIB::Grounding_Hazard_Manager grounding_hazard_manager(
		// relative_path + "/../tests/grounding_hazard_data/trondheim/old version data/charts/land/land.shp", // IF RELEASE
		// relative_path + "/tests/grounding_hazard_data/trondheim/old version data/charts/land/land.shp", // IF DEBUG
		"",
		equatorial_radius,
		flattening_factor,
		utm_zone,
		true,
		lla_origin,
		"local_NED",
		psbmpc.pars);

	std::string other_polygons_filename = "../tests/grounding_hazard_data/piren_frame_psbmpc_polygons_trd.csv"; // IF RELEASE
	// std::string other_polygons_filename = "tests/grounding_hazard_data/piren_frame_psbmpc_polygons_trd.csv"; // IF DEBUG
	if (other_polygons_filename != "")
	{
		grounding_hazard_manager.read_other_polygons(other_polygons_filename, true, false);
	}

	PSBMPC_LIB::Static_Obstacles polygons_lla = grounding_hazard_manager.get_polygons_lla();
	PSBMPC_LIB::Static_Obstacles polygons = grounding_hazard_manager.get_polygons_ned();
	PSBMPC_LIB::Static_Obstacles simplified_polygons = grounding_hazard_manager.get_simplified_polygons_ned();
	PSBMPC_LIB::Static_Obstacles simplified_polygons_lla = grounding_hazard_manager.get_simplified_polygons_lla();

	// Make matlab polygons type friendly array:
	Eigen::Matrix<double, -1, 2> polygon_matrix_lla, polygon_matrix, simplified_polygon_matrix;

	// NED POLYGONS
	int n_total_vertices = 0;
	BOOST_FOREACH (polygon_2D const &poly, polygons)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			n_total_vertices += 1;
		}
		n_total_vertices += 1;
	}
	polygon_matrix.resize(n_total_vertices, 2);

	/*format polygon_matrix array for matlab plotting*/
	int pcount = 0;
	BOOST_FOREACH (polygon_2D const &poly, polygons)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it); // east
			polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it); // north format for matlab

			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		polygon_matrix(pcount, 1) = -1;
		polygon_matrix(pcount, 0) = -1;
		pcount += 1;
	}

	// LLA POLYGONS
	int n_total_vertices_lla = 0;
	BOOST_FOREACH (polygon_2D const &poly, simplified_polygons_lla)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			n_total_vertices_lla += 1;
		}
		n_total_vertices_lla += 1;
	}
	polygon_matrix_lla.resize(n_total_vertices_lla, 2);

	/*format polygon_matrix array for matlab plotting*/
	pcount = 0;
	BOOST_FOREACH (polygon_2D const &poly, simplified_polygons_lla)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			polygon_matrix_lla(pcount, 0) = boost::geometry::get<0>(*it); // latitude
			polygon_matrix_lla(pcount, 1) = boost::geometry::get<1>(*it); // longitude

			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		polygon_matrix_lla(pcount, 1) = -1;
		polygon_matrix_lla(pcount, 0) = -1;
		pcount += 1;
	}

	// SIMPLIFIED POLYGONS
	int n_total_vertices_simplified = 0;
	BOOST_FOREACH (polygon_2D const &poly, simplified_polygons)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			n_total_vertices_simplified += 1;
		}
		n_total_vertices_simplified += 1;
	}
	simplified_polygon_matrix.resize(n_total_vertices_simplified, 2);

	/*format polygon_matrix array for matlab plotting*/
	pcount = 0;
	BOOST_FOREACH (polygon_2D const &poly, simplified_polygons)
	{
		for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)); ++it)
		{
			simplified_polygon_matrix(pcount, 1) = boost::geometry::get<0>(*it); // east
			simplified_polygon_matrix(pcount, 0) = boost::geometry::get<1>(*it); // north format for matlab

			pcount += 1;
		}
		// each polygon is separated with (-1, -1)
		simplified_polygon_matrix(pcount, 1) = -1;
		simplified_polygon_matrix(pcount, 0) = -1;
		pcount += 1;
	}

	PSBMPC_LIB::CPU::save_matrix_to_file("simplified_polygons_lla.csv", polygon_matrix_lla.transpose());

#if ENABLE_TEST_FILE_PLOTTING
	mxArray *map_origin_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
	mxArray *polygon_matrix_mx = mxCreateDoubleMatrix(n_total_vertices, 2, mxREAL);
	mxArray *polygon_matrix_lla_mx = mxCreateDoubleMatrix(n_total_vertices_lla, 2, mxREAL);
	mxArray *simplified_polygon_matrix_mx = mxCreateDoubleMatrix(n_total_vertices_simplified, 2, mxREAL);

	double *p_map_origin = mxGetPr(map_origin_mx);
	double *p_polygon_matrix = mxGetPr(polygon_matrix_mx);
	double *p_polygon_matrix_lla = mxGetPr(polygon_matrix_lla_mx);
	double *p_simplified_polygon_matrix = mxGetPr(simplified_polygon_matrix_mx);

	Eigen::Map<Eigen::Vector2d> map_map_origin(p_map_origin, 2, 1);
	Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, n_total_vertices, 2);
	Eigen::Map<Eigen::MatrixXd> map_polygon_matrix_lla(p_polygon_matrix_lla, n_total_vertices_lla, 2);
	Eigen::Map<Eigen::MatrixXd> map_simplified_polygon_matrix(p_simplified_polygon_matrix, n_total_vertices_simplified, 2);

	map_map_origin = map_origin;
	map_polygon_matrix = polygon_matrix;
	map_polygon_matrix_lla = polygon_matrix_lla;
	map_simplified_polygon_matrix = simplified_polygon_matrix;

	engPutVariable(ep, "map_origin", map_origin_mx);
	engPutVariable(ep, "P", polygon_matrix_mx);
	engPutVariable(ep, "P_lla", polygon_matrix_lla_mx);
	engPutVariable(ep, "P_simplified", simplified_polygon_matrix_mx);
#endif

	//*****************************************************************************************************************
	// Simulation
	//*****************************************************************************************************************
	// Eigen::Vector2d map_origin = grounding_hazard_manager.get_map_origin_ned();
	// Use positions relative to the map origin
	// Eigen::Vector2d map_origin = grounding_hazard_manager.get_map_origin();

	/* xs_os_0.block<2, 1>(0, 0) -= map_origin;
	trajectory.block<2, 1>(0, 0) -= map_origin;
	for (int l = 0; l < n_wps_os; l++)
	{
		waypoints.col(l) -= map_origin;
	}

	for (int i = 0; i < n_do; i++)
	{
		for (int k = 0; k < trajectory_i[i].cols(); k++)
		{
			trajectory_i[i].block<2, 1>(0, k) -= map_origin;
		}
		for (int l = 0; l < n_wps_i[i]; l++)
		{
			waypoints_i[i].col(l) -= map_origin;
		}
	} */
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

#if ENABLE_TEST_FILE_PLOTTING
	for (int i = 0; i < n_do; i++)
	{
		wps_i_mx[i] = mxCreateDoubleMatrix(2, n_wps_i[i], mxREAL);
		traj_i_mx[i] = mxCreateDoubleMatrix(trajectory_i[i].rows(), N, mxREAL);
		P_traj_i_mx[i] = mxCreateDoubleMatrix(16, 1, mxREAL);
	}

	mxArray *T_sim_mx, *n_do_mx, *d_safe_mx, *dt_sim_mx;
	T_sim_mx = mxCreateDoubleScalar(T_sim);
	n_do_mx = mxCreateDoubleScalar(n_do);
	d_safe_mx = mxCreateDoubleScalar(psbmpc.pars.get_dpar(i_dpar_d_safe));
	dt_sim_mx = mxCreateDoubleScalar(dt);

	mxArray *pred_traj_mx;
	double *p_pred_traj;

	mxArray *wps_os_mx = mxCreateDoubleMatrix(2, n_wps_os, mxREAL);
	double *p_wps_os = mxGetPr(wps_os_mx);

	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_os, 2, n_wps_os);
	map_wps_i = waypoints;

	engPutVariable(ep, "n_do", n_do_mx);
	engPutVariable(ep, "d_safe", d_safe_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);
	engPutVariable(ep, "WPs", wps_os_mx);

	engEvalString(ep, "init_psbmpc_plotting_grounding");
	mxArray *i_mx(nullptr), *k_s_mx(nullptr);

	for (int i = 0; i < n_do; i++)
	{
		p_wps_i = mxGetPr(wps_i_mx[i]);

		Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i[i]);
		map_wps_i = waypoints_i[i];

		engPutVariable(ep, "WPs_i", wps_i_mx[i]);

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		engEvalString(ep, "init_obstacle_plot_grounding");
	}
#endif
	Eigen::Vector4d xs_i_k;
	Eigen::VectorXd xs_aug(9);
	double mean_t = 0, t(0.0);
	for (int k = 0; k < N; k++)
	{
		t = k * dt;

		for (int i = 0; i < n_do; i++)
		{
			if (trajectory_i[i].rows() == 4)
			{
				xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
				xs_i_k(2) = trajectory_i[i](3, k) * cos(trajectory_i[i](2, k));
				xs_i_k(3) = trajectory_i[i](3, k) * sin(trajectory_i[i](2, k));
			}
			else
			{
				xs_i_k.block<2, 1>(0, 0) = trajectory_i[i].block<2, 1>(0, k);
				xs_i_k.block<2, 1>(2, 0) = PSBMPC_LIB::CPU::rotate_vector_2D(trajectory_i[i].block<2, 1>(3, k), trajectory_i[i](2, k));
			}
			obstacle_states.col(i) << xs_i_k, A, B, C, D, ID[i];

			obstacle_covariances.col(i) = PSBMPC_LIB::CPU::flatten(P_0);
		}

		obstacle_manager(obstacle_states, obstacle_covariances, psbmpc.pars, dt);

		obstacle_predictor(obstacle_manager.get_data(), trajectory.col(k), psbmpc.pars);

		relevant_polygons = grounding_hazard_manager(trajectory.col(k));

		ownship.update_guidance_references(u_d, chi_d, waypoints, trajectory.col(k), dt, PSBMPC_LIB::LOS);

		if (fmod(t, 5) == 0)
		{
			// std::cout << "n_relevant_so = " << relevant_polygons.size() << std::endl;
			start = std::chrono::system_clock::now();

			psbmpc.calculate_optimal_offsets(
				u_opt,
				chi_opt,
				predicted_trajectory,
				u_d,
				chi_d,
				waypoints,
				trajectory.col(k),
				V_w,
				wind_direction,
				relevant_polygons,
				obstacle_manager.get_data(),
				false);

			end = std::chrono::system_clock::now();
			elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

			mean_t = elapsed.count();

			std::cout << "PSBMPC time usage : " << mean_t << " milliseconds" << std::endl;
		}
		u_c = u_d * u_opt;
		chi_c = chi_d + chi_opt;

		if (k < N - 1)
		{
			trajectory.col(k + 1) = ownship.predict(trajectory.col(k), u_c, chi_c, dt, PSBMPC_LIB::ERK1);
		}

//===========================================
// Send trajectory data to matlab
//===========================================
#if ENABLE_TEST_FILE_PLOTTING
		buffer[BUFSIZE] = '\0';
		engOutputBuffer(ep, buffer, BUFSIZE);

		k_s_mx = mxCreateDoubleScalar(k + 1);
		engPutVariable(ep, "k", k_s_mx);

		pred_traj_mx = mxCreateDoubleMatrix(predicted_trajectory.rows(), predicted_trajectory.cols(), mxREAL);
		p_pred_traj = mxGetPr(pred_traj_mx);

		Eigen::Map<Eigen::MatrixXd> map_pred_traj_os(p_pred_traj, predicted_trajectory.rows(), predicted_trajectory.cols());
		map_pred_traj_os = predicted_trajectory;

		Eigen::Map<Eigen::MatrixXd> map_traj_os(ptraj_os, trajectory.rows(), N);
		map_traj_os = trajectory;

		engPutVariable(ep, "X_pred", pred_traj_mx);
		engPutVariable(ep, "X", traj_os_mx);

		engEvalString(ep, "update_ownship_plot_grounding");

		for (int i = 0; i < n_do; i++)
		{
			ptraj_i = mxGetPr(traj_i_mx[i]);
			p_P_traj_i = mxGetPr(P_traj_i_mx[i]);

			Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, N);
			Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, 1);

			map_traj_i = trajectory_i[i];
			map_P_traj_i = trajectory_covariances_i[i];

			engPutVariable(ep, "X_i", traj_i_mx[i]);
			engPutVariable(ep, "P_i", P_traj_i_mx[i]);

			i_mx = mxCreateDoubleScalar(i + 1);
			engPutVariable(ep, "i", i_mx);

			engEvalString(ep, "update_obstacle_plot_grounding");
		}
#endif
		//======================================================
	}

#if ENABLE_TEST_FILE_PLOTTING
	mxDestroyArray(map_origin_mx);
	mxDestroyArray(d_safe_mx);
	mxDestroyArray(polygon_matrix_mx);
	mxDestroyArray(simplified_polygon_matrix_mx);
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_os_mx);
	mxDestroyArray(pred_traj_mx);
	mxDestroyArray(i_mx);
	mxDestroyArray(k_s_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(dt_sim_mx);
	mxDestroyArray(n_do_mx);
	for (int i = 0; i < n_do; i++)
	{
		mxDestroyArray(traj_i_mx[i]);
		mxDestroyArray(P_traj_i_mx[i]);
		mxDestroyArray(wps_i_mx[i]);
	}
	engClose(ep);
#endif

	return 0;
}