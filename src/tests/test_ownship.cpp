
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Matplotlib/matplotlibcpp.h"
#include "ownship.h"
#include <iostream>
#include <variant>
#include <vector>
#include "Eigen/StdVector"

namespace plt = matplotlibcpp;

int main(){

	Eigen::Matrix<double, 6, 1> xs;
	xs << 0, 0, 0, 6, 0, 0;

	std::cout << "xs = [" << xs(0) << ", " << xs(1) << ", " << xs(2) << ", " << xs(3) << ", " << xs(4) << ", " << xs(5) << "]" << std::endl;

	double T = 100; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	std::vector<Eigen::VectorXd> chi_offsets, value;
	chi_offsets.resize(3);
	chi_offsets[0].resize(13); chi_offsets[1].resize(7); chi_offsets[2].resize(5);
	Eigen::VectorXd v1, v2, v3;
	v1.resize(13); v2.resize(7); v3.resize(5);
	v1 << -90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90;
	v2 << -90, -75, -45, 0, 45, 75, 90;
	v3 << -90, -45, 0, 45, 90;
	chi_offsets[0] << v1;
	chi_offsets[1] << v2;
	chi_offsets[2] << v3;

	Eigen::Vector4d offset_sequence;
	Eigen::Vector2d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0;
	maneuver_times << 1, 100;

	
	for (std::vector<Eigen::VectorXd>::iterator it = chi_offsets.begin(); it != chi_offsets.end(); it++){
		std::cout << "chi_offsets(j) = " << *it << " | ";
	}
	std::cout << std::endl;
	
	Ownship* asv = new Ownship(); 

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	int n_samples = T / dt;
	trajectory.resize(6, n_samples);
	trajectory.block<6, 1>(0, 0) << xs;
	std::cout << "traj init = [" << trajectory(0, 0) << ", " << trajectory(1, 0) << ", " << trajectory(2, 0) << ", " \
	<< trajectory(3, 0) << ", " << trajectory(4, 0) << ", " << trajectory(5, 0) << "]" << std::endl;

	waypoints.resize(2, 4); 
	waypoints << 0, 200, 400, 600,
				 0,  0,   0,  100;
	std::cout << "wp(:, 0) = " << waypoints(0, 0) << ", " << waypoints(1, 0) << std::endl;

	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, ERK1, LOS, T, dt);

	
	matplotlibcpp::plot(trajectory.block(1, n_samples, 1, 0), trajectory.block(1, n_samples, 0, 0));

	return 0;
}