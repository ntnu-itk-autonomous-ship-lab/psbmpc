/****************************************************************************************
*
*  File name : test_obstacle.cu
*
*  Function  : Test file for the Tracked Obstacle class, to figure out bad_alloc bug.
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

#include "tracked_obstacle.h"
#include "utilities.h"
#include <iostream>
#include <vector>
#include <memory>
#include "Eigen/Dense"


int main(){	
	double T = 200, dt = 0.5;
	//*****************************************************************************************************************
	// Obstacle setup
	//*****************************************************************************************************************

	Eigen::VectorXd xs_aug(9), xs_os(6);
	xs_aug << 1000, 0, -5, 0, 5, 5, 5, 5, 0;

	xs_os << 0, 0, 0, 9, 0, 0;

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

	Tracked_Obstacle obstacle = Tracked_Obstacle(xs_aug, flatten(P), Pr_a, Pr_CC, filter_on, T, dt);

	std::cout << obstacle.get_intention_probabilities() << std::endl;

	//*****************************************************************************************************************
	// Test obstacle functionality
	//*****************************************************************************************************************
	/* double d_close = 1000, d_safe = 100; 
	double phi_AH = 68.5 * DEG2RAD, phi_OT = 68.5 * DEG2RAD, phi_HO = 22.5 * DEG2RAD, phi_CR = 68.5 * DEG2RAD;	
	
	int n_ps = 5; 	// KCC, two alternative maneuvers to starboard and port, only one course change of 45 deg
	std::vector<Intention> ps_ordering;
	Eigen::VectorXd ps_course_changes, ps_weights, ps_maneuver_times;
	ps_ordering.resize(n_ps); ps_course_changes.resize(n_ps);
	ps_weights.resize(n_ps); ps_maneuver_times.resize(n_ps);
	
	double Pr_CC_i_test = obstacle.get_a_priori_CC_probability();
	std::cout << "Pr_CC = " << Pr_CC_i_test << std::endl;

	for (int ps = 0; ps < n_ps; ps++)
	{
		if (ps == 0) { ps_ordering[ps] = KCC; }
		else if (ps > 0 && ps < (n_ps - 1) / 2 + 1) { ps_ordering[ps] = SM; }
		else { ps_ordering[ps] = PM; }
	}
	// Lets say a head-on situation, assign weights accordingly:
	ps_maneuver_times << 0, 0, 100, 0, 100;
	ps_course_changes << 0, 45 * DEG2RAD, 45 * DEG2RAD, - 45 * DEG2RAD, - 45 * DEG2RAD;
	ps_weights << (1 - Pr_CC_i_test), Pr_CC_i_test, Pr_CC_i_test, (1 - Pr_CC_i_test), (1 - Pr_CC_i_test);

	obstacle.initialize_prediction(ps_ordering, ps_course_changes, ps_weights, ps_maneuver_times);

	obstacle.predict_independent_trajectories(T, dt, xs_os, phi_AH, phi_CR, phi_HO, phi_OT, d_close, d_safe);

	std::vector<Eigen::MatrixXd> xs_p = obstacle.get_trajectories();

	Eigen::MatrixXd P_p = obstacle.get_trajectory_covariance();

	std::vector<bool> mu = obstacle.get_COLREGS_violation_indicator(); */


	return 0;
}