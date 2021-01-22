/****************************************************************************************
*
*  File name : tracked_obstacle.cu
*
*  Function  : Tracked bstacle class functions. Derived class of base Obstacle class,
*		  	   used in the PSB-MPC obstacle management. Modified with .cu for this 
*			   GPU-implementation.
*
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

#include "tracked_obstacle.cuh"
#include "utilities.cuh"

#include "assert.h"
#include <iostream> 

/****************************************************************************************
*  Name     : Tracked_Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Tracked_Obstacle::Tracked_Obstacle(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_a,								// In: Obstacle intention probability vector
	const double Pr_CC, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) :
	Obstacle(xs_aug, false), 
	duration_tracked(0.0), duration_lost(0.0),
	mrou(MROU())
{
 	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = reshape(P, 4, 4); 

	this->kf = KF(xs_0, P_0, ID, dt, 0.0);

	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }

 	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	this->xs_p.resize(1);
	this->xs_p[0].resize(4, n_samples);
	this->xs_p[0].col(0) = xs_0;

	this->P_p.resize(16, n_samples);
	this->P_p.col(0) = P; 

	if(filter_on) 
	{
		this->kf.update(xs_0, duration_lost, dt);

		this->duration_tracked = kf.get_time();
	}
}

/****************************************************************************************
*  Name     : resize_trajectories
*  Function : Resizes independent trajectories, and also the trajectory covariance
*			  which is the same for all trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::resize_trajectories(const int n_samples)
{
	int n_ps = ps_ordering.size();
	xs_p.resize(n_ps);
	for(int ps = 0; ps < n_ps; ps++)
	{
		xs_p[ps].resize(4, n_samples); 	xs_p[ps].col(0) = kf.get_state();
	}
	P_p.resize(16, n_samples);
	P_p.col(0) 	= flatten(kf.get_covariance());
}

/****************************************************************************************
*  Name     : initialize_prediction
*  Function : Sets up independent or dependent obstacle prediction, depending on if
*		      colav is active or not. For the dependent obstacle prediction,
*			  ps_course_changes and ps_maneuver_times are "dont care" variables, hence
*		      two overloads.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::initialize_prediction(
	const std::vector<Intention> &ps_ordering, 						// In: Prediction scenario ordering
	const Eigen::VectorXd &ps_course_changes, 						// In: Order of alternative maneuvers for the prediction scenarios
	const Eigen::VectorXd &ps_maneuver_times 						// In: Time of alternative maneuvers for the prediction scenarios	
	)
{
	// the size of ps_ordering is greater than the size of ps_course_changes and ps_maneuver_times
	// when intelligent obstacle predictions are considered (joint predictions are active)
	// Thus n_ps_i = length(ps_ordering) = n_ps_i_independent + n_ps_i_dependent
	// where n_ps_i_independent = size(ps_course_changes)
	this->ps_ordering = ps_ordering;

	this->ps_course_changes = ps_course_changes;

	this->ps_maneuver_times = ps_maneuver_times;

	int n_ps_independent = ps_ordering.size();
	mu.resize(n_ps_independent);

	int n_a = Pr_a.size();
	ps_intention_count.resize(n_a); ps_intention_count.setZero();
	
	if (n_a == 1)	{ ps_intention_count(0) = 1; }
	else // n_a = 3
	{
		ps_intention_count(0) = 1;
		
		for (int ps = 0; ps < n_ps_independent; ps++)
		{
			if (ps_ordering[ps] == SM)		{ ps_intention_count(1) += 1; }
			else if (ps_ordering[ps] == PM)	{ ps_intention_count(2) += 1; }
		}
	}	
	std::cout << ps_intention_count.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : update
*  Function : Updates the obstacle state, covariance, intention probabilities, a priori
* 			  COLREGS compliance probability at the current time prior to a run of the
*			  PSB-MPC. Two overloads depending on if obstacle is lost or not, respectively
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::update(
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double dt 											// In: Prediction time step
	)
{
	// Depending on if the KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf.update(xs_0, duration_lost, dt);

		duration_tracked = kf.get_time();

		xs_0 = kf.get_state();

		P_0 = kf.get_covariance();
	}
	else
	{ 
		kf.reset(xs_0, P_0, 0.0); 
	}
}

void Tracked_Obstacle::update(
	const Eigen::VectorXd &xs_aug, 								// In: Augmented obstacle state [x, y, V_x, V_y, A, B, C, D, ID]
	const Eigen::VectorXd &P, 									// In: Obstacle covariance
	const Eigen::VectorXd &Pr_a,								// In: Obstacle intention probability vector
	const double Pr_CC, 										// In: A priori COLREGS compliance probability
	const bool filter_on, 										// In: Indicator of whether the KF is active
	const double dt 											// In: Prediction time step
	)
{
	double psi = atan2(xs_aug(3), xs_aug(2));
	xs_0(0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_0(1) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_0(2) = xs_aug(2);
	xs_0(3) = xs_aug(3);

	P_0 = reshape(P, 4, 4);

	// Depending on if the KF is on/off, the state and
	// covariance are updated, or just reset directly to the input
	// data (xs_0 and P_0)
	if (filter_on)
	{
		kf.update(xs_0, duration_lost, dt);

		duration_tracked = kf.get_time();

		xs_0 = kf.get_state();

		P_0 = kf.get_covariance();
	}
	else
	{ 
		kf.reset(xs_0, P_0, 0.0); 
	}
	
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }
}

/****************************************************************************************
*  Private functions
*****************************************************************************************/