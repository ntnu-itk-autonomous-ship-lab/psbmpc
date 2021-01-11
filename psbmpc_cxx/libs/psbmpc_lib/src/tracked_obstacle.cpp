/****************************************************************************************
*
*  File name : tracked_obstacle.cpp
*
*  Function  : Tracked bstacle class functions. Derived class of base Obstacle class,
*		  	   used in the PSB-MPC obstacle management.
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

#include "tracked_obstacle.h"

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
	Obstacle(xs_aug, P, false), 
	duration_lost(0.0),
	kf(new KF(xs_0, P_0, ID, dt, 0.0)),
	mrou(new MROU(0.8, 0.0, 0.8, 0.1, 0.1))
{
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }

	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	xs_p.resize(1);
	xs_p[0].resize(4, n_samples);

	P_p.resize(16, n_samples);
	P_p.col(0) = P;
	
	xs_p[0].col(0) = xs_0;

	if(filter_on) 
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}
}

/****************************************************************************************
*  Name     : Tracked_Obstacle
*  Function : Copy constructor, prevents shallow copies and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Tracked_Obstacle::Tracked_Obstacle(
	const Tracked_Obstacle &to 													// In: Tracked obstacle to copy
	) : 
	Obstacle(to)
{
	assign_data(to);
}

/****************************************************************************************
*  Name     : operator=
*  Function : Assignment operator to prevent shallow assignments and bad pointer management
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
Tracked_Obstacle& Tracked_Obstacle::operator=(
	const Tracked_Obstacle &rhs 										// In: Rhs tracked obstacle to assign
	)
{
	if (this == &rhs)
	{
		return *this;
	}

	assign_data(rhs);

	return *this;
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
		xs_p[ps].resize(4, n_samples); 	xs_p[ps].col(0) = kf->get_state();
	}
	P_p.resize(16, n_samples);
	P_p.col(0) 	= flatten(kf->get_covariance());
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
	this->ps_ordering = ps_ordering;
	
	this->ps_course_changes = ps_course_changes;

	this->ps_maneuver_times = ps_maneuver_times;

	int n_a = Pr_a.size();
	ps_intention_count.resize(n_a);
	
	if (n_a == 1)	{ ps_intention_count(0) = 1; }
	else // n_a = 3
	{
		ps_intention_count(0) = 1;
		int n_ps = ps_ordering.size();
		for (int ps = 0; ps < n_ps; ps++)
		{
			if (ps_ordering[ps] == SM)		{ ps_intention_count(1) += 1; }
			else if (ps_ordering[ps] == PM)	{ ps_intention_count(2) += 1; }
		}
	}	
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
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();

		xs_0 = kf->get_state();

		P_0 = kf->get_covariance();
	}
	else
	{ 
		kf->reset(xs_0, P_0, 0.0); 
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
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();

		xs_0 = kf->get_state();

		P_0 = kf->get_covariance();
	}
	else
	{ 
		kf->reset(xs_0, P_0, 0.0); 
	}
	
	this->Pr_a = Pr_a / Pr_a.sum(); 
	
	if (Pr_CC > 1) 	{ this->Pr_CC = 1;}
	else 			{ this->Pr_CC = Pr_CC; }
}

/****************************************************************************************
*  Private functions
*****************************************************************************************/
/****************************************************************************************
*  Name     : assign_data
*  Function : 
*  Author   : 
*  Modified :
*****************************************************************************************/
void Tracked_Obstacle::assign_data(
	const Tracked_Obstacle &to 												// In: Tracked_Obstacle whose data to assign to *this
	)
{
	// Boring non-pointer class member copy
	this->ID = to.ID;

	this->colav_on = to.colav_on;

	this->A = to.A; this->B = to.B; this->C = to.C; this->D = to.D;
	this->l = to.l; this->w = to.w;

	this->x_offset = to.x_offset; this->y_offset = to.y_offset;

	this->xs_0 = to.xs_0;
	this->P_0 = to.P_0;

	this->Pr_a = to.Pr_a; 

	this->Pr_CC = to.Pr_CC;

	this->duration_tracked = to.duration_tracked; this->duration_lost = to.duration_lost;
	
	this->mu = to.mu;

	this->P_p = to.P_p;
	this->xs_p = to.xs_p;
	this->v_p = to.v_p;

	this->ps_ordering = to.ps_ordering;
	this->ps_course_changes = to.ps_course_changes; this->ps_maneuver_times = to.ps_maneuver_times;

	this->kf.reset(new KF(*(to.kf)));
	this->mrou.reset(new MROU(*(to.mrou)));
}