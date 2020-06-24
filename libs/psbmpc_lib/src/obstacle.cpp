/****************************************************************************************
*
*  File name : obstacle.cpp
*
*  Function  : Obstacle class functions. Modified version of the one created for SBMPC 
*			   by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor through the 
*  			   Autosea project.
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


#include "obstacle.h"
#include "kf.h"
#include "iostream" 

/****************************************************************************************
*  Name     : Obstacle
*  Function : Class constructor, initializes parameters, variables and objects
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::Obstacle(
	const Eigen::VectorXd& xs_aug, 								// In: Augmented bstacle state [x, y, V_x, V_y, A, B, C, D, id]
	const Eigen::Matrix4d& P, 									// In: Obstacle covariance
	const Eigen::VectorXd Pr_a,									// In: Obstacle intention probability vector
	const bool filter_on, 										// In: Boolean determining whether KF should be used or not
	const bool colav_on,										// In: Boolean determining whether the obstacle uses a COLAV system or not in the MPC predictions
	const double T, 											// In: Prediction horizon
	const double dt 											// In: Sampling interval
	) : 
	ID(xs_aug(9)), filter_on(filter_on), colav_on(colav_on),
	l(xs_aug(5) + xs_aug(6)), w(xs_aug(7) + xs_aug(8)), 
	x_offset(xs_aug(5) - xs_aug(6)), y_offset(xs_aug(7) - xs_aug(8)),
	duration_lost(0.0)
{
	this->Pr_a = Pr_a;

	int n_samples = std::round(T / dt);

	// n = 4 states in obstacle model for independent trajectories, using MROU
	xs_p.resize(1);
	xs_p[0].resize(4, n_samples);

	P_p.resize(1);
	P_p[0].resize(16, n_samples);

	xs_colav_p.resize(4, n_samples);

	P_colav_p.resize(16, n_samples);

	double psi = xs_aug(2);
	xs_p[0](0, 0) = xs_aug(0) + x_offset * cos(psi) - y_offset * sin(psi); 
	xs_p[0](1, 0) = xs_aug(1) + x_offset * cos(psi) + y_offset * sin(psi);
	xs_p[0](2, 0) = xs_aug(3) * cos(psi) - xs_aug(4) * sin(psi); 
	xs_p[0](3, 0) = xs_aug(3) * sin(psi) + xs_aug(4) * cos(psi); 

	P_p[0].block<16, 1>(0, 0) = Utilities::flatten(P);

	Eigen::Vector4d xs_0;
	xs_0 << xs_p[0](0, 0), xs_p[0](1, 0), xs_p[0](2, 0), xs_p[0](3, 0);

	kf = new KF(xs_0, P, ID, dt, 0.0);

	if(filter_on) 
	{
		kf->update(xs_0, duration_lost, dt);

		duration_tracked = kf->get_time();
	}

	mrou = new MROU(0.8, 0, 0.8, 0.1, 0.1);
}

/****************************************************************************************
*  Name     : ~Obstacle
*  Function : Class destructor, clears the dynamic kalman filter object
*  Author   : 
*  Modified :
*****************************************************************************************/
Obstacle::~Obstacle(){
	delete kf;
	delete mrou;
}

/****************************************************************************************
*  Name     : resize_trajectories
*  Function : Resizes trajectory and preserves old values inside new range of samples
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::resize_trajectories(const int n_samples)
{
	int n_ps = xs_p.size();
	for(int ps = 0; ps < n_ps; ps++)
	{
		xs_p[ps].conservativeResize(4, n_samples);
		P_p[ps].conservativeResize(16, n_samples);
	}
	xs_colav_p.conservativeResize(4, n_samples);
	P_colav_p.conservativeResize(16, n_samples);
}

/****************************************************************************************
*  Name     : initialize_independent_prediction
*  Function : Sets up prediction scenario ordering, maneuvering times for the different
* 			  obstacle alternative maneuvers
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/

void Obstacle::initialize_independent_prediction(
	const std::vector<Intention> &ps_ordering, 					// In: Prediction scenario ordering
	const Eigen::VectorXd &ps_course_change_ordering, 			// In: Order of alternative maneuvers for the prediction scenarios
	const Eigen::VectorXd &ps_weights,	 						// In: The different course changes employed in the prediction scenarios
	const Eigen::VectorXd &maneuver_times 						// In: Time of alternative maneuvers for the prediction scenarios
	)
{
	this->ps_ordering = ps_ordering;
	
	this->ps_course_change_ordering = ps_course_change_ordering;

	this->ps_weights = ps_weights;

	this->maneuver_times = maneuver_times;
}

/****************************************************************************************
*  Name     : predict_independent_trajectories
*  Function : Predicts the obstacle trajectories for scenarios where the obstacle
*			  does not take the own-ship into account 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void Obstacle::predict_independent_trajectories(						
	const double T, 											// In: Time horizon
	const double dt 											// In: Time step
	)
{
	int n_ps = ps_ordering.size();
	mu.resize(n_ps);
	int n_samples = std::round(T / dt);
	resize_trajectories(n_samples);
	Eigen::Matrix4d P_0 = kf->get_covariance();
	double t = 0;
	for(int ps = 0; ps < n_ps; ps++)
	{
		xs_p[ps].col(0) = kf->get_state();
		P_p[ps].col(0) = Utilities::flatten(P_0);
		for(int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;

			switch (ps_ordering[ps])
			{
				case KCC :	// Proceed
				case SM :
				{

				}
				case PM :
				{
					
				}
				default :
				{
					std::cout << "This intention is not valid!" << std::endl;
				}
			}

			if (k < n_samples - 1)
			{
				xs_p[ps].col(k + 1) = mrou->predict_state(xs_p[ps].col(k), v_p[ps].col(k), dt);
				P_p[ps].col(k + 1) = mrou->predict_covariance(P_0, t);
			}
			
		}
	}
}