/****************************************************************************************
*
*  File name : kf.cpp
*
*  Function  : Class functions for a "hardcoded" Kalman Filter (KF). Alternative version
*			   of the one created by Giorgio D. Kwame Minde Kufoalor through the Autosea
*		       project, modified with .cu for this GPU-implementation.
*
*
*            ---------------------
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

#include "kf.hpp"
#include <stdexcept>
#include <iostream>

namespace PSBMPC_LIB
{
	/****************************************************************************************
	*  Name     : KF
	*  Function : Class constructors, initializes parameters and variables. Default constructor
	*			  is used for the Autosea KF tracker stuff.
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	KF::KF() : t_0(0.0), t(0.0), initialized(false)
	{
		xs_p.setZero();
		xs_upd.setZero();

		I.setIdentity();

		set_measurement_matrix(true);

		q = 0.001;

		R << .2, .0, .0, .0,
			.0, .2, .0, .0,
			.0, .0, .01, .0,
			.0, .0, .0, .01;

		P_0 << .1, .0, .0, .0,
			.0, .1, .0, .0,
			.0, .0, .1, .0,
			.0, .0, .0, .1;

		P_p = P_0;

		P_upd = P_0;
	}

	KF::KF(
		const Eigen::Vector4d &xs_0,		 // In: Initial filter state
		const Eigen::Matrix4d &P_0,			 // In: Initial filter covariance
		const double t_0,					 // In: Initial time
		const bool use_velocity_measurements // In: Determines whether or not the KF processes a 2d or 4d measurement vector
		) : t_0(t_0), t(t_0), initialized(true), xs_p(xs_0), P_0(P_0)
	{
		I.setIdentity();

		set_measurement_matrix(use_velocity_measurements);

		q = 0.001;

		R << .2, .0, .0, .0,
			.0, .2, .0, .0,
			.0, .0, .01, .0,
			.0, .0, .0, .01;

		xs_upd = xs_p;

		P_p = P_0;
		P_upd = P_0;
	}

	KF::KF(
		const Eigen::Matrix4d &P_0,			 // In: Initial filter covariance
		const Eigen::Matrix4d &R,			 // In: Measurement noise covariance
		const double q,						 // In: Process noise strength
		const double t_0,					 // In: Initial time
		const bool use_velocity_measurements // In: Determins whether or not the KF processes a 2d or 4d measurement vector
		) : t_0(t_0), t(t_0), initialized(false), P_0(P_0), R(R), q(q)
	{
		I.setIdentity();

		set_measurement_matrix(use_velocity_measurements);

		xs_p.setZero();
		xs_upd.setZero();

		P_p = P_0;
		P_upd = P_0;
	}

	KF::KF(
		const Eigen::Vector4d &xs_0,		 // In: Initial filter state
		const Eigen::Matrix4d &P_0,			 // In: Initial filter covariance
		const Eigen::Matrix4d &R,			 // In: Measurement noise covariance
		const double q,						 // In: Process noise strength
		const double t_0,					 // In: Initial time
		const bool use_velocity_measurements // In: Determins whether or not the KF processes a 2d or 4d measurement vector
		) : t_0(t_0), t(t_0), initialized(true), xs_p(xs_0), P_0(P_0), R(R), q(q)
	{
		I.setIdentity();

		set_measurement_matrix(use_velocity_measurements);

		xs_upd = xs_p;

		P_p = P_0;
		P_upd = P_0;
	}

	/****************************************************************************************
	*  Name     : reset
	*  Function : Resets filter to specified time and state, and covariance to P_0
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void KF::reset(
		const Eigen::Vector4d &xs_0, // In: Initial filter state
		const Eigen::Matrix4d &P_0,	 // In: Initial filter covariance
		const double t_0			 // In: Initial time
	)
	{
		this->t_0 = t_0;
		t = t_0;

		xs_p = xs_0;
		xs_upd = xs_0;

		this->P_0 = P_0;

		P_p = P_0;
		P_upd = P_0;

		initialized = true;
	}

	/****************************************************************************************
	*  Name     : predict
	*  Function : Predicts the estimate dt seconds forward in time using the system model
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void KF::predict(
		const double dt // In: Sampling interval
	)
	{
		A << 1, 0, dt, 0,
			0, 1, 0, dt,
			0, 0, 1, 0,
			0, 0, 0, 1;

		Q << pow(dt, 3) / 3.0, 0.0, pow(dt, 2) / 2.0, 0.0,
			0.0, pow(dt, 3) / 3.0, 0.0, pow(dt, 2) / 2.0,
			pow(dt, 2) / 2.0, 0.0, dt, 0.0,
			0.0, pow(dt, 2) / 2.0, 0.0, dt;
		Q = q * Q;

		xs_p = A * xs_upd;

		P_p = A * P_upd * A.transpose() + Q;
	}

	/****************************************************************************************
	*  Name     : update
	*  Function : Updates the KF prediction with the measurement, and
	*			  propagates the prediction
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void KF::update(
		const Eigen::Vector4d &y_m, // In: Measurement of cartesian position and velocity
		const double duration_lost, // In: How long a track on an obstacle has been lost
		const double dt				// In: Sampling interval
	)
	{
		if (!initialized)
			throw std::runtime_error("Filter is not initialized!");

		if (duration_lost == 0.0)
		{

			Eigen::Matrix<double, 4, 4> K;
			K = P_p * C.transpose() * (C * P_p * C.transpose() + R).inverse();

			xs_upd = xs_p + K * (y_m - C * xs_p);

			P_upd = (I - K * C) * P_p;
		}
		else
		{

			xs_upd = xs_p;

			P_upd = P_p;
		}

		t += dt; // Time used for fault detection (measurement loss)
	}

	// Use this update function when the KF is used as a tracking system outside the COLAV algorithm
	// where typically only position measurements of vessels are used.
	void KF::update(
		const Eigen::Vector2d &y_m, // In: Measurement of cartesian position
		const double dt,			// In: Sampling interval
		const bool dead_reckon		// In: Boolean flag to determine whether to use measurements or not
	)
	{
		if (!initialized)
			throw std::runtime_error("Filter is not initialized!");

		if (!dead_reckon)
		{
			Eigen::Matrix<double, 4, 2> K;
			K = P_p * C.transpose() * (C * P_p * C.transpose() + R.block<2, 2>(0, 0)).inverse();

			xs_upd = xs_p + K * (y_m - C * xs_p);

			P_upd = (I - K * C) * P_p;
		}
		else
		{
			xs_upd = xs_p;

			P_upd = P_p;
		}

		t += dt;
	}

	/****************************************************************************************
	*  PRIVATE FUNCTIONS
	*****************************************************************************************/

	/****************************************************************************************
	*  Name     : set_measurement_matrix
	*  Function :
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void KF::set_measurement_matrix(
		const bool use_velocity_measurements // In: Determins whether or not the KF processes a 2d or 4d measurement vector
	)
	{
		if (use_velocity_measurements)
		{
			C.resize(4, 4);
			C << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
		}
		else
		{
			C.resize(2, 4);
			C << 1, 0, 0, 0,
				0, 1, 0, 0;
		}
	}
}