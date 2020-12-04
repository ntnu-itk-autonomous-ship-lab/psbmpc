/****************************************************************************************
*
*  File name : obstacle.h
*
*  Function  : Header file for the obstacle class. New version of the one created
*			   for SBMPC by Inger Berge Hagen and Giorgio D. Kwame Minde Kufoalor
*			   through the Autosea project.
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


#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <vector>
#include "Eigen/Dense"

class Obstacle_SBMPC;

enum Intention 
{
	KCC, 					// Keep current course
	SM, 					// Starboard maneuver
	PM 						// Port maneuver
};

class Obstacle 
{
protected:

	int ID;

	bool colav_on;

	// Obstacle dimension quantifiers, length (l) and width (w)
	double A, B, C, D, l, w;

	double x_offset, y_offset;

	// State and covariance at the current time or predicted time (depending on the derived class usage)
	Eigen::Vector4d xs_0;
	Eigen::Matrix4d P_0;

	void assign_data(const Obstacle &o);

public:

	Obstacle() {};

	Obstacle(const Eigen::VectorXd &xs_aug, const Eigen::VectorXd &P, const bool colav_on);

	Obstacle(const Eigen::Vector4d &xs, const Eigen::VectorXd &P, const int ID, const bool colav_on);

	Obstacle(const Obstacle &o); 

	Obstacle& operator=(const Obstacle &rhs);

	int get_ID() const { return ID; };

	void set_colav_on(const bool colav_on) { this->colav_on = colav_on; };

	double get_length() const { return l; };

	double get_width() const { return w; };
};

#endif