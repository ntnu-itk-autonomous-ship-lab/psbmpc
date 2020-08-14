/****************************************************************************************
*
*  File name : obstacle.cuh
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


#ifndef _OBSTACLE_CUH_
#define _OBSTACLE_CUH_

#include <thrust/device_vector.h>
#include <vector>
#include "Eigen/Dense"
#include <memory>

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

	__host__ __device__ void assign_data(const Obstacle &o);

public:

	__host__ __device__ Obstacle() {};

	__host__ __device__ Obstacle(const Eigen::VectorXd &xs_aug, const Eigen::VectorXd &P, const bool colav_on);

	__host__ __device__ Obstacle(const Obstacle &o); 

	__host__ __device__ Obstacle& operator=(const Obstacle &rhs);

	__host__ __device__ int get_ID() const { return ID; };

	__host__ __device__ void set_colav_on(const bool colav_on) { this->colav_on = colav_on; };

	__host__ __device__ double get_length() const { return l; };

	__host__ __device__ double get_width() const { return w; };
};

#endif