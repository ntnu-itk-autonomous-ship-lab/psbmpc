/****************************************************************************************
*
*  File name : obstacle.cuh
*
*  Function  : Header file for the obstacle class, slightly
*			   modified for this GPU-implementation.
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

#pragma once

#include <thrust/device_vector.h>
#include <vector>

#include "tml.cuh"
#include "Eigen/Dense"

enum Intention 
{
	KCC, 					// Keep current course
	SM, 					// Starboard maneuver
	PM 						// Port maneuver
};
class __align__(16) Obstacle 
{
protected:

	int ID;

	// Obstacle dimension quantifiers, length (l) and width (w)
	float A, B, C, D, l, w;

	float x_offset, y_offset;

public:

	__host__ __device__ Obstacle() {}

	__host__ __device__ Obstacle(const Eigen::VectorXd &xs_aug);

	__host__ __device__ Obstacle(const TML::PDMatrix<float, 9, 1> &xs_aug);

	__host__ __device__ int get_ID() const { return ID; };

	__host__ __device__ float get_length() const { return l; };

	__host__ __device__ float get_width() const { return w; };
};