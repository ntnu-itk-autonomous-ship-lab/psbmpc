/****************************************************************************************
*
*  File name : eigen_interface.cuh
*
*  Function  : Header file for functions used to transfer data from Eigen objects to 
*			   CML objects.
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

#ifndef _EIGEN_INTERFACE_CUH_
#define _EIGEN_INTERFACE_CUH_

#include <thrust/device_vector.h>
#include <assert.h>
#include <math.h>
#include "Eigen/Dense"

#include "dynamic_matrix.cuh"
#include "static_matrix.cuh"
#include "cml.cuh"


namespace CML
{
	template<class T, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Dynamic_Matrix<T> &lhs, Eigen_Type_T &rhs)
	{
		int n_rows = rhs.rows(), n_cols = rhs.cols();
		lhs.resize(n_rows, n_cols);
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, int Rows, int Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Static_Matrix<T, Rows, Cols> &lhs, Eigen_Type_T &rhs)
	{
		int n_rows = rhs.rows(), n_cols = rhs.cols();
		assert(Rows == n_rows && Cols == n_cols);

		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}
}

#endif