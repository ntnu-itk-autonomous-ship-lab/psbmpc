/****************************************************************************************
*
*  File name : eigen_interface.cuh
*
*  Function  : Header file for functions used to transfer data from Eigen objects to 
*			   TML objects.
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
#include <assert.h>
#include <math.h>
#include "Eigen/Dense"

#include "dynamic_matrix.cuh"
#include "static_matrix.cuh"
#include "tml.cuh"


namespace TML
{

	/****************************************************************************************
	*  Name     : assign_eigen_object
	*  Function : Assigns rhs eigen object of any type, to the tml matrix on the lhs.
	*		      Overloaded for dynamic and static tml objects.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template<class T, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Dynamic_Matrix<T> &lhs, const Eigen_Type_T &rhs)
	{
		size_t n_rows = rhs.rows(), n_cols = rhs.cols();
		lhs.resize(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, size_t Max_Rows, size_t Max_Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(PDMatrix<T, Max_Rows, Max_Cols> &lhs, const Eigen_Type_T &rhs)
	{
		size_t n_rows = rhs.rows(), n_cols = rhs.cols();
		assert(Max_Rows >= n_rows && Max_Cols >= n_cols);
		
		lhs.resize(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, size_t Rows, size_t Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Static_Matrix<T, Rows, Cols> &lhs, const Eigen_Type_T &rhs)
	{
		size_t n_rows = rhs.rows(), n_cols = rhs.cols();
		assert(Rows == n_rows && Cols == n_cols);

		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	/****************************************************************************************
	*  Name     : assign_eigen_object
	*  Function : Assigns rhs TML object, to the (dynamic) eigen object on the lhs. 
				  Overloaded for dynamic and static TML objects. Should be used with care
				  such that e.g. Eigen::VectorXd is not assigned to a TML::Matrix2d..
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template<class T, typename Eigen_Type_T>
	__host__ __device__ void assign_tml_object(Eigen_Type_T &lhs, const Dynamic_Matrix<T> &rhs)
	{
		size_t n_rows = rhs.get_rows(), n_cols = rhs.get_cols();
		lhs.resize(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, size_t Max_Rows, size_t Max_Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_tml_object(Eigen_Type_T &lhs, const PDMatrix<T, Max_Rows, Max_Cols> &rhs)
	{
		size_t n_rows = rhs.get_rows(), n_cols = rhs.get_cols();
		lhs.resize(n_rows, n_cols);
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, size_t Rows, size_t Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_tml_object(Eigen_Type_T &lhs, const Static_Matrix<T, Rows, Cols> &rhs)
	{
		lhs.resize(Rows, Cols);
		for (size_t i = 0; i < Rows; i++)
		{
			for (size_t j = 0; j < Cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

}

#endif