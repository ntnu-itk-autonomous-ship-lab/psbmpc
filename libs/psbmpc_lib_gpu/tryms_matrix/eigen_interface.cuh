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

	/****************************************************************************************
	*  Name     : assign_eigen_object
	*  Function : Assigns rhs eigen object of any type, to the CML matrix on the lhs.
	*		      Overloaded for dynamic and static cml objects.
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template<class T, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Dynamic_Matrix<T> &lhs, const Eigen_Type_T &rhs)
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

	template<class T, int Max_Rows, int Max_Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_eigen_object(Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols> &lhs, const Eigen_Type_T &rhs)
	{
		assert(Max_Rows >= rhs.rows() && Max_Cols >= rhs.cols());
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
	__host__ __device__ void assign_eigen_object(Static_Matrix<T, Rows, Cols> &lhs, const Eigen_Type_T &rhs)
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

	/****************************************************************************************
	*  Name     : assign_eigen_object
	*  Function : Assigns rhs cml object, to the (dynamic) eigen object on the lhs. 
				  Overloaded for dynamic and static cml objects. Should be used with care
				  such that e.g. Eigen::VectorXd is not assigned to a CML::Matrix2d..
	*  Author   : 
	*  Modified :
	*****************************************************************************************/
	template<class T, typename Eigen_Type_T>
	__host__ __device__ void assign_cml_object(Eigen_Type_T &lhs, const Dynamic_Matrix<T> &rhs)
	{
		int n_rows = rhs.get_rows(), n_cols = rhs.get_cols();
		lhs.resize(n_rows, n_cols);
		for (int i = 0; i < n_rows; i++)
		{
			for (int j = 0; j < n_cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

	template<class T, int Max_Rows, int Max_Cols, typename Eigen_Type_T>
	__host__ __device__ void assign_cml_object(Eigen_Type_T &lhs, const Pseudo_Dynamic_Matrix<T, Max_Rows, Max_Cols> &rhs)
	{
		int n_rows = rhs.get_rows(), n_cols = rhs.get_cols();
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
	__host__ __device__ void assign_cml_object(Eigen_Type_T &lhs, const Static_Matrix<T, Rows, Cols> &rhs)
	{
		lhs.resize(Rows, Cols);
		for (int i = 0; i < Rows; i++)
		{
			for (int j = 0; j < Cols; j++)
			{
				lhs(i, j) = rhs(i, j);
			}
		}
	}

}

#endif