/****************************************************************************************
*
*  File name : test_cmatrix.cpp
*
*  Function  : Test file for the CMatrix class meant for use in CUDA kernels.
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


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "cmatrix.cuh"
#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include "Eigen/Dense"

int main()
{
	std::random_device seed;

	std::mt19937_64 gen(seed());

	std::normal_distribution<double> std_norm_pdf(0.0, 1.0);


	//================================================================================
	// Assignment operator + copy constructor test
	//================================================================================
	CMatrix<double> t1(3);
	CMatrix<double> t2(2);
	t1 = t2;

	CMatrix<double> t3 = t1;

	//================================================================================
	// 2x2 inverse test
	//================================================================================
	size_t n_rows = 2, n_cols = 2;
	CMatrix<double> M1(n_rows, n_cols);
	Eigen::MatrixXd M2(n_rows, n_cols); 
	Eigen::MatrixXd M_diff(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M2(i, j) = std_norm_pdf(gen) + 5;
			M1(i, j) = M2(i, j);
		}
	}

	std::cout << M1 << std::endl;
	std::cout << M2 << std::endl;

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1.inverse()(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << M_diff << std::endl;
	
	//================================================================================
	// 3x3 inverse test
	//================================================================================
	n_rows = 3; n_cols = 3;
	M1.resize(n_rows, n_cols);
	M2.resize(n_rows, n_cols); 
	M_diff.resize(n_rows, n_cols);
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M2(i, j) = std_norm_pdf(gen) + 5;
			M1(i, j) = M2(i, j);
		}
	}

	std::cout << M1 << std::endl;
	std::cout << M2 << std::endl;

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1.inverse()(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << M_diff << std::endl;
	//================================================================================
	// 4x4 inverse test
	//================================================================================
	n_rows = 4; n_cols = 4;
	M1.resize(n_rows, n_cols);
	M2.resize(n_rows, n_cols); 
	M_diff.resize(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M2(i, j) = std_norm_pdf(gen) + 5;
			M1(i, j) = M2(i, j);
		}
	}

	std::cout << M1 << std::endl;
	std::cout << M2 << std::endl;

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1.inverse()(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << M_diff << std::endl;
	//================================================================================
	// Dot product test
	//================================================================================

	//================================================================================
	// Quadratic form calculation test
	//================================================================================

	return 0;
}