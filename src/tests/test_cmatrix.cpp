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
#include "kf.h"
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
	CMatrix<double> M1(n_rows, n_cols), M1_inv = M1;
	Eigen::MatrixXd M2(n_rows, n_cols); 
	Eigen::MatrixXd M_diff(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M2(i, j) = 2 * std_norm_pdf(gen) + 5;
			if (i == j)
			{
				M2(i, j) = 2 * std_norm_pdf(gen) + 10;
			}
			M1(i, j) = M2(i, j);
		}
	}
	std::cout << "Custom 2x2 = " << std::endl;
	std::cout << M1 << std::endl;

	M1_inv = M1.inverse();

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1_inv(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << "Difference in 2x2 inverse: " << std::endl;
	std::cout << M_diff << std::endl;
	
	//================================================================================
	// 3x3 inverse test
	//================================================================================
	n_rows = 3; n_cols = 3;
	M1.resize(n_rows, n_cols); M1_inv.resize(n_rows, n_cols);
	M2.resize(n_rows, n_cols); 
	M_diff.resize(n_rows, n_cols);
	while (M2.determinant() <= 0)
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				M2(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					M2(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				M1(i, j) = M2(i, j);
			}
		}
	}
	
	std::cout << "Custom 3x3 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 3x3 determinant= " << M2.determinant() << std::endl;
	std::cout << "Custom 3x3 determinant= " << M1.determinant() << std::endl;

	M1_inv = M1.inverse();
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1_inv(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << "Difference in 3x3 inverse: " << std::endl;
	std::cout << M_diff << std::endl;
	//================================================================================
	// 4x4 inverse test
	//================================================================================
	n_rows = 4; n_cols = 4;
	M1.resize(n_rows, n_cols); M1_inv.resize(n_rows, n_cols);
	M2.resize(n_rows, n_cols); 
	M_diff.resize(n_rows, n_cols);

	//M2 = kf->get_covariance();
	while (M2.determinant() <= 0)
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				M2(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					M2(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				M1(i, j) = M2(i, j);
			}
		}
	}
	std::cout << "Custom 4x4 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 4x4 determinant= " << M2.determinant() << std::endl;
	std::cout << "Custom 4x4 determinant= " << M1.determinant() << std::endl;

	M1_inv = M1.inverse();

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = M1_inv(i, j) - M2.inverse()(i, j);
		}
	}
	std::cout << "Difference in 4x4 inverse: " << std::endl;
	std::cout << M_diff << std::endl;
	//================================================================================
	// Transpose test
	//================================================================================
	n_rows = 4; n_cols = 2;
	CMatrix<double> O(n_rows, n_cols);
	std::cout << "Original = " << std::endl;
	std::cout << O << std::endl;

	std::cout << "Transposed = " << std::endl;
	std::cout << O.transposed() << std::endl;
	//================================================================================
	// Dot product test
	//================================================================================
	CMatrix<double> v1(6, 1), v2(6, 1);
	double v3;
	for (size_t i = 0; i < 6; i++)
	{
		v1(i, 0) = i; 
		v2(i, 0) = v1(i, 0);
	}

	std::cout << "v1' * v2 = " << std::endl; 
	v3 = v1.dot(v2);
	std::cout << v3 << std::endl;

	std::cout << "v2' * v1 = " << std::endl; 
	v3 = v2.dot(v1);
	std::cout << v3 << std::endl;

	v1.resize(1, 10); v2.resize(1, 10);
	for (size_t i = 0; i < 10; i++)
	{
		v1(0, i) = i; 
		v2(0, i) = v1(0, i);
	}
	std::cout << "New v1' * v2 = " << std::endl; 
	v3 = v2.dot(v1);
	std::cout << v3 << std::endl;

	//================================================================================
	// Operator tests
	//================================================================================
	n_rows = 4; n_cols = 4;
	CMatrix<double> A(n_rows, n_cols), B(n_rows, n_cols), C;
	Eigen::MatrixXd A_e(n_rows, n_cols), B_e(n_rows, n_cols), C_e(n_rows, n_cols);
	M_diff.resize(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			A(i, j) = 2 * std_norm_pdf(gen) + 5;
			A_e(i, j) = A(i, j);

			B(i, j) = 2 * std_norm_pdf(gen) + 5;
			B_e(i, j) = B(i, j);
		}
	}
	std::cout << "A + B diff " << std::endl; 
	C = A + B; C_e = A_e + B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "A - B diff " << std::endl; 
	C = A - B; C_e = A_e - B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "A * B diff " << std::endl; 
	C = A * B; C_e = A_e * B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	//================================================================================
	// Quadratic form calculation test
	//================================================================================
	n_rows = 10; n_cols = 10;
	CMatrix<double> x(n_rows, 1);
	A.resize(n_rows, n_cols);
	CMatrix<double> res;
	double qf;
	for (size_t i = 0; i < n_rows; i++)
	{
		x(i, 0) = 1;
		for (size_t j = 0; j < n_cols; j++)
		{
			A(i, j) = 0;
			if (i == j)
			{
				A(i, j) = 1;
			}
		}
	}

	res = x.transposed() * A.inverse() * x; 
	
	return 0;
}