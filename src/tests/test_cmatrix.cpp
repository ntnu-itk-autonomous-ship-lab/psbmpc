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
	n_rows = 6; n_cols = 1;
	CMatrix<double> v1(n_rows, n_cols), v2(n_rows, n_cols);
	Eigen::VectorXd v1_e(n_rows), v2_e(n_rows);
	for (size_t i = 0; i < 6; i++)
	{
		v1(i) = 2 * std_norm_pdf(gen) + 5;  
		v1_e(i) = v1(i);
		v2(i) = v1(i); 
		v2_e(i) = v1(i);
	}

	std::cout << "v1' * v2 diff = " << v1.dot(v2) - v1_e.dot(v2_e) << std::endl;
	std::cout << "v2' * v1 diff = " << v2.dot(v1) - v2_e.dot(v1_e) << std::endl;
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
	std::cout << "A + B diff = " << std::endl; 
	C = A + B; C_e = A_e + B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "C += A diff = " << std::endl; 
	C.set_zero(); C_e.setZero();
	C += A; C_e += A_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "A - B diff = " << std::endl; 
	C = A - B; C_e = A_e - B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "C -= A diff = " << std::endl; 
	C.set_zero(); C_e.setZero();
	C -= A; C_e -= A_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "A * B diff = " << std::endl; 
	C = A * B; C_e = A_e * B_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "scalar * A diff = " << std::endl; 
	double scalar = 5 * std_norm_pdf(gen);
	C = scalar * A; C_e = scalar * A_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	std::cout << "A * scalar diff = " << std::endl; 
	scalar = 5 * std_norm_pdf(gen);
	C = A * scalar; C_e = A_e * scalar;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			M_diff(i, j) = C(i, j) - C_e(i, j);
		}
	}
	std::cout << M_diff << std::endl;

	A.set_zero();
	CMatrix<double> a_vec(A.get_rows(), 1);
	for (size_t i = 0; i < A.get_rows(); i++)
	{
		a_vec(i) = i + 1;
	}
	std::cout << "Cwise add of vector : " << std::endl;
	std::cout << A + a_vec << std::endl;
	A.set_zero();
	std::cout << "Cwise subtract of vector : " << std::endl;
	std::cout << A - a_vec << std::endl;

	a_vec.resize(1, A.get_cols());
	for (size_t j = 0; j < A.get_cols(); j++)
	{
		a_vec(j) = j + 1;
	}
	std::cout << "Rwise add of vector : " << std::endl;
	std::cout << a_vec + A << std::endl;
	A.set_zero();
	std::cout << "Rwise subtract of vector : " << std::endl;
	std::cout << (double)-1 * a_vec + A << std::endl;



	//================================================================================
	// Quadratic form calculation test
	//================================================================================
	n_rows = 10; n_cols = 10;
	CMatrix<double> x(n_rows, 1);
	A.resize(n_rows, n_cols);
	CMatrix<double> res;
	A_e.resize(n_rows, n_cols);
	Eigen::VectorXd x_e(n_rows);
	double qf;
	for (size_t i = 0; i < n_rows; i++)
	{
		x(i) = 2 * std_norm_pdf(gen) + 5; x_e(i) = x(i);
	}
	while (A.determinant() <= 0)
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				A(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					A(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				A_e(i, j) = A(i, j);
			}
		}
	}
	res = x.transposed() * A.inverse() * x; 
	qf = x_e.transpose() * A_e.inverse() * x_e;
	std::cout << "Quadratic form diff = " << res(0, 0) - qf << std::endl;

	//================================================================================
	// Norm and normalization test
	//================================================================================
	n_rows = 3; n_cols = 3;
	x.resize(n_rows, 1);
	x_e.resize(n_rows);
	Eigen::VectorXd r_e1(n_rows);

	A.resize(n_rows, n_cols); A_e.resize(n_rows, n_cols);
	CMatrix<double> r2(n_rows, n_cols);
	Eigen::MatrixXd r2_e(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		x(i) = 2 * std_norm_pdf(gen) + 5; x_e(i) = x(i);
		for (size_t j = 0; j < n_cols; j++)
		{
			A(i, j) = 2 * std_norm_pdf(gen) + 5;
			A_e(i, j) = A(i, j);
		}
	}
	std::cout << "x unnormalized = " << x.transposed() << std::endl;
	std::cout << "x normalized diff = ";
	for (size_t i = 0; i < n_rows; i++)
	{
		r_e1(i) = x.normalized()(i) - x_e.normalized()(i);
	}
	std::cout << r_e1.transpose() << std::endl;
	std::cout << "||x||_2 diff = " << x.norm() - x_e.norm() << std::endl;

	std::cout << "A" << std::endl;
	std::cout << A << std::endl;
	std::cout << "||A||_2 diff = " << A.norm() - A_e.norm() << std::endl;

	//================================================================================
	// Set function tests
	//================================================================================
	n_rows = 6; n_cols = 6;
	CMatrix<double> T(n_rows, n_cols), T_sub(n_rows - 3, n_cols - 3);
	T.set_zero(); T_sub.set_all_coeffs(3);
	std::cout << "T before set = " << std::endl;
	std::cout << T << std::endl;
	CMatrix<double> row_vec(1, n_cols), col_vec(n_rows, 1);
	for (size_t i = 0; i < n_rows; i++)
	{
		col_vec(i) = 1;
	}
	for (size_t j = 0; j < n_cols; j++)
	{
		row_vec(j) = 2;
	}
	std::cout << "T after set col = " << std::endl;
	T.set_col(0, col_vec);
	std::cout << T << std::endl;

	std::cout << "T after set row = " << std::endl;
	T.set_row(0, row_vec);
	std::cout << T << std::endl;

	std::cout << "T after set block = " << std::endl;
	T.set_block(3, 3, n_rows - 3, n_cols - 3, T_sub);
	std::cout << T << std::endl;

	std::cout << "Row 3 of T = " << std::endl;
	std::cout << T.get_row(3) << std::endl;

	std::cout << "Column 3 of T = " << std::endl;
	std::cout << T.get_col(3) << std::endl;

	std::cout << "Block of T of size 4x4 starting at (2, 2) = " << std::endl;
	std::cout << T.get_block(2, 2, 4, 4) << std::endl;

	std::cout << "Block of T of size 3x3 starting at (0, 0) = " << std::endl;
	std::cout << T.get_block(0, 0, 3, 3) << std::endl;

	return 0;
}