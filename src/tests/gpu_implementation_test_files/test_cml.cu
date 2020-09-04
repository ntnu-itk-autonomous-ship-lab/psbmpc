/****************************************************************************************
*
*  File name : test_cmatrix.cpp
*
*  Function  : Test file for the Cuda Matrix Library (CML) meant for use in CUDA kernels.
*			   NOTE! It is until now mainly the dynamic matrix that is tested for use. 
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

#include "cml.cuh"
#include <iostream>
#include <random>
#include "Eigen/Dense"

int main()
{
	std::random_device seed;

	std::mt19937_64 gen(seed());

	std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

	//================================================================================
	// Assignment operator + copy constructor test
	//================================================================================
	CML::MatrixXd t1(3);
	CML::MatrixXd t2(2);
	t1 = t2;

	CML::MatrixXd t3 = t1;

	//================================================================================
	// 2x2 inverse test
	//================================================================================
	size_t n_rows = 2, n_cols = 2;
	CML::MatrixXd M1(n_rows, n_cols), M1_inv = M1;
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
	std::cout << "CML 2x2 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 2x2 determinant= " << M2.determinant() << std::endl;
	std::cout << "CML 2x2 determinant= " << M1.determinant() << std::endl;

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
	
	std::cout << "CML 3x3 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 3x3 determinant= " << M2.determinant() << std::endl;
	std::cout << "CML 3x3 determinant= " << M1.determinant() << std::endl;

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
	std::cout << "CML 4x4 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 4x4 determinant= " << M2.determinant() << std::endl;
	std::cout << "CML 4x4 determinant= " << M1.determinant() << std::endl;

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
	CML::MatrixXd O = CML::MatrixXd::ones(n_rows, n_cols);
	std::cout << "Original = " << std::endl;
	std::cout << O << std::endl;

	std::cout << "Transposed = " << std::endl;
	std::cout << O.transposed() << std::endl;
	//================================================================================
	// Dot product test
	//================================================================================
	n_rows = 4; n_cols = 1;
	CML::MatrixXd v1(n_rows, n_cols), v2(n_rows, n_cols);
	Eigen::VectorXd v1_e(n_rows), v2_e(n_rows);
	for (size_t i = 0; i < n_rows; i++)
	{
		v1(i, 0) = 2 * std_norm_pdf(gen) + 5;  
		v1_e(i) = v1(i, 0); 
		v2(i) = 2 * std_norm_pdf(gen) + 5;
		v2_e(i) = v2(i);
		
	}
	std::cout << "v1' * v2 diff = " << v1.dot(v2) - v1_e.dot(v2_e) << std::endl;
	std::cout << "v2' * v1 diff = " << v2.dot(v1) - v2_e.dot(v1_e) << std::endl;
	//================================================================================
	// Cross product test
	//================================================================================
	n_rows = 3; n_cols = 1;
	v1.resize(n_rows, n_cols), v2.resize(n_rows, n_cols);
	Eigen::Vector3d v1_e3, v2_e3;
	Eigen::VectorXd v_diff1(n_rows), v_diff2(n_rows); 
	for (size_t i = 0; i < n_rows; i++)
	{
		v1(i) = 2 * std_norm_pdf(gen) + 5;  
		v1_e3(i) = v1(i); 
		v2(i) = 2 * std_norm_pdf(gen) + 5;
		v2_e3(i) = v2(i);		
	}

	for (size_t i = 0; i < n_rows; i++)
	{
		v_diff1(i) = v1.cross(v2)(i) - v1_e3.cross(v2_e3)(i);
		v_diff2(i) = v2.cross(v1)(i) - v2_e3.cross(v1_e3)(i);
	}
	std::cout << "v1 x v2 diff = " << v_diff1.transpose() << std::endl;
	std::cout << "v2 x v1 diff = " << v_diff2.transpose() << std::endl;
	//================================================================================
	// Operator tests
	//================================================================================
	n_rows = 4; n_cols = 4;
	CML::MatrixXd A(n_rows, n_cols), B(n_rows, n_cols), C;
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
	double scalar = 5 * std_norm_pdf(gen);
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

	A.set_all_coeffs(2);
	scalar = 1;
	std::cout << "A + scalar = " << std::endl; 
	std::cout << A + scalar << std::endl;

	std::cout << "scalar + A= " << std::endl; 
	std::cout << scalar + A << std::endl;

	std::cout << "A - scalar = " << std::endl; 
	std::cout << A - scalar << std::endl;

	std::cout << "scalar - A= " << std::endl; 
	std::cout << scalar - A << std::endl;

	CML::MatrixXd a_vec(A.get_rows(), 1);
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
	n_rows = 4; n_cols = 4;
	CML::MatrixXd x(n_rows, 1);
	A.resize(n_rows, n_cols);
	CML::MatrixXd res;
	A_e.resize(n_rows, n_cols);
	Eigen::VectorXd x_e(n_rows);
	double qf, qf_e;
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
	qf = x.transposed() * A.inverse() * x; 
	qf_e = x_e.transpose() * A_e.inverse() * x_e;
	std::cout << "Quadratic form diff = " << qf - qf_e << std::endl;

	//================================================================================
	// Norm and normalization test
	//================================================================================
	n_rows = 3; n_cols = 3;
	x.resize(n_rows, 1);
	x_e.resize(n_rows);
	Eigen::VectorXd r_e1(n_rows);

	A.resize(n_rows, n_cols); A_e.resize(n_rows, n_cols);
	CML::MatrixXd r2(n_rows, n_cols);
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
	CML::MatrixXd T(n_rows, n_cols), T_sub(n_rows - 3, n_cols - 3);
	T.set_zero(); T_sub.set_all_coeffs(3);
	std::cout << "T before set = " << std::endl;
	std::cout << T << std::endl;
	CML::MatrixXd row_vec(1, n_cols), col_vec(n_rows, 1);
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

	//================================================================================
	// Eigen interface tests
	//================================================================================
	Eigen::MatrixXd test1(4, 10); test1.setZero();
	CML::MatrixXd assign_to_1; 
	CML::assign_eigen_object(assign_to_1, test1);
	std::cout << assign_to_1 << std::endl;

	Eigen::Matrix<double, 1, -1> test2(1, 20); test2.setZero();
	CML::assign_eigen_object(assign_to_1, test2);
	std::cout << assign_to_1 << std::endl;

	Eigen::Matrix<int, -1, 10> test3(10, 10); test3.setZero();
	CML::MatrixXi assign_to_2;
	CML::assign_eigen_object(assign_to_2, test3);
	std::cout << assign_to_2 << std::endl;

	Eigen::VectorXd test4(10); test4.setZero();
	CML::assign_eigen_object(assign_to_1, test4);
	std::cout << assign_to_1.transposed() << std::endl;

	Eigen::Vector2d test5; test5.setZero(); 
	CML::assign_eigen_object(assign_to_1, test5);
	std::cout << assign_to_1 << std::endl;

	Eigen::Matrix2d test6; test6.setZero();
	CML::assign_eigen_object(assign_to_1, test6);
	std::cout << assign_to_1 << std::endl;
	//================================================================================
	// Other tests
	//================================================================================
	std::cout << CML::Dynamic_Matrix<double>::identity(3, 3) << std::endl;
	std::cout << CML::Dynamic_Matrix<double>::ones(3, 3) << std::endl;

	Eigen::Matrix<double, 4, 2> m42; m42.transpose();

	return 0;
}