/****************************************************************************************
*
*  File name : test_tml_pdmatrix.cu
*
*  Function  : Test file for the PDMatrix class in TML meant for use in 
*			   CUDA kernels.
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

#include "tml.cuh"
#include <iostream>
#include <random>
#include "Eigen/Dense"

// Conversion does not work with non-const parameters, because the implicit conversion is temporary.
void test_conversion(const TML::MatrixXd &m) { std::cout << m << std::endl; }

void test_conversion2(const TML::Vector3d &v3d) { std::cout << v3d << std::endl; }

void test_conversion3(const TML::PDMatrix<double, 100, 100> &pdm) { std::cout << pdm << std::endl; }


int main()
{
	std::random_device seed;

	std::mt19937_64 gen(seed());

	std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

//================================================================================
// Dynamic Matrix tests
//================================================================================	
	std::cout << "Dynamic Matrix testing..." << std::endl;
	//================================================================================
	// Assignment operator + copy constructor test
	//================================================================================
	TML::PDMatrix<double, 4, 4> t1(3);
	TML::PDMatrix<double, 4, 4> t2(2);
	t1 = t2;

	TML::PDMatrix<double, 4, 4> t3 = t1;

	//================================================================================
	// 2x2 inverse test
	//================================================================================
	size_t n_rows = 2, n_cols = 2;
	TML::PDMatrix<double, 4, 4> M1(n_rows, n_cols), M1_inv = M1;
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
	std::cout << "TML 2x2 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 2x2 determinant= " << M2.determinant() << std::endl;
	std::cout << "TML 2x2 determinant= " << M1.determinant() << std::endl;

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
	M2.resize(n_rows, n_cols); M2.setZero();
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
	
	std::cout << "TML 3x3 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 3x3 determinant= " << M2.determinant() << std::endl;
	std::cout << "TML 3x3 determinant= " << M1.determinant() << std::endl;

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
	M2.resize(n_rows, n_cols); M2.setZero();
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
	std::cout << "TML 4x4 = " << std::endl;
	std::cout << M1 << std::endl;

	std::cout << "Eigen 4x4 determinant= " << M2.determinant() << std::endl;
	std::cout << "TML 4x4 determinant= " << M1.determinant() << std::endl;

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
	TML::PDMatrix<double, 4, 2> O = TML::PDMatrix<double, 4, 2>::ones(n_rows, n_cols);
	std::cout << "Original = " << std::endl;
	std::cout << O << std::endl;

	std::cout << "Transposed = " << std::endl;
	std::cout << O.transposed() << std::endl;
	//================================================================================
	// Dot product test
	//================================================================================
	n_rows = 4; n_cols = 1;
	TML::PDVector4d v1(n_rows, n_cols), v2(n_rows, n_cols);
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
	TML::PDMatrix<double, 16, 16> A(n_rows, n_cols), B(n_rows, n_cols), C(n_rows, n_cols);
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

	std::cout << "scalar - A = " << std::endl; 
	std::cout << scalar - A << std::endl;

	TML::PDMatrix4d a_vec(A.get_rows(), 1);
	for (size_t i = 0; i < A.get_rows(); i++)
	{
		a_vec(i) = i + 1;
	}
	A.set_zero();
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
	TML::PDVector4d x(n_rows, 1);
	A.resize(n_rows, n_cols); A.set_zero();
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
	TML::PDMatrix3d r2(n_rows, n_cols);
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
	TML::PDMatrix6d T(n_rows, n_cols), T_sub(n_rows - 3, n_cols - 3);
	T.set_zero(); T_sub.set_all_coeffs(3);
	std::cout << "T before set = " << std::endl;
	std::cout << T << std::endl;
	TML::PDMatrix<double, 6, 6> row_vec(1, n_cols), col_vec(n_rows, 1);
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
	std::cout << T.get_block<4, 4>(2, 2, 4, 4) << std::endl;

	std::cout << "Block of T of size 3x3 starting at (0, 0) = " << std::endl;
	std::cout << T.get_block<3, 3>(0, 0, 3, 3) << std::endl;


	//================================================================================
	// Further tests
	//================================================================================
	TML::MatrixXd samples_d(4, 10); samples_d.set_ones();
	Eigen::MatrixXd transfer(4, 10); transfer.setZero();
	TML::PDMatrix<double, 4, 100> samples = samples_d, samples_2;
	std::cout << samples << std::endl;
	TML::assign_eigen_object(samples_2, transfer);
	std::cout << samples_2 << std::endl;
	
	TML::Static_Matrix<double, 4, 4> testc1; testc1.set_ones();
	TML::PDMatrix<double, 200, 1000> testc1_pdm(2, 2); testc1_pdm.set_all_coeffs(2.0);
	test_conversion(testc1);
	test_conversion(testc1_pdm);


	TML::MatrixXd testc2(3, 1); testc2.set_ones();
	TML::PDMatrix<double, 100, 1000> testc2_pdm(3, 1); testc2_pdm.set_all_coeffs(2.0);
	test_conversion2(testc2);
	test_conversion2(testc2_pdm);

	TML::MatrixXd testc3(3, 3); testc3.set_ones();
	TML::Vector4d testc3_sm; testc3_sm.set_all_coeffs(2.0);
	test_conversion3(testc3);
	test_conversion3(testc3_sm);


	TML::PDMatrix<double, 8, 30> bigone(1, 1);

	TML::MatrixXd assign_to(2, 20); assign_to.set_all_coeffs(3.0);
	bigone.set_block(0, 0, assign_to.get_rows(), assign_to.get_cols(), assign_to);

	std::cout << bigone << std::endl;

	assign_to.set_all_coeffs(4.0);
	bigone.set_block(2, 0, assign_to.get_rows(), assign_to.get_cols(), assign_to);

	std::cout << bigone << std::endl;

	assign_to.set_all_coeffs(5.0);
	bigone.set_block(4, 0, assign_to.get_rows(), assign_to.get_cols(), assign_to);

	std::cout << bigone << std::endl;

	std::cout << bigone.get_block<3, 3>(0, 0, 3, 3) << std::endl;

	std::cout << bigone.get_block<5, 2>(0, 0, 5, 2) << std::endl;

	std::cout << bigone.get_row(2) << std::endl;

	std::cout << bigone.get_col(2) << std::endl;

	bigone.set_block(0, 0, 3, 3, TML::Matrix3d::identity());

	std::cout << bigone << std::endl;

	TML::PDMatrix<double, 6, 4> test222(6, 3), ones(6, 1), twos(6, 1), threes(6, 1);
	ones.set_ones(); twos.set_all_coeffs(2.0); threes.set_all_coeffs(3.0);
	test222.set_col(0, ones);
	test222.set_col(1, twos);
	test222.set_col(2, threes);

	std::cout << test222 << std::endl;

	test222.shift_columns_right();

	std::cout << test222 << std::endl;

	//================================================================================
	// Testing CPE math operations such as sample expectation and sample covariance
	// calculation
	//================================================================================
	TML::PDMatrix<double, 4, 100> sres(2, 20); sres.set_all_coeffs(1);
	TML::PDMatrix4d mult_by = TML::PDMatrix4d::identity(2, 2);

	TML::Vector2d vec2d; vec2d.set_all_coeffs(3.5);
	sres = mult_by * sres + vec2d;
	std::cout << sres << std::endl;

	n_rows = 2; n_cols = 100;
	TML::PDMatrix<float, 2, 1000> es(n_rows, n_cols);
	Eigen::MatrixXf es_e(n_rows, n_cols);

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			es(i, j) = 2 * std_norm_pdf(gen) + 100;
			es_e(i, j) = es(i, j);
		}
	}
	
	Eigen::Vector2f mu_e = es_e.rowwise().mean();

	TML::Vector2f mu, mu_diff;
	mu = es.rwise_mean();
	std::cout << "TML rwise mean = " << mu << std::endl;
	std::cout << "Eigen rwise mean = " << mu_e << std::endl;

	Eigen::Matrix2f P_e = Eigen::Matrix2f::Zero();
	TML::Matrix2f P, P_diff;
	P.set_zero();
	for (size_t i = 0; i < n_cols; i++)
	{	
		P_e += (es_e.col(i) - mu_e) * (es_e.col(i) - mu_e).transpose();
		P += (es.get_col(i) - mu) * (es.get_col(i) - mu).transposed();
	}
	P_e /= (float)n_cols;
	P /= (float)n_cols;
	std::cout << "TML sample covariance = " << std::endl;
	std::cout << P << std::endl;
	std::cout << "Eigen sample covariance = " << std::endl;
	std::cout << P_e << std::endl;

	for (size_t i = 0; i < 2; i++)
	{
		mu_diff(i) = mu(i) - mu_e(i);
		for (size_t j = 0; j < 2; j++)
		{
			P_diff(i, j) = P(i, j) - P_e(i, j);
		}
	}
	std::cout << "Sample mean diff : " << mu_diff.transposed() << std::endl;
	std::cout << "Sample covariance diff: " << std::endl;
	std::cout << P_diff << std::endl;

	float alpha_n = 0.9;
	TML::Vector2f mu_prev;
	Eigen::Vector2f mu_e_prev;
	TML::Matrix2f P_prev; 
	Eigen::Matrix2f P_e_prev;
	for (int i = 0; i < 2; i++)
	{
		mu_prev(i) = 2 * std_norm_pdf(gen) + 100;
		mu_e_prev(i) = mu_prev(i);
		for (int j = 0; j < 2; j++)
		{
			P_prev(i, j) = 2 * std_norm_pdf(gen) + 100;
			P_e_prev(i, j) = P_prev(i, j);
		}
	}
	mu = alpha_n * mu + (1 - alpha_n) * mu_prev;
    P =  alpha_n * P  + (1 - alpha_n) * P_prev;

	mu_e = alpha_n * mu_e + (1 - alpha_n) * mu_e_prev;
    P_e =  alpha_n * P_e  + (1 - alpha_n) * P_e_prev;

	for (size_t i = 0; i < 2; i++)
	{
		mu_diff(i) = mu(i) - mu_e(i);
		for (size_t j = 0; j < 2; j++)
		{
			P_diff(i, j) = P(i, j) - P_e(i, j);
		}
	}
	std::cout << "Smoothing sample mean diff : " << mu_diff.transposed() << std::endl;
	std::cout << "Smoothing sample covariance diff: " << std::endl;
	std::cout << P_diff << std::endl;

	return 0;
}