#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "tml/tml.cuh"
#include <iostream>
#include <random>
#include <Eigen/Dense>

// Conversion does not work with non-const parameters, because the implicit conversion is temporary.
void test_conversion2(const TML::Vector3d &v3d) { std::cout << v3d << std::endl; }

void test_conversion3(const TML::PDMatrix<double, 100, 100> &pdm) { std::cout << pdm << std::endl; }


int main()
{
	std::random_device seed;

	std::mt19937_64 gen(seed());

	std::normal_distribution<double> std_norm_pdf(0.0, 1.0);

//================================================================================
// Static Matrix tests
//================================================================================
	std::cout << "Static Matrix testing..." << std::endl;

	//================================================================================
	// 2x2 inverse test
	//================================================================================
	TML::Matrix2d M2d, M2d_inv = M2d;
	Eigen::Matrix2d M2e; 
	Eigen::Matrix2d M_diff2;

	for (size_t i = 0; i < 2; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			M2e(i, j) = 2 * std_norm_pdf(gen) + 5;
			if (i == j)
			{
				M2e(i, j) = 2 * std_norm_pdf(gen) + 10;
			}
			M2d(i, j) = M2e(i, j);
		}
	}
	std::cout << "TML 2x2 = " << std::endl;
	std::cout << M2d << std::endl;

	std::cout << "Eigen 2x2 determinant= " << M2e.determinant() << std::endl;
	std::cout << "TML 2x2 determinant= " << M2d.determinant() << std::endl;

	M2d_inv = M2d.inverse();

	for (size_t i = 0; i < 2; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			M_diff2(i, j) = M2d_inv(i, j) - M2e.inverse()(i, j);
		}
	}
	std::cout << "Difference in 2x2 inverse: " << std::endl;
	std::cout << M_diff2 << std::endl;
	
	//================================================================================
	// 3x3 inverse test
	//================================================================================
	TML::Matrix3d M3d, M3d_inv = M3d;
	Eigen::Matrix3d M3; 
	Eigen::Matrix3d M_diff3;
	while (M3d.determinant() <= 0)
	{
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0; j < 3; j++)
			{
				M3(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					M3(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				M3d(i, j) = M3(i, j);
			}
		}
	}
	
	std::cout << "TML 3x3 = " << std::endl;
	std::cout << M3d << std::endl;

	std::cout << "Eigen 3x3 determinant= " << M3.determinant() << std::endl;
	std::cout << "TML 3x3 determinant= " << M3d.determinant() << std::endl;

	M3d_inv = M3d.inverse();
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			M_diff3(i, j) = M3d_inv(i, j) - M3.inverse()(i, j);
		}
	}
	std::cout << "Difference in 3x3 inverse: " << std::endl;
	std::cout << M_diff3 << std::endl;
	//================================================================================
	// 4x4 inverse test
	//================================================================================
	TML::Matrix4d M4d, M4d_inv = M4d;
	Eigen::Matrix4d M4; 
	Eigen::Matrix4d M_diff4;

	//M2 = kf->get_covariance();
	while (M4d.determinant() <= 0)
	{
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				M4(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					M4(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				M4d(i, j) = M4(i, j);
			}
		}
	}
	std::cout << "TML 4x4 = " << std::endl;
	std::cout << M4d << std::endl;

	std::cout << "Eigen 4x4 determinant = " << M4.determinant() << std::endl;
	std::cout << "TML 4x4 determinant = " << M4d.determinant() << std::endl;

	M4d_inv = M4d.inverse();

	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			M_diff4(i, j) = M4d_inv(i, j) - M4.inverse()(i, j);
		}
	}
	std::cout << "Difference in 4x4 inverse: " << std::endl;
	std::cout << M_diff4 << std::endl;
	//================================================================================
	// Transpose test
	//================================================================================
	TML::Static_Matrix<double, 4, 2> s_t = TML::Static_Matrix<double, 4, 2>::ones();
	std::cout << "Original static matrix = " << std::endl;
	std::cout << s_t << std::endl;

	std::cout << "Transposed = " << std::endl;
	std::cout << s_t.transposed() << std::endl;
	//================================================================================
	// Dot product test
	//================================================================================
	TML::Vector4d v1s, v2s;
	Eigen::Vector4d v1s_e, v2s_e;
	for (size_t i = 0; i < 4; i++)
	{
		v1s(i) = 2 * std_norm_pdf(gen) + 5;  
		v1s_e(i) = v1s(i); 
		v2s(i) = 2 * std_norm_pdf(gen) + 5;
		v2s_e(i) = v2s(i);
	}
	std::cout << "v1' * v2 diff = " << v1s.dot(v2s) - v1s_e.dot(v2s_e) << std::endl;
	std::cout << "v2' * v1 diff = " << v2s.dot(v1s) - v2s_e.dot(v1s_e) << std::endl;
	//================================================================================
	// Cross product test
	//================================================================================
	TML::Vector3d v1c, v2c;
	Eigen::Vector3d v1s_e3, v2s_e3;
	Eigen::Vector3d v_diff1s, v_diff2s; 
	for (size_t i = 0; i < 3; i++)
	{
		v1c(i) = 2 * std_norm_pdf(gen) + 5;  
		v1s_e3(i) = v1c(i); 
		v2c(i) = 2 * std_norm_pdf(gen) + 5;
		v2s_e3(i) = v2c(i);		
	}

	for (size_t i = 0; i < 3; i++)
	{
		v_diff1s(i) = v1c.cross(v2c)(i) - v1s_e3.cross(v2s_e3)(i);
		v_diff2s(i) = v2c.cross(v1c)(i) - v2s_e3.cross(v1s_e3)(i);
	}
	std::cout << "v1 x v2 diff = " << v_diff1s.transpose() << std::endl;
	std::cout << "v2 x v1 diff = " << v_diff2s.transpose() << std::endl;
	//================================================================================
	// Operator tests
	//================================================================================
	int n_rows = 4; int n_cols = 4;
	TML::Matrix4d As, Bs, Cs;
	Eigen::Matrix4d As_e, Bs_e, Cs_e, Ms_diff;

	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			As(i, j) = 2 * std_norm_pdf(gen) + 5;
			As_e(i, j) = As(i, j);

			Bs(i, j) = 2 * std_norm_pdf(gen) + 5;
			Bs_e(i, j) = Bs(i, j);
		}
	}
	int scalar = 5 * std_norm_pdf(gen);
	std::cout << "A + B diff = " << std::endl; 
	Cs = As + Bs; Cs_e = As_e + Bs_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			Ms_diff(i, j) = Cs(i, j) - Cs_e(i, j);
		}
	}
	std::cout << Ms_diff << std::endl;

	std::cout << "C += A diff = " << std::endl; 
	Cs.set_zero(); Cs_e.setZero();
	Cs += As; Cs_e += As_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			Ms_diff(i, j) = Cs(i, j) - Cs_e(i, j);
		}
	}
	std::cout << Ms_diff << std::endl;

	std::cout << "A - B diff = " << std::endl; 
	Cs = As - Bs; Cs_e = As_e - Bs_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			Ms_diff(i, j) = Cs(i, j) - Cs_e(i, j);
		}
	}
	std::cout << Ms_diff << std::endl;

	std::cout << "C -= A diff = " << std::endl; 
	Cs.set_zero(); Cs_e.setZero();
	Cs -= As; Cs_e -= As_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			Ms_diff(i, j) = Cs(i, j) - Cs_e(i, j);
		}
	}
	std::cout << Ms_diff << std::endl;

	std::cout << "A * B diff = " << std::endl; 
	Cs = As * Bs; Cs_e = As_e * Bs_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		for (size_t j = 0; j < n_cols; j++)
		{
			Ms_diff(i, j) = Cs(i, j) - Cs_e(i, j);
		}
	}
	std::cout << Ms_diff << std::endl;

	As.set_all_coeffs(2);
	scalar = 1;
	std::cout << "A + scalar = " << std::endl; 
	std::cout << As + scalar << std::endl;

	std::cout << "scalar + A = " << std::endl; 
	std::cout << scalar + As << std::endl;

	std::cout << "A - scalar = " << std::endl; 
	std::cout << As - scalar << std::endl;

	std::cout << "scalar - A = " << std::endl; 
	std::cout << scalar - As << std::endl;

	//================================================================================
	// Quadratic form calculation test
	//================================================================================
	n_rows = 4; n_cols = 4;
	TML::Vector4d xs;
	Eigen::Vector4d xs_e;
	double qfs, qfs_e;
	for (size_t i = 0; i < n_rows; i++)
	{
		xs(i) = 2 * std_norm_pdf(gen) + 5; xs_e(i) = xs(i);
	}
	while (As.determinant() <= 0)
	{
		for (size_t i = 0; i < n_rows; i++)
		{
			for (size_t j = 0; j < n_cols; j++)
			{
				As(i, j) = 2 * std_norm_pdf(gen) + 5;
				if (i == j || (i + j) % 2 == 0)
				{
					As(i, j) = 2 * std_norm_pdf(gen) + 20;
				}
				As_e(i, j) = As(i, j);
			}
		}
	}
	qfs = xs.transposed() * As.inverse() * xs; 
	qfs_e = xs_e.transpose() * As_e.inverse() * xs_e;
	std::cout << "Quadratic form diff = " << qfs - qfs_e << std::endl;

	//================================================================================
	// Norm and normalization test
	//================================================================================
	n_rows = 3; n_cols = 3;
	TML::Vector3d xs3;
	Eigen::Vector3d xs3_e;
	Eigen::Vector3d rs_e3;

	TML::Matrix3d A2s, r3s;
	Eigen::Matrix3d A2s_e, r3s_e;

	for (size_t i = 0; i < n_rows; i++)
	{
		xs3(i) = 2 * std_norm_pdf(gen) + 5; xs3_e(i) = xs3(i);
		for (size_t j = 0; j < n_cols; j++)
		{
			A2s(i, j) = 2 * std_norm_pdf(gen) + 5;
			A2s_e(i, j) = A2s(i, j);
		}
	}
	std::cout << "x unnormalized = " << xs3.transposed() << std::endl;
	std::cout << "x normalized diff = ";
	for (size_t i = 0; i < n_rows; i++)
	{
		rs_e3(i) = xs3.normalized()(i) - xs3_e.normalized()(i);
	}
	std::cout << rs_e3.transpose() << std::endl;

	std::cout << "A" << std::endl;
	std::cout << A2s << std::endl;
	std::cout << "||A||_2 diff = " << A2s.norm() - A2s_e.norm() << std::endl;

	//================================================================================
	// Set function tests
	//================================================================================
	n_rows = 6; n_cols = 6;
	TML::Static_Matrix<double, 6, 6> Ts;
	TML::Static_Matrix<double, 3, 3> Ts_sub;
	Ts.set_zero(); Ts_sub.set_all_coeffs(3);
	std::cout << "T before set = " << std::endl;
	std::cout << Ts << std::endl;

	TML::Static_Matrix<double, 1, 6> rvec_s;
	TML::Vector6d cvec_s;
	for (size_t j = 0; j < n_cols; j++)
	{
		cvec_s(j) = 1;
	}

	std::cout << "T after set col = " << std::endl;
	Ts.set_col(0, cvec_s);
	std::cout << Ts << std::endl;

	std::cout << "T after set row = " << std::endl;
	Ts.set_row(0, rvec_s);
	std::cout << Ts << std::endl;

	std::cout << "Row 3 of T = " << std::endl;
	std::cout << Ts.get_row(3) << std::endl;

	std::cout << "Column 3 of T = " << std::endl;
	std::cout << Ts.get_col(3) << std::endl;

	std::cout << "Block of T of size 4x4 starting at (2, 2) = " << std::endl;
	std::cout << Ts.get_block<4, 4>(2, 2) << std::endl;

	std::cout << "Block of T of size 3x3 starting at (0, 0) = " << std::endl;
	std::cout << Ts.get_block<3, 3>(0, 0) << std::endl;
	return 0;
}