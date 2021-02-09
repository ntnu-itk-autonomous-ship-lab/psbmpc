/****************************************************************************************
*
*  File name : utilities.cuh
*
*  Function  : Header file for all-purpose math functions which are used by multiple 
*			   library files. Thus, do NOT add a function here if it belongs to one 
*			   distinct class. Overloads for eigen and tml type matrices.
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

#ifndef _UTILITIES_CUH_
#define _UTILITIES_CUH_

#include "psbmpc_defines.h"
#include <thrust/device_vector.h>
#include "Eigen/Dense"
#include "tml.cuh"
#include <iostream>
#include <fstream>
#include <cstdlib>

enum Axis 
	{
		Roll,
		Pitch,
		Yaw
	};

/****************************************************************************************
*  Place inline functions here	
****************************************************************************************/
/****************************************************************************************
*  Name     : save_matrix_to_file
*  Function : Two overloads
*  Author   :
*  Modified :
*****************************************************************************************/
inline void save_matrix_to_file(const Eigen::MatrixXd &in)
{
    std::ofstream outdata("/home/trymte/Desktop/thecolavrepo/psbmpc_cxx/src/matlab_scripts/matrix.csv", std::ofstream::trunc);
    int n_rows = in.rows();
    int n_cols = in.cols();

    if(!outdata) 
    {
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    }
    //outdata << n_rows << " " << n_cols << std::endl;
    for (int i = 0; i < n_rows; i++)
    {
        for (int j = 0; j < n_cols; j++)
        {
            outdata << in(i, j);
            if (j != n_cols - 1) { outdata << ","; }
        }
        if (i != n_rows - 1) { outdata << std::endl; }
    }
}

template <class T, size_t Max_Rows, size_t Max_Cols>
inline void save_matrix_to_file(const TML::PDMatrix<T, Max_Rows, Max_Cols> &in)
{
    std::ofstream outdata("/home/trymte/Desktop/thecolavrepo/psbmpc_cxx/src/matlab_scripts/matrix.csv", std::ofstream::trunc);
    int n_rows = in.get_rows();
    int n_cols = in.get_cols();

    if(!outdata) 
    {
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    }
    //outdata << n_rows << " " << n_cols << std::endl;
    for (int i = 0; i < n_rows; i++)
    {
        for (int j = 0; j < n_cols; j++)
        {
            outdata << in(i, j);
            if (j != n_cols - 1) { outdata << ","; }
        }
        if (i != n_rows - 1) { outdata << std::endl; }
    }
}

/****************************************************************************************
*  Name     : read_matrix_from_file
*  Function : 
*  Author   :
*  Modified :
*****************************************************************************************/
template <class T, size_t Max_Rows, size_t Max_Cols>
inline TML::PDMatrix<T, Max_Rows, Max_Cols> read_matrix_from_file(const size_t n_rows, const size_t n_cols)
{
    TML::PDMatrix<T, Max_Rows, Max_Cols> out(n_rows, n_cols);
    std::ifstream indata("/home/trymte/Desktop/thecolavrepo/psbmpc_cxx/src/matlab_scripts/matrix.csv");

    if(!indata) 
    {
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    }

    //indata >> n_rows >> n_cols;
    for (int i = 0; i < n_rows; i++)
    {
         for (int j = 0; j < n_cols; j++)
         {
             indata >> out(i, j);
         }
    }
    return out;
}


/****************************************************************************************
*  Name     : wrap_angle_to_pmpi
*  Function : Shifts angle into [-pi, pi] interval
*  Author   :
*  Modified :
*****************************************************************************************/
inline double wrap_angle_to_pmpi(const double angle) 
{
	double a = fmod(angle, 2 * M_PI);
	if (a <= -M_PI) a += 2 * M_PI;
	if (a > M_PI) a -= 2 * M_PI;
	return a;
}

__host__ __device__ inline float wrap_angle_to_pmpi(const float angle) 
{
	float a = fmod(angle, 2 * M_PI);
	if (a <= -M_PI) a += 2 * M_PI;
	if (a > M_PI) a -= 2 * M_PI;
	return a;
}

/****************************************************************************************
*  Name     : wrap_angle_to_pmpi
*  Function : Shifts angle into [0, 2pi] interval
*  Author   :
*  Modified :
*****************************************************************************************/
inline double wrap_angle_to_02pi(const double angle) 
{
	double a = fmod(angle, 2 * M_PI);
	if (a < 0) a += 2 * M_PI;
	return a;
}

__host__ __device__ inline float wrap_angle_to_02pi(const float angle) 
{
	float a = fmod(angle, 2 * M_PI);
	if (a < 0) a += 2 * M_PI;
	return a;
}

/****************************************************************************************
*  Name     : angle_difference_pmpi
*  Function : Makes sure angle difference a_1 - a_2 is within [-pi, pi] interval
*  Author   :
*  Modified :
*****************************************************************************************/
inline double angle_difference_pmpi(const double a_1, const double a_2) 
{
	double diff = wrap_angle_to_pmpi(a_1) - wrap_angle_to_pmpi(a_2);
	while (diff > M_PI) diff -= 2 * M_PI;
	while (diff < -M_PI) diff += 2 * M_PI;
	return diff;
}

__host__ __device__ inline float angle_difference_pmpi(const float a_1, const float a_2) 
{
	float diff = wrap_angle_to_pmpi(a_1) - wrap_angle_to_pmpi(a_2);
	while (diff > M_PI) diff -= 2 * M_PI;
	while (diff < -M_PI) diff += 2 * M_PI;
	return diff;
}

/****************************************************************************************
*  Name     : rotate_vector_2D
*  Function : Rotates two-dimensional vectory v by angle, around z-axis here
*  Author   :
*  Modified :
*****************************************************************************************/
inline Eigen::Vector2d rotate_vector_2D(const Eigen::Vector2d &v, const double angle)
{
	Eigen::Vector2d v_temp;
	v_temp(0) = v(0) * cos(angle) - v(1) * sin(angle);
	v_temp(1) = v(0) * sin(angle) - v(1) * cos(angle);
	return v_temp;
}

__host__ __device__ inline TML::Vector2f rotate_vector_2D(const TML::Vector2f &v, const float angle)
{
	TML::Vector2f v_temp;
	v_temp(0) = v(0) * cos(angle) - v(1) * sin(angle);
	v_temp(1) = v(0) * sin(angle) - v(1) * cos(angle);
	return v_temp;
}

/****************************************************************************************
*  Name     : rotate_vector_3D
*  Function : Rotates three-dimensional vectory v by angle, around specified axis
*  Author   :
*  Modified :
*****************************************************************************************/
inline Eigen::Vector3d rotate_vector_3D(const Eigen::Vector3d &v, const double angle, const Axis axis)
{
	Eigen::Vector3d v_temp;
	switch (axis) 
	{
		case Roll : 
		{
			v_temp(0) = v(0);
			v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
			v_temp(2) = v(1) * sin(angle) + v(2) * cos(angle);
			break;
		}
		case Pitch : 
		{
			v_temp(0) = v(0) * cos(angle) + v(2) * sin(angle);
			v_temp(1) = v(1);
			v_temp(2) = - v(0) * sin(angle) + v(2) * cos(angle);	
			break;
		}			
		case Yaw : 
		{
			v_temp(0) = v(0) * cos(angle) - v(1) * sin(angle);
			v_temp(1) = v(0) * sin(angle) + v(1) * cos(angle);
			v_temp(2) = v(2);
			break;
		}
		default : std::cout << "Invalid axis specified!" << std::endl;
	}
	return v_temp;
}

__host__ __device__ inline TML::Vector3f rotate_vector_3D(const TML::Vector3f &v, const float angle, const Axis axis)
{
	assert(v.get_rows() == 3 && v.get_cols() == 1);
	TML::Vector3f v_temp;
	switch (axis) 
	{
		case Roll : 
		{
			v_temp(0) = v(0);
			v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
			v_temp(2) = v(1) * sin(angle) + v(2) * cos(angle);
			break;
		}
		case Pitch : 
		{
			v_temp(0) = v(0) * cos(angle) + v(2) * sin(angle);
			v_temp(1) = v(1);
			v_temp(2) = - v(0) * sin(angle) + v(2) * cos(angle);	
			break;
		}			
		case Yaw : 
		{
			v_temp(0) = v(0) * cos(angle) - v(1) * sin(angle);
			v_temp(1) = v(0) * sin(angle) + v(1) * cos(angle);
			v_temp(2) = v(2);
			break;
		}
		default : v_temp.set_zero(); return v_temp;
	}
	return v_temp;
}

/****************************************************************************************
*  Name     : flatten
*  Function : Flattens a matrix of size n_rows x n_cols to n_rows * n_cols x 1
*  Author   :
*  Modified :
*****************************************************************************************/
inline Eigen::MatrixXd flatten(const Eigen::MatrixXd &in)
{
	int n_rows = in.rows();
	int n_cols = in.cols();

	Eigen::MatrixXd out;
	out.resize(n_rows * n_cols, 1);
	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(count) = in(i, j);
			count += 1;
		}
	}
	return out;
}

__host__ __device__ inline TML::MatrixXf flatten(const TML::MatrixXf &in)
{
	int n_rows = in.get_rows();
	int n_cols = in.get_cols();

	TML::MatrixXf out(n_rows * n_cols, 1);
	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(count) = in(i, j);
			count += 1;
		}
	}
	return out;
}

template <size_t Rows, size_t Cols>
__host__ __device__ inline TML::Static_Matrix<float, Rows * Cols, 1> flatten(const TML::Static_Matrix<float, Rows, Cols> &in)
{
	TML::Static_Matrix<float, Rows * Cols, 1> out;
	int count = 0;
	for(int i = 0; i < Rows; i++)
	{
		for(int j = 0; j < Cols; j++)
		{
			out(count) = in(i, j);
			count += 1;
		}
	}
	return out;
}

template <size_t Max_Rows, size_t Max_Cols>
__host__ __device__ inline TML::PDMatrix<float, Max_Rows * Max_Cols, 1> flatten(const TML::PDMatrix<float, Max_Rows, Max_Cols> &in)
{
	int n_rows = in.get_rows();
	int n_cols = in.get_cols();

	TML::PDMatrix<float, Max_Rows * Max_Cols, 1> out(n_rows, n_cols);
	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(count) = in(i, j);
			count += 1;
		}
	}
	return out;
}

/****************************************************************************************
*  Name     : reshape
*  Function : Reshapes a vector of size n_rows * n_cols x 1 to  a matrix of 
*			  n_rows x n_cols 
*  Author   :
*  Modified :
*****************************************************************************************/
inline Eigen::MatrixXd reshape(const Eigen::VectorXd &in, const int n_rows, const int n_cols)
{
	Eigen::MatrixXd out;
	out.resize(n_rows, n_cols);
	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(i, j) = in(count);
			count += 1;
		}
	}
	return out;
}

__host__ __device__ inline TML::MatrixXd reshape(const TML::MatrixXd &in, const int n_rows, const int n_cols)
{
	TML::MatrixXd out(n_rows, n_cols);

	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(i, j) = in(count);
			count += 1;
		}
	}
	return out;
}

template <size_t Rows, size_t Cols, size_t New_Rows, size_t New_Cols>
__host__ __device__ inline TML::Static_Matrix<float, New_Rows, New_Cols> reshape(const TML::Static_Matrix<float, Rows, Cols> &in)
{
	TML::Static_Matrix<float, New_Rows, New_Cols> out;

	int count = 0;
	for(int i = 0; i < New_Rows; i++)
	{
		for(int j = 0; j < New_Cols; j++)
		{
			out(i, j) = in(count);
			count += 1;
		}
	}
	return out;
}

template <size_t Max_Rows, size_t Max_Cols, size_t New_Max_Rows, size_t New_Max_Cols>
__host__ __device__ inline TML::PDMatrix<float, New_Max_Rows, New_Max_Cols> reshape(const TML::PDMatrix<float, Max_Rows, Max_Cols> &in, const int n_rows, const int n_cols)
{
	assert(n_rows <= New_Max_Rows && n_cols <= New_Max_Cols);
	TML::PDMatrix<float, New_Max_Rows, New_Max_Cols> out(n_rows, n_cols);

	int count = 0;
	for(int i = 0; i < n_rows; i++)
	{
		for(int j = 0; j < n_cols; j++)
		{
			out(i, j) = in(count);
			count += 1;
		}
	}
	return out;
}

/****************************************************************************************
*  Name     : calculate_cpa
*  Function : Calculates time, distance (vector) and position (vector) at the Closest 
*			  Point of Approach between vessel A and B
*  Author   : 
*  Modified :
*****************************************************************************************/
inline void calculate_cpa(
	Eigen::Vector2d &p_cpa, 												// In/out: Position of vessel A at CPA
	double &t_cpa, 															// In/out: Time to CPA
	double &d_cpa, 															// In/out: Distance at CPA
	const Eigen::VectorXd &xs_A, 											// In: State of vessel A 
	const Eigen::VectorXd &xs_B 											// In: State of vessel B
	)
{
	double epsilon = 0.25; // lower boundary on relative speed to calculate t_cpa "safely"
	double psi_A(0.0), psi_B(0.0);
	Eigen::Vector2d v_A, v_B, p_A, p_B, L_AB;
	p_A = xs_A.block<2, 1>(0, 0); p_B = xs_B.block<2, 1>(0, 0);
	if (xs_A.size() == 6) { psi_A = xs_A(2); v_A(0) = xs_A(3); v_A(1) = xs_A(4); rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B(2); v_B(1) = xs_B(4); v_B(1) = xs_B(4); rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	// Check if the relative speed is too low, or if the vessels are moving away from each other with the current velocity vectors
	double dt_incr = 0.1;
	if ((v_A - v_B).norm() < epsilon || (p_A - p_B).norm() < ((p_A + v_A * dt_incr) - (p_B + v_B * dt_incr)).norm())
	{
		t_cpa = 0;
		p_cpa = p_A;
		d_cpa = (p_A - p_B).norm();
	}
	else
	{
		t_cpa = - (p_A - p_B).dot(v_A - v_B) / pow((v_A - v_B).norm(), 2);
		p_cpa = p_A + v_A * t_cpa;
		d_cpa = (p_cpa - (p_B + v_B * t_cpa)).norm();
	}
}

__host__ __device__ inline void calculate_cpa(
	TML::Vector2f &p_cpa, 													// In/out: Position of vessel A at CPA
	float &t_cpa, 															// In/out: Time to CPA
	float &d_cpa, 															// In/out: Distance at CPA
	const TML::PDVector6f &xs_A, 											// In: State of vessel A 
	const TML::PDVector6f &xs_B 											// In: State of vessel B
	)
{
	float epsilon = 0.25; // lower boundary on relative speed to calculate t_cpa "safely"
	float psi_A(0.0f), psi_B(0.0f);
	TML::Vector2f v_A, v_B, p_A, p_B, L_AB;
	p_A(0) = xs_A(0); p_A(1) = xs_A(1);
	p_B(0) = xs_B(0); p_B(1) = xs_B(1);
	if (xs_A.size() == 6) { psi_A = xs_A[2]; v_A(0) = xs_A(3); v_A(1) = xs_A(4); rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B[2]; v_B(1) = xs_B(4); v_B(1) = xs_B(4); rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	// Check if the relative speed is too low, or if the vessels are moving away from each other with the current velocity vectors
	float dt_incr = 0.1;
	if ((v_A - v_B).norm() < epsilon || (p_A - p_B).norm() < ((p_A + v_A * dt_incr) - (p_B + v_B * dt_incr)).norm())
	{
		t_cpa = 0;
		p_cpa = p_A;
		d_cpa = (p_A - p_B).norm();
	}
	else
	{
		t_cpa = - (p_A - p_B).dot(v_A - v_B) / powf((v_A - v_B).norm(), 2);
		p_cpa = p_A + v_A * t_cpa;
		d_cpa = (p_cpa - (p_B + v_B * t_cpa)).norm();
	}
}

__host__ __device__ inline void calculate_cpa(
	TML::Vector2d &p_cpa, 													// In/out: Position of vessel A at CPA
	double &t_cpa, 															// In/out: Time to CPA
	double &d_cpa, 															// In/out: Distance at CPA
	const TML::PDVector6f &xs_A, 											// In: State of vessel A 
	const TML::PDVector6f &xs_B 											// In: State of vessel B
	)
{

	float epsilon = 0.25; // lower boundary on relative speed to calculate t_cpa "safely"
	float psi_A(0.0f), psi_B(0.0f);
	TML::Vector2f v_A, v_B, p_A, p_B, L_AB;
	p_A(0) = xs_A(0); p_A(1) = xs_A(1);
	p_B(0) = xs_B(0); p_B(1) = xs_B(1);
	if (xs_A.size() == 6) { psi_A = xs_A[2]; v_A(0) = xs_A(3); v_A(1) = xs_A(4); rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B[2]; v_B(1) = xs_B(4); v_B(1) = xs_B(4); rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	// Check if the relative speed is too low, or if the vessels are moving away from each other with the current velocity vectors
	float dt_incr = 0.1;
	if ((v_A - v_B).norm() < epsilon || (p_A - p_B).norm() < ((p_A + v_A * dt_incr) - (p_B + v_B * dt_incr)).norm())
	{
		t_cpa = 0;
		p_cpa = p_A;
		d_cpa = (p_A - p_B).norm();
	}
	else
	{
		t_cpa = - (p_A - p_B).dot(v_A - v_B) / powf((v_A - v_B).norm(), 2);
		p_cpa = p_A + v_A * t_cpa;
		d_cpa = (p_cpa - (p_B + v_B * t_cpa)).norm();
	}
}

/****************************************************************************************
*  Name     : determine_COLREGS_violation
*  Function : Determine if vessel A violates COLREGS with respect to vessel B.
*			  Temporary solution until SBMPC class is implemented for the obstacles.
*			  Extra functionality to avoid obstacle being penalized for keeping stand-on
*			  duty in certain situations, in the PSB-MPC predictions.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
inline bool determine_COLREGS_violation(
	const Eigen::VectorXd &xs_A,											// In: State vector of vessel A (most often the ownship)
	const Eigen::VectorXd &xs_B, 											// In: State vector of vessel B (most often an obstacle)
	const double phi_AH,
	const double phi_OT, 
	const double phi_HO,
	const double phi_CR,
	const double d_close,
	const double d_safe
	)
{
	bool B_is_starboard(false), A_is_overtaken(false), B_is_overtaken(false);
	bool is_ahead(false), is_close(false), is_passed(false), is_head_on(false), is_crossing(false);

	Eigen::Vector2d v_A, v_B, L_AB;
	double psi_A(0.0), psi_B(0.0);
	if (xs_A.size() == 6) { psi_A = xs_A(2); v_A(0) = xs_A(3); v_A(1) = xs_A(4); v_A = rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B(2); v_B(0) = xs_B(3); v_B(1) = xs_B(4); v_B = rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	L_AB(0) = xs_B(0) - xs_A(0);
	L_AB(1) = xs_B(1) - xs_A(1);
	double d_AB = L_AB.norm();
	L_AB = L_AB / L_AB.norm();

	is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	is_close = d_AB <= d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  		&&
					v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							  		&&
					v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;
	
	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > d_safe;

	is_head_on = v_A.dot(v_B) < - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25											&&
				is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  	&&
				v_A.dot(L_AB) > cos(90 * DEG2RAD) * v_A.norm()				&& 	// Its not a crossing situation if A's velocity vector points away from vessel B  
				v_A.norm() > 0.25											&&	// or does not make a crossing occur. This is not mentioned in Johansen.
				v_B.norm() > 0.25											&&
				!is_head_on 												&&
				!is_passed;

	// Extra condition that the COLREGS violation is only considered in an annulus; i.e. within d_close but outside d_safe.
	// The logic behind having to be outside d_safe is that typically a collision happens here, and thus COLREGS should be disregarded
	// in order to make a safe reactive avoidance maneuver, if possible. 
	// NOTE that this extra condition is only applied for "dumb"
	// obstacle predictions where one assumes neither the own-ship nor the obstacle makes avoidance maneuvers. 
	bool mu = is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken)) && (d_AB > d_safe);

	return mu;
}

__host__ __device__ inline bool determine_COLREGS_violation(
	const TML::PDVector6d &xs_A,											// In: State vector of vessel A (most often the ownship)
	const TML::PDVector6d &xs_B, 											// In: State vector of vessel B (most often an obstacle)
	const float phi_AH,
	const float phi_OT, 
	const float phi_HO,
	const float phi_CR,
	const float d_close,
	const float d_safe
	)
{
	bool B_is_starboard(false), A_is_overtaken(false), B_is_overtaken(false);
	bool is_ahead(false), is_close(false), is_passed(false), is_head_on(false), is_crossing(false);

	TML::Vector2f v_A, v_B, L_AB;
	float psi_A(0.0f), psi_B(0.0f);
	if (xs_A.size() == 6) { psi_A = xs_A(2); v_A(0) = xs_A(3); v_A(1) = xs_A(4); v_A = rotate_vector_2D(v_A, psi_A); }
	else 				  { psi_A = atan2(xs_A(3), xs_A(2)); v_A(0) = xs_A(2); v_A(1) = xs_A(3); }
	
	if (xs_B.size() == 6) { psi_B = xs_B(2); v_B(0) = xs_B(3); v_B(1) = xs_B(4); v_B = rotate_vector_2D(v_B, psi_B); }
	else 				  { psi_B = atan2(xs_B(3), xs_B(2)); v_B(0) = xs_B(2); v_B(1) = xs_B(3); }

	L_AB(0) = xs_B(0) - xs_A(0);
	L_AB(1) = xs_B(1) - xs_A(1);
	float d_AB = L_AB.norm();
	L_AB = L_AB / L_AB.norm();

	is_ahead = v_A.dot(L_AB) > cos(phi_AH) * v_A.norm();

	is_close = d_AB <= d_close;

	A_is_overtaken = v_A.dot(v_B) > cos(phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  		&&
					v_A.norm() > 0.25;

	B_is_overtaken = v_B.dot(v_A) > cos(phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							  		&&
					v_B.norm() > 0.25;

	B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;
	
	is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
				!A_is_overtaken) 											||
				(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
				!B_is_overtaken)) 											&&
				d_AB > d_safe;

	is_head_on = v_A.dot(v_B) < - cos(phi_HO) * v_A.norm() * v_B.norm() 	&&
				v_A.norm() > 0.25											&&
				v_B.norm() > 0.25											&&
				is_ahead;

	is_crossing = v_A.dot(v_B) < cos(phi_CR) * v_A.norm() * v_B.norm()  	&&
				v_A.dot(L_AB) > cos(90 * DEG2RAD) * v_A.norm()				&& 	// Its not a crossing situation if A's velocity vector points away from vessel B  
				v_A.norm() > 0.25											&&	// or does not make a crossing occur. This is not mentioned in Johansen.
				v_B.norm() > 0.25											&&
				!is_head_on 												&&
				!is_passed;

	// Extra condition that the COLREGS violation is only considered in an annulus; i.e. within d_close but outside d_safe.
	// The logic behind having to be outside d_safe is that typically a collision happens here, and thus COLREGS should be disregarded
	// in order to make a safe reactive avoidance maneuver, if possible. 
	// NOTE that this extra condition is only applied for "dumb"
	// obstacle predictions where one assumes neither the own-ship nor the obstacle makes avoidance maneuvers. 
	bool mu = is_close && (( B_is_starboard && is_head_on) || (B_is_starboard && is_crossing && !A_is_overtaken)) && (d_AB > d_safe);

	return mu;
}

#endif