/****************************************************************************************
*
*  File name : utilities.h
*
*  Function  : Header file for all-purpose math functions which are used by multiple 
*			   library files. Thus, do NOT add a function here if it belongs to one 
*			   distinct class.
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

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Eigen/Dense"
#include "iostream"

enum Axis 
	{
		Roll,
		Pitch,
		Yaw
	};
namespace Utilities 
{
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

	/****************************************************************************************
	*  Name     : angle_difference_pmpi
	*  Function : Makes sure angle difference is within [-pi, pi] interval
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	inline double angle_difference_pmpi(const double a_1, const double a_2) 
	{
		double diff = wrap_angle_to_02pi(a_2) - wrap_angle_to_02pi(a_1);
		while (diff >= M_PI) diff -= 2 * M_PI;
		while (diff < -M_PI) diff += 2 * M_PI;
		return diff;
	}

	/****************************************************************************************
	*  Name     : rotate_vector_2D
	*  Function : Rotates two-dimensional vectory v by angle, around z-axis here
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	inline Eigen::Vector2d rotate_vector_2D(const Eigen::Vector2d v, const double angle)
	{
		Eigen::Vector2d v_temp;
		v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
		v_temp(2) = v(1) * sin(angle) - v(2) * cos(angle);
		return v_temp;
	}

	/****************************************************************************************
	*  Name     : rotate_vector_3D
	*  Function : Rotates three-dimensional vectory v by angle, around specified axis
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	inline Eigen::Vector3d rotate_vector_3D(const Eigen::Vector3d v, const double angle, const Axis axis)
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

	/****************************************************************************************
	*  Name     : flatten
	*  Function : Flattens a matrix of size n_rows x n_cols to n_rows * n_cols x 1
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	inline Eigen::MatrixXd flatten(const Eigen::MatrixXd& in)
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
				out(count, 1) = in(i, j);
				count += 1;
			}
		}
		return out;
	}

	/****************************************************************************************
	*  Name     : reshape
	*  Function : Reshapes a matrix of size n_rows * n_cols x 1 to n_rows x n_cols 
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	inline Eigen::MatrixXd reshape(const Eigen::MatrixXd& in)
	{
		int n_rows = in.rows();
		int n_cols = in.cols();

		Eigen::MatrixXd out;
		out.resize(n_rows, n_cols);
		int count = 0;
		for(int i = 0; i < n_rows; i++)
		{
			for(int j = 0; j < n_cols; j++)
			{
				out(i, j) = in(count, 1);
				count += 1;
			}
		}
		return out;
	}

/*
	inline void rotate_vector_2D(Eigen::Vector2d &v, const double angle)
	{
		Eigen::Vector2d v_temp;
		v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
		v_temp(2) = v(1) * sin(angle) - v(2) * cos(angle);
		v = v_temp;
	}	
*/

/*
	inline void rotate_vector_3D(Eigen::Vector3d &v, const double angle, const Axis axis)
	{
		Eigen::Vector3d v_temp;
		switch (axis) 
		{
			case Roll : 
			{
				v_temp(0) = v(0);
				v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
				v_temp(2) = v(1) * sin(angle) + v(2) * cos(angle);
			}
			case Pitch : 
			{
				v_temp(0) = v(0) * cos(angle) + v(2) * sin(angle);
				v_temp(1) = v(1);
				v_temp(2) = - v(0) * sin(angle) + v(2) * cos(angle);	
			}			
			case Yaw : 
			{
				v_temp(0) = v(0) * cos(angle) - v(1) * sin(angle);
				v_temp(1) = v(0) * sin(angle) + v(1) * cos(angle);
				v_temp(2) = v(2);
			}
		}
		v = v_temp;
	}
*/
}

#endif