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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Eigen/Dense"

enum Axis 
	{
		Roll,
		Pitch,
		Yaw
	};
namespace Utilities 
{
	inline double wrap_angle_to_pmpi(const double angle) 
	{
		double a = fmod(angle, 2 * M_PI);
		if (a <= -M_PI) a += 2 * M_PI;
		if (a > M_PI) a -= 2 * M_PI;
		return a;
	}

	inline double wrap_angle_to_02pi(const double angle) 
	{
		double a = fmod(angle, 2 * M_PI);
		if (a < 0) a += 2 * M_PI;
		return a;
	}

	inline double angle_difference_pmpi(const double a_1, const double a_2) 
	{
		double diff = wrap_angle_to_02pi(a_2) - wrap_angle_to_02pi(a_1);
		while (diff >= M_PI) diff -= 2 * M_PI;
		while (diff < -M_PI) diff += 2 * M_PI;
		return diff;
	}

/*
	inline Eigen::Vector2d rotate_vector_2D(const Eigen::Vector2d v, const double angle)
	{
		Eigen::Vector2d v_temp;
		v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
		v_temp(2) = v(1) * sin(angle) - v(2) * cos(angle);
		return v_temp;
	}
*/
	inline void rotate_vector_2D(Eigen::Vector2d &v, const double angle)
	{
		Eigen::Vector2d v_temp;
		v_temp(1) = v(1) * cos(angle) - v(2) * sin(angle);
		v_temp(2) = v(1) * sin(angle) - v(2) * cos(angle);
		v = v_temp;
	}	

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
		return v_temp;
	}

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
}