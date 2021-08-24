/****************************************************************************************
*
*  File name : utilities_cpu.hpp
*
*  Function  : Header file for all-purpose math functions which can/are used by multiple 
*			   host library files. Thus, do NOT add a function here if it belongs to one 
*			   distinct class.
*  
*	           ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2021 Trym Tengesdal, NTNU Trondheim. 
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  : 
*
*****************************************************************************************/

#pragma once

#include "psbmpc_defines.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace PSBMPC_LIB
{
	namespace CPU
	{
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
		*  Name     : parse_VVD
		*  Function : Parse string into nested vector of doubles. Modified version from 
		*			  <https://github.com/ros-planning/navigation2/blob/30b405c58e6d53ba8c96381416bc4679d35a1483/nav2_costmap_2d/src/array_parser.cpp>
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		inline std::vector<std::vector<double>> parse_VVD(const std::string &input)
		{
			std::vector<std::vector<double>> result;

			std::stringstream input_ss(input);
			int depth = 0;
			std::vector<double> current_vector;
			while (!!input_ss && !input_ss.eof()) {
				switch (input_ss.peek()) {
				case EOF:
					break;
				case '[':
					depth++;
					if (depth > 2) 
					{
						throw std::logic_error("Array depth greater than 2");
						return result;
					}
					input_ss.get();
					current_vector.clear();
					break;
				case ']':
					depth--;
					if (depth < 0) 
					{
						throw std::logic_error("More close ] than open [");
						return result;
					}
					input_ss.get();
					if (depth == 1) 
					{
						result.push_back(current_vector);
					}
					break;
				case ',':
				case ' ':
				case '\t':
					input_ss.get();
					break;
				default:  // All other characters should be part of the numbers.
					if (depth != 2) 
					{
						std::stringstream err_ss;
						err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
						throw std::logic_error(err_ss.str());
						return result;
					}
					float value;
					input_ss >> value;
					if (!!input_ss) 
					{
						current_vector.push_back(value);
					}
					break;
				}
			}

			if (depth != 0) 
			{
				throw std::logic_error("Unterminated vector string.");
			} else 
			{
				throw std::logic_error("");
			}

			return result;
		}

		/****************************************************************************************
		*  Name     : save_matrix_to_file
		*  Function : Two overloads
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		template <class Eigen_Type>
		inline void save_matrix_to_file(const Eigen_Type &in)
		{
			std::ofstream outdata("/home/admin/Desktop/thecolavrepo/psbmpc_cxx/src/matlab_scripts/matrix.csv", std::ios::out | std::ios::trunc);
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

		/****************************************************************************************
		*  Name     : rotate_matrix_2D
		*  Function : Rotates the 2D matrix by the angle. 
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		inline Eigen::Matrix2d rotate_matrix_2D(const Eigen::Matrix2d &M, const double angle)
		{
			Eigen::Matrix2d R;
			R << cos(angle), -sin(angle), sin(angle), cos(angle);
			
			return R * M * R.transpose();
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

			// Either xs = [x, y, psi, u, v, r]^T or [x, y, Vx, Vy]
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
	}
}
