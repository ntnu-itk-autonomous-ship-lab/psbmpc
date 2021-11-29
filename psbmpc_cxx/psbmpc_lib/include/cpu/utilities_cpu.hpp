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
		*  Name     : read_matrix_from_file
		*  Function : Must be a resizeable eigen matrix type. The matrix size must be given
		*			  in the first row
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		template <class Eigen_Type>
		inline Eigen_Type read_matrix_from_file(const std::string &filename)
		{
			Eigen_Type out;
			std::ifstream indata(filename);

			if(!indata) 
			{
				std::cerr << "Error: file could not be opened" << std::endl;
				exit(1);
			}
			std::string line, data;
			
			int n_rows(0), n_cols(0);
			int row_count(0), col_count(0);
			while (std::getline(indata, line))
			{
				std::stringstream ss(line);
				if (row_count == 0)
				{
					getline( ss, data, ',' );
					n_rows = std::stoi(data);
					getline( ss, data, ',' );
					n_cols = std::stoi(data);
					out.resize(n_rows, n_cols);
				}
				else
				{
					col_count = 0;
					while (std::getline(ss, data, ','))
					{
						out(row_count - 1, col_count) = std::stod(data);
						col_count++;
					}
				}
				row_count++;
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

		/****************************************************************************************
		*  Name     : ship_is_passed_by
		*  Function : Checks if an obstacle vessel B is passed by the ownship
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		inline bool ship_is_passed_by(
				const Eigen::VectorXd &xs_0,
				const Eigen::Vector4d &xs_i,
				const double d_safe
			)
		{
			Eigen::Vector2d v_0, v_i, L_0i;
			double psi_0 = xs_0(2), d_0i;
			if (xs_0.size() == 4)	
			{
				v_0(0) = xs_0(3) * cos(xs_0(2));
				v_0(1) = xs_0(3) * sin(xs_0(2));
			}
			else
			{
				v_0(0) = xs_0(3);
				v_0(1) = xs_0(4);
				v_0 = rotate_vector_2D(v_0, psi_0);
			}
			v_i(0) = xs_i(2); v_i(1) = xs_i(3);
			L_0i = xs_i.block<2, 1>(0, 0) - xs_0.block<2, 1>(0, 0);
			d_0i = L_0i.norm();
			L_0i.normalize();
			
			bool A_is_overtaken = v_0.dot(v_i) > cos(68.5 * DEG2RAD) * v_0.norm() * v_i.norm() 	&&
							v_0.norm() < v_i.norm()							  					&&
							v_0.norm() > 0.25;

			bool B_is_overtaken = v_i.dot(v_0) > cos(68.5 * DEG2RAD) * v_i.norm() * v_0.norm() 	&&
							v_i.norm() < v_0.norm()							  					&&
							v_i.norm() > 0.25;

			bool is_passed = ((v_0.dot(L_0i) < cos(112.5 * DEG2RAD) * v_0.norm()				&& // Vessel A's perspective
						!A_is_overtaken) 														||
						(v_i.dot(-L_0i) < cos(112.5 * DEG2RAD) * v_i.norm() 					&& // Vessel B's perspective
						!B_is_overtaken)) 														&&
						d_0i > d_safe;
			return is_passed;
		}

		/****************************************************************************************
		*  Name     : evaluateDistance
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto evaluateDistance(double dx, double dy)
		{
			return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
		}

		/****************************************************************************************
		*  Name     : relativeBearing
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto relativeBearing(const Eigen::Vector4d &obstacle_state, const double x, const double y)
		{
			const auto dx = x - obstacle_state(PX);
			const auto dy = y - obstacle_state(PY);
			const auto bearing = std::atan2(dy, dx);
			return wrap_angle_to_pmpi(bearing - obstacle_state(COG));
		}

		/****************************************************************************************
		*  Name     : toStandardForm
		*  Function : Line [px, py, COG, SOG] to ax+by=c
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto toStandardForm(const Eigen::Vector4d &line)
		{
			struct
			{
				double a;
				double b;
				double c;
			} res;

			double s = sin(line(COG));
			double c = cos(line(COG));
			double x0 = line(PX);
			double y0 = line(PY);

			if (std::abs(s) > std::abs(c))
			{
				//sin is non-zero, division by sin possible
				res.a = 1;
				res.b = -c / s;
				res.c = res.a * x0 + res.b * y0;
			}
			else
			{
				//cos is non-zero, division by cos possible
				res.a = -s / c;
				res.b = 1;
				res.c = res.a * x0 + res.b * y0;
			}

			return res;
		}
		
		/****************************************************************************************
		*  Name     : intersectionpoint
		*  Function : Parameterised [px,py,COG,U], U is unused
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto intersectionpoint(const Eigen::Vector4d &line1, const Eigen::Vector4d &line2)
		{
			struct
			{
				double x;
				double y;
			} res;

			auto sline1 = toStandardForm(line1);
			auto sline2 = toStandardForm(line2);

			//according to https://math.stackexchange.com/questions/1992153/given-two-lines-each-defined-using-hesse-normal-form-find-the-intersection-poin
			res.x = (sline1.c * sline2.b - sline1.b * sline2.c) / (sline1.a * sline2.b - sline1.b * sline2.a);
			res.y = (sline1.a * sline2.c - sline1.c * sline2.a) / (sline1.a * sline2.b - sline1.b * sline2.a);

			return res;
		}

		/****************************************************************************************
		*  Name     : evaluate_arrival_time
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline double evaluate_arrival_time(const Eigen::Vector4d &line, const double x, const double y)
		{
			if (line(SOG) < 1e-6)
			{
				if (std::abs(relativeBearing(line, x, y)) < 90 * DEG2RAD)
				{
					return INFINITY;
				}
				else
				{
					return -INFINITY;
				}
			}

			//0/inf can happen if res.x-line(PX) ~=0. Avoid this by using the biggest of x and y to evaluate
			double dx = x - line(PX);
			double dy = y - line(PY);

			if (dx > dy)
			{
				return dx / (line(SOG) * cos(line(COG)));
			}
			else
			{
				return dy / (line(SOG) * sin(line(COG)));
			}
		}

		struct CPA
		{
			double bearing_relative_to_heading;
			double distance_at_CPA = INFINITY;
			double time_untill_CPA = INFINITY;
			bool passing_in_front;
			std::string has_passed;
		};

		/****************************************************************************************
		*  Name     : evaluateCPA
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline CPA evaluateCPA(
			const Eigen::Vector4d &ship1, 
			const Eigen::Vector4d &ship2
			)
		{
			//See the following for derivation: https://math.stackexchange.com/questions/1775476/shortest-distance-between-two-objects-moving-along-two-lines
			CPA result;
			const double dPx = ship2(PX) - ship1(PX);											  //Difference in pos x
			const double dPy = ship2(PY) - ship1(PY);											  //Difference in pos y
			const double dVx = ship2(SOG) * std::cos(ship2(COG)) - ship1(SOG) * std::cos(ship1(COG)); //Difference in velocity x
			const double dVy = ship2(SOG) * std::sin(ship2(COG)) - ship1(SOG) * std::sin(ship1(COG)); //Difference in velocity x

			const double A = std::pow(dVx, 2) + std::pow(dVy, 2);
			const double B = dPx * dVx + dPy * dVy;

			double t;
			if (A == 0)
			{
				//ships are relativly stationary
				t = 0;
				result.time_untill_CPA = INFINITY;
			}
			else
			{
				t = -B / A;
				result.time_untill_CPA = t;
			}

			const double dx_at_CPA = dPx + dVx * t;
			const double dy_at_CPA = dPy + dVy * t;

			result.distance_at_CPA = evaluateDistance(dx_at_CPA, dy_at_CPA);

			const double bearing = std::atan2(dy_at_CPA, dx_at_CPA);
			result.bearing_relative_to_heading = bearing - ship1(COG);
			wrap_angle_to_pmpi(result.bearing_relative_to_heading);

			//Identify which passes in front
			//Find intersectpoint of the two paths (if there is one)
			auto intersection = intersectionpoint(ship1, ship2);
			//Find when the two are at the intersect-point
			//arrival time along the trajectory
			//0/inf can happen if dx ~=0. Avoid this by using the biggest of x and y to evaluate
			double time_at_intersectionpoint_ship1 = evaluate_arrival_time(ship1, intersection.x, intersection.y);
			double time_at_intersectionpoint_ship2 = evaluate_arrival_time(ship2, intersection.x, intersection.y);

			//Whichever is there first passes in front of the other
			result.passing_in_front = time_at_intersectionpoint_ship1 < time_at_intersectionpoint_ship2;

			return result;
		}

		/****************************************************************************************
		*  Name     : evaluateCPA
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto evaluateCPA(
			const Eigen::MatrixXd &ownship_trajectory, 
			const Eigen::MatrixXd &obstacle_trajectory
			)
		{
			struct
			{
				Eigen::Vector4d closest_point_ownship;
				Eigen::Vector4d closest_point_obstacle_ship;
				double closest_distance = INFINITY;
			} result;

			Eigen::Vector2d distance_vec;
			for (int i = 0; i < std::min(ownship_trajectory.cols(), obstacle_trajectory.cols()); ++i)
			{
				distance_vec = ownship_trajectory.block<2, 1>(0, i) - obstacle_trajectory.block<2, 1>(0, i);
				double distance = distance_vec.norm();
				if (distance < result.closest_distance)
				{
					result.closest_point_ownship = ownship_trajectory.col(i);
					result.closest_point_obstacle_ship = obstacle_trajectory.col(i);
					result.closest_distance = distance;
				}
				else
				{
					break;
				}
			}
			return result;
		}

		/****************************************************************************************
		*  Name     : evaluate_crossing_aft
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline bool evaluate_crossing_aft(
			const Eigen::MatrixXd &ownship_trajectory, 
			const Eigen::MatrixXd &obstacle_trajectory
			)
		{
			int crossing_point_index_ownship;
			int crossing_point_index_obst_ship;
			double min_distance = INFINITY;
			Eigen::Vector2d distance_vec;
			for (int i = 0; i < ownship_trajectory.cols(); ++i)
			{
				int this_round_obstacle_index;
				double this_round_best = INFINITY;

				for (int j = 0; j < obstacle_trajectory.cols(); ++j)
				{
					distance_vec = ownship_trajectory.block<2, 1>(0, i) - obstacle_trajectory.block<2, 1>(0, j);
					double distance = distance_vec.norm();
					if (distance < this_round_best)
					{
						this_round_best = distance;
						this_round_obstacle_index = j;
					}
					else
					{
						break;
					}
				}
				if (this_round_best < min_distance)
				{
					min_distance = this_round_best;
					crossing_point_index_ownship = i;
					crossing_point_index_obst_ship = this_round_obstacle_index;
				}
				else
				{
					break;
				}
			}
			//If ownship reaches the closest point last then it crosses aft
			return crossing_point_index_ownship > crossing_point_index_obst_ship;
		}

		/****************************************************************************************
		*  Name     : evaluate_crossing_port_to_port
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline bool evaluate_crossing_port_to_port(
			const Eigen::MatrixXd &ownship_trajectory, 
			const Eigen::MatrixXd &obstacle_trajectory
			)
		{
			auto CPA = evaluateCPA(ownship_trajectory, obstacle_trajectory);
			auto bearing_to_obstacle_at_CPA = relativeBearing(CPA.closest_point_ownship, CPA.closest_point_obstacle_ship(PX), CPA.closest_point_obstacle_ship(PY));
			return bearing_to_obstacle_at_CPA < 0;
		}

		/****************************************************************************************
		*  Name     : vx_vy_to_heading_speed_state
		*  Function : 
		*  Author   : Sverre Velten Rothmund
		*  Modified :
		*****************************************************************************************/
		inline auto vx_vy_to_heading_speed_state(
			const Eigen::Vector4d &vx_vy_state
			)
		{
			const auto speed = std::sqrt(std::pow(vx_vy_state(VX),2)+std::pow(vx_vy_state(VY),2));
			const auto course = std::atan2(vx_vy_state(VY),vx_vy_state(VX));
			Eigen::Vector4d heading_speed_state = vx_vy_state;
			heading_speed_state(COG) = course;
			heading_speed_state(SOG) = speed;
			return heading_speed_state;
		}
	}
}
