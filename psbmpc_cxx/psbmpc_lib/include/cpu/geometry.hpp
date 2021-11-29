//This files contains a large variety of geometry functions. It includes simple functions that are much used, to very spicific larger functions.

#pragma once

#include <math.h>
#include <Eigen/Dense>
#include "psbmpc_defines.hpp"


namespace PSBMPC_LIB
{

	inline void wrapPI(double *value)
	{
		while (std::abs(*value) > M_PI)
		{
			if (*value < 0)
			{
				*value += 2 * M_PI;
			}
			else
			{
				*value -= 2 * M_PI;
			}
		}
	}


	inline double wrapPI(double value)
	{
		wrapPI(&value);
		return value;
	}


	inline auto evaluateDistance(double dx, double dy)
	{
		return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
	}


	inline auto vx_vy_to_heading_speed_state(const Eigen::Vector4d &vx_vy_state){
		const auto speed = std::sqrt(std::pow(vx_vy_state(VX),2)+std::pow(vx_vy_state(VY),2));
		const auto course = std::atan2(vx_vy_state(VY),vx_vy_state(VX));
		Eigen::Vector4d heading_speed_state = vx_vy_state;
		heading_speed_state(COG) = course;
		heading_speed_state(SOG) = speed;
		return heading_speed_state;
	}



	//Line [px, py, COG, SOG] to ax+by=c
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
			//sin is none-zero, division by sin possible
			res.a = 1;
			res.b = -c / s;
			res.c = res.a * x0 + res.b * y0;
		}
		else
		{
			//cos is none-zero, division by cos posible
			res.a = -s / c;
			res.b = 1;
			res.c = res.a * x0 + res.b * y0;
		}

		return res;
	}


	//Parameterised [px,py,COG,U], U is unused
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


	inline auto relativeBearing(const Eigen::Vector4d &obstacle_state, const double x, const double y)
	{
		const auto dx = x - obstacle_state(PX);
		const auto dy = y - obstacle_state(PY);
		const auto bearing = std::atan2(dy, dx);
		return wrapPI(bearing - obstacle_state(COG));
	}


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


	inline CPA evaluateCPA(const Eigen::Vector4d &ship1, const Eigen::Vector4d &ship2)
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
		wrapPI(&result.bearing_relative_to_heading);

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


	inline auto evaluateCPA(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory)
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


	inline bool evaluate_crossing_aft(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory)
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


	inline bool evaluate_crossing_port_to_port(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory)
	{
		auto CPA = evaluateCPA(ownship_trajectory, obstacle_trajectory);
		auto bearing_to_obstacle_at_CPA = relativeBearing(CPA.closest_point_ownship, CPA.closest_point_obstacle_ship(PX), CPA.closest_point_obstacle_ship(PY));
		return bearing_to_obstacle_at_CPA < 0;
	}


}
