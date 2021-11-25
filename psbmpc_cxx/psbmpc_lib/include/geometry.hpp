//This files contains a large variety of geometry functions. It includes simple functions that are much used, to very spicific larger functions.

#pragma once

#include <math.h>
#include <Eigen/Dense>
#include "psbmpc_defines.hpp"

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

inline double surgeSwayToCourse(double heading, double surge, double sway)
{
	double crab_angle(0.0);
	if (std::abs(surge) < 1e-02 && std::abs(sway) < 1e-02)
	{
		crab_angle = 0.0;
	}
	else
	{
		crab_angle = atan2(sway, surge);
	}
	return heading + crab_angle;
}

inline double surgeSwayToSpeed(double surge, double sway)
{
	return std::sqrt(std::pow(surge, 2) + std::pow(sway, 2));
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

//Line parameterised as [px,py,COG,U], point as [px,py]
inline auto closest_point_on_line(const Eigen::Vector4d &line, const Eigen::Vector2d &point)
{
	struct
	{
		double x;
		double y;
		double distance;
		double arrival_time;
	} res;

	auto sline = toStandardForm(line);

	//Closest point is according to: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double x0 = point(PX);
	double y0 = point(PY);
	res.x = (sline.b * (sline.b * x0 - sline.a * y0) + sline.a * sline.c) / (std::pow(sline.a, 2) + std::pow(sline.b, 2));
	res.y = (sline.a * (sline.a * y0 - sline.b * x0) + sline.b * sline.c) / (std::pow(sline.a, 2) + std::pow(sline.b, 2));
	res.distance = std::abs(sline.a * x0 + sline.b * y0 - sline.c) / (std::sqrt(std::pow(sline.a, 2) + std::pow(sline.b, 2)));

	//arrival time along the trajectory
	res.arrival_time = evaluate_arrival_time(line, res.x, res.y);
	return res;
}

inline auto closest_point_in_trajectory_to_line(const Eigen::MatrixXd &trajecotry, const Eigen::Vector4d &obstacle_state, double dt)
{
	size_t number_of_timesteps = trajecotry.cols();
	struct
	{
		double ownship_arrival_time;
		double othership_arrival_time;
		double x;
		double y;
	} res;
	double minimal_distance = INFINITY;
	bool found_closest_point = false;
	for (size_t i = 0; i < number_of_timesteps; ++i)
	{
		auto closest_point = closest_point_on_line(obstacle_state, trajecotry.block<2, 1>(0, i));
		if (closest_point.distance < minimal_distance)
		{
			minimal_distance = closest_point.distance;
			res.ownship_arrival_time = i * dt;
			res.othership_arrival_time = closest_point.arrival_time;
			res.x = trajecotry(PX, i);
			res.y = trajecotry(PY, i);
		}
		else
		{
			found_closest_point = true;
			break;
		}
	}
	if (!found_closest_point)
	{
		//did not cross in trajectory, extrapolate straight path after trajectory
		auto intersection_point = intersectionpoint(trajecotry.col(number_of_timesteps - 1), obstacle_state);
		res.ownship_arrival_time = number_of_timesteps * dt + evaluate_arrival_time(trajecotry.col(number_of_timesteps - 1), intersection_point.x, intersection_point.y);
		res.othership_arrival_time = evaluate_arrival_time(obstacle_state, intersection_point.x, intersection_point.y);
		res.x = intersection_point.x;
		res.y = intersection_point.y;
	}
	return res;
}

//CPA - closest point of approach
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

//Evaluate closest point of approach (CPA) distance, and time untill CPA
inline CPA evaluateCPA(const Eigen::MatrixXd &trajecotry, const Eigen::Vector4d &obstacle_state, double dt)
{
	CPA result;

	const Eigen::Vector2d obstacle_initial_position = obstacle_state.block<2, 1>(0, 0);
	Eigen::Vector2d obstacle_velocity;
	obstacle_velocity(0) = std::cos(obstacle_state(COG)) * obstacle_state(SOG);
	obstacle_velocity(1) = std::sin(obstacle_state(COG)) * obstacle_state(SOG);

	bool found_closest_point = false;
	size_t number_of_timesteps = trajecotry.cols();
	for (size_t i = 0; i < number_of_timesteps; ++i)
	{
		double current_time = i * dt;
		const Eigen::Vector2d obstacle_curent_position = obstacle_initial_position + obstacle_velocity * current_time;
		const Eigen::Vector2d current_position = trajecotry.block<2, 1>(0, i);
		auto vector_to_obst = obstacle_curent_position - current_position;
		double current_distane_to_obstacle = vector_to_obst.norm();
		if (current_distane_to_obstacle < result.distance_at_CPA)
		{
			result.distance_at_CPA = current_distane_to_obstacle;
			result.time_untill_CPA = current_time;

			const double angle_to_other_ship = std::atan2(vector_to_obst(1), vector_to_obst(0));
			result.bearing_relative_to_heading = angle_to_other_ship - trajecotry(COG, i);
			wrapPI(&result.bearing_relative_to_heading);
		}
		else
		{
			found_closest_point = true;
			break;
		}
	}

	//IF CPA was the last time-step, then evaluate forward in time assuming constant heading
	if (!found_closest_point)
	{
		result = evaluateCPA(trajecotry.col(number_of_timesteps - 1), obstacle_state);
	}
	else
	{
		auto closest_point = closest_point_in_trajectory_to_line(trajecotry, obstacle_state, dt);
		result.passing_in_front = closest_point.ownship_arrival_time < closest_point.othership_arrival_time;
	}

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

inline double crossingInFrontDistance(const Eigen::Vector4d &ship1, const Eigen::Vector4d &ship2)
{
	auto intersection_point = intersectionpoint(ship1, ship2);
	if (!std::isfinite(intersection_point.x) || !std::isfinite(intersection_point.y))
	{
		return INFINITY;
	}

	//find when ship1 is at the intersection point
	const auto t1 = evaluate_arrival_time(ship1, intersection_point.x, intersection_point.y);
	const auto t2 = evaluate_arrival_time(ship2, intersection_point.x, intersection_point.y);
	if (t1 < 0 || t2 < 0)
	{
		return INFINITY;
	}
	//Check if ship 1 is in front
	//if it arrives last, then it passes behind
	if (t1 > t2)
		return INFINITY;

	//Find where ship 2 is when ship1 crosses
	const auto Px2 = ship2[PX] + ship2[SOG] * cos(ship2[COG]) * t1;
	const auto Py2 = ship2[PY] + ship2[SOG] * sin(ship2[COG]) * t1;
	//Find distance
	return evaluateDistance(Px2 - intersection_point.x, Py2 - intersection_point.y);
}

inline double crossingInFrontDistanceTrajectory(const Eigen::MatrixXd &trajecotry, const Eigen::Vector4d &ship2, double dt)
{
	auto closest_point = closest_point_in_trajectory_to_line(trajecotry, ship2, dt);
	if (!std::isfinite(closest_point.x) || !std::isfinite(closest_point.y) || !std::isfinite(closest_point.othership_arrival_time) || !std::isfinite(closest_point.ownship_arrival_time))
		return INFINITY;
	if (closest_point.ownship_arrival_time > closest_point.othership_arrival_time)
		return INFINITY;

	//Find where ship 2 is when ship1 is at intersection point
	const auto Px2 = ship2[PX] + ship2[SOG] * cos(ship2[COG]) * closest_point.ownship_arrival_time;
	const auto Py2 = ship2[PY] + ship2[SOG] * sin(ship2[COG]) * closest_point.ownship_arrival_time;
	//Find distance
	return evaluateDistance(Px2 - closest_point.x, Py2 - closest_point.y);
}

inline double distanceToMidpointCourse(const Eigen::Vector4d &ship1, const Eigen::Vector4d &ship2)
{
	//Find midpoint
	const double midpoint_x = (ship1(PX) + ship2(PX)) / 2;
	const double midpoint_y = (ship1(PY) + ship2(PY)) / 2;
	Eigen::Vector4d midpoint_state;
	midpoint_state << midpoint_x, midpoint_y, 0, 0;
	//Find CPA-vector relative to midpoint
	CPA cpa_to_midpoint = evaluateCPA(ship1, midpoint_state);

	//Must pass with the midpoint on the port side
	if (cpa_to_midpoint.bearing_relative_to_heading < 0)
	{
		return cpa_to_midpoint.distance_at_CPA;
	}
	else
	{
		return 0;
	}
}

inline double distanceToMidpointTrajectory(const Eigen::MatrixXd &trajecotry, const Eigen::Vector4d &ship2, double dt)
{
	//Find midpoint
	const double midpoint_x = (trajecotry(PX, 0) + ship2(PX)) / 2;
	const double midpoint_y = (trajecotry(PY, 0) + ship2(PY)) / 2;
	Eigen::Vector4d midpoint_state;
	midpoint_state << midpoint_x, midpoint_y, 0, 0;
	//Find CPA-vector relative to midpoint
	CPA cpa_to_midpoint = evaluateCPA(trajecotry, midpoint_state, dt);

	//Must pass with the midpoint on the port side
	if (cpa_to_midpoint.bearing_relative_to_heading < 0)
	{
		return cpa_to_midpoint.distance_at_CPA;
	}
	else
	{
		return 0;
	}
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
