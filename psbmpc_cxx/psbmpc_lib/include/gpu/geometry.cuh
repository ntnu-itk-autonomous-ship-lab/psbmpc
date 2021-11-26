//This files contains a large variety of geometry functions. It includes simple functions that are much used, to very spicific larger functions.

#pragma once

#include <math.h>
#include "psbmpc_defines.hpp"
#include "gpu/utilities_gpu.cuh"
#include "tml/tml.cuh"
#include <cmath>
#include <thrust/device_vector.h>


namespace PSBMPC_LIB
{
namespace GPU
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

inline auto evaluateDistance(const TML::PDVector4f &state1, const TML::PDVector4f &state2){
	return (state1.get_block<2, 1>(0,0,2,1) - state2.get_block<2, 1>(0,0,2,1)).norm();
}

inline auto vx_vy_to_heading_speed_state(const TML::PDVector4f &vx_vy_state){
	const auto speed = std::sqrt(std::pow(vx_vy_state(VX),2)+std::pow(vx_vy_state(VY),2));
	const auto course = std::atan2(vx_vy_state(VY),vx_vy_state(VX));
	TML::PDVector4f heading_speed_state = vx_vy_state;
	heading_speed_state(COG) = course;
	heading_speed_state(SOG) = speed;
	return heading_speed_state;
}

//Line [px, py, COG, SOG] to ax+by=c
inline auto toStandardForm(const const TML::PDVector4f &line)
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
inline auto intersectionpoint(const TML::PDVector4f &line1, const TML::PDVector4f &line2)
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

inline auto relativeBearing(const TML::PDVector4f &obstacle_state, const double x, const double y)
{
	const auto dx = x - obstacle_state(PX);
	const auto dy = y - obstacle_state(PY);
	const auto bearing = std::atan2(dy, dx);
	return wrapPI(bearing - obstacle_state(COG));
}
}
}
