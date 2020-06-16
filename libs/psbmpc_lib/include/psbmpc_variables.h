/****************************************************************************************
*
*  File name : psbmpc_parameters.h
*
*  Function  : Variables in model. Do not duplicate definitions.
*	           ------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim
*
*  Author    : Trym Tengesdal
*
*  Modified  :
*
*****************************************************************************************/

#ifndef _PSBMPC_VARIABLES_H_
#define _PSBMPC_VARIABLES_H_


		

enum Guidance_Strategy 
{
	LOS, 													// Line-of-sight		
	WPP,													// WP-Pursuit
	CH 														// Course Hold (~approx equal to heading hold here as we neglect crab-angle (Can compensate with integral gain in a LOS law e.g))
};

enum Prediction_Method 
{
	Linear,													// Linear prediction
	ERK1 													// Explicit Runge Kutta 1 = Eulers method
};

#endif