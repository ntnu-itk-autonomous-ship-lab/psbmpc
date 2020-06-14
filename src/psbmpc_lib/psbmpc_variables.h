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


// See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" for more information
enum CPE_Method 
{
	2D,														// Consider positional uncertainty only
	4D														// Consider uncertainty in both position and velocity along piece-wise linear segments 
};

enum ST 
{
	A, 														// Non-COLREGS situation	(ST = Ø)
	B, 														// Stand-on in Overtaking 	(ST = OT, SO)
	C, 														// Stand-on in Crossing 	(ST = CR, SO)
	D, 														// Give-way in Overtaking 	(ST = OT, GW)
	E, 														// Give-way in Head-on 		(ST = HO, GW)
	F 														// Give-way in Crossing 	(ST = CR, GW)
};			

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