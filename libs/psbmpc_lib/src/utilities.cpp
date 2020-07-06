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

#include "utilities.h"
#include "Eigen/Dense"
#include "iostream"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0f
#endif
#ifndef RAD2DEG
#define RAD2DEG 180.0f / M_PI
#endif

