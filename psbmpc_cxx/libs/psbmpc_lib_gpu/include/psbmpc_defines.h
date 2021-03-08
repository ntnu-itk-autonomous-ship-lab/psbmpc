/****************************************************************************************
*
*  File name : psbmpc_defines.h
*
*  Function  : File for Probabilistic Scneario-based Model Predictive Control, defining
*              constants, max thresholds on size parameters for gpu matrices. 
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

#ifndef _PSBMPC_DEFINES_H_
#define _PSBMPC_DEFINES_H_

#define M_PI 3.14159265358979323846

#define DEG2RAD M_PI / 180.0f
#define RAD2DEG 180.0f / M_PI

#define MAX_N_M 5                       // Max number of avoidance maneuvers allowable for the PSBMPC

#define MAX_N_U_OFFSETS_OSBMPC 3        // Max number of offset values considered in surge for the obstacle SBMPC
#define MAX_N_CHI_OFFSETS_OSBMPC 13     // Max number of offset values considered in course for the obstacle SBMPC

#define MAX_N_SAMPLES 400               // Max number of samples for trajectories and such allowable on the device

#define MAX_N_WPS 6                     // Max number of waypoints for the waypoint matrix allowable on the device

#define MAX_N_OBST 5                   // Max number of obstacles allowable on the device
#define MAX_N_PS 10                    // Max number of prediction scenarios allowable for obstacle trajectory matrices on the device

#define MAX_N_SEG_SAMPLES 4             // Max number of samples in segment considered in the CPE method MCSKF4D
#define MAX_N_CPE_SAMPLES 500           // Max number of samples allowable to draw in the CPE. 

#endif