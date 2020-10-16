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

#define MAX_N_M 10                      // Max number of avoidance maneuvers allowable

#define MAX_N_SAMPLES 500               // Max number of samples for trajectories and such allowable on the device

#define MAX_N_WPS 10                    // Max number of waypoints for the waypoint matrix allowable on the device

#define MAX_N_OBST 20                   // Max number of obstacles allowable on the device
#define MAX_N_PS 50                     // Max number of prediction scenarios allowable for obstacle trajectory matrices on the device

#define MAX_N_SEG_SAMPLES 4             // Max number of samples in segment considered in the CPE method MCSKF4D
#define MAX_N_CPE_SAMPLES 1000          // Max number of samples allowable to draw in the CPE. 

#endif