/****************************************************************************************
*
*  File name : ctrl_cpu.h
*
*  Function  : Header file for the Controller class, responsible for producing
*              generalized forces and moments to steer the own-ship.
*            ---------------------
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

#include "Eigen/Dense"

namespace PSBMPC_LIB
{
    enum Ship_Type
    {
        Telemetron,
        MilliAmpere
    };

    namespace CPU
    {
        class Controller
        {
        private:

            Ship_Type type;

            // Azimuth and propeller vectors (for the front and rear azimuth thrusters on milliAmpere)
            Eigen::Vector2d alpha, omega;

            

            // Controller parameters
			double Kp_u;
			double Kp_psi;
			double Kd_psi;
			double Kp_r;
			
			double r_max; 


        public:

            Controller();

            Eigen::Vector3d operator()(const double u_d, const double psi_d, const Eigen::Matrix<double, 6, 1> &xs);
        };
    }
}