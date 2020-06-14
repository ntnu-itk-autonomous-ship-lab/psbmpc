/*
 *    This file is part of SB-MPC Library.
 *
 *    SB-MPC -- Scenario-Based MPC for Maritime Collision Avoidance.
 *    Copyright (C) 2016-2019 Inger Berge Hagen, Giorgio D. Kwame Minde Kufoalor, 
 *    NTNU Trondheim.
 *    Developed within the Autosea Project (Sensor fusion and collision avoidance for 
 *    autonomous surface vehicles) under the supervision of Tor Arne Johansen. 
 *    All rights reserved.
 *
 *    SB-MPC Library is software used according to the conditions of the Autosea Consortium.
 *    <https://www.ntnu.edu/autosea>
 */

 
/**
 *    \file   main.cpp
 *    \brief  An example of how to use the SB-MPC Library.
 *    \author Inger Berge Hagen, Giorgio D. K. M. Kufoalor
 */ 


#include "obstacle.h"
#include "sb_mpc.h"
#include "ship_model.h"

#include "Eigen/Dense"
#include "iostream"

int main(){

	// Guidance parameters
	double u_d = 5;
    double psi_d = 0;//1.5708;

	// Offsets
	double u_os;
	double psi_os;

	simulationBasedMpc *sb_mpc = new simulationBasedMpc();

	Eigen::Matrix<double, 6, 1> asv_state;
	asv_state << 0, 0, 0, 5, 0, 0;   // x,y,psi,u,v,r

//    Eigen::Matrix<double, 5, 10> obst_states;        // x,y,u,v,psi,A,B,C,D, id (A,B,C,D are from AIS message)
//    obst_states << 110.359,  146.154,  4.71239,        3,        0,       10,       10,       10,       10, 1,
//              -826.865,  714.124,        0,        0,        0,       10,       10,       10,       10, 2,
//               3444.27, -1950.35,  5.72293,  13.2727,        0,       10,       10,       10,       10, 3,
//              -614.694,  735.825,  1.24093,  3.29244,        0,       10,       10,       10,       10, 4,
//              -522.324,  6325.05,  2.27242,        0,        0,       10,       10,       10,       10, 5;
    
    Eigen::Matrix<double, 1, 10> obst_states;
    obst_states << 10000, 10000, 10, 0, 3., 10, 10, 10, 10, 1; //out of range obstacle - clears obst
//    obst_states << -50, -300, 10, 0, 3.14, 10, 10, 10, 10, 1; // head-on ?
    
    Eigen::Matrix<double, 1,4> static_obst;
    static_obst << 50, -10, 20, 100;                // x_0, y_0, x_1, y_1
    
    Eigen::Matrix<double,1,2> next_waypoints;
    next_waypoints << 1000, 0;

	sb_mpc->getBestControlOffset(u_os, psi_os, u_d, psi_d, asv_state, obst_states, static_obst, next_waypoints);

	std::cout << "u_ os : " << u_os << std::endl;
	std::cout << "psi_os : " << psi_os*180.0f/M_PI << std::endl;

//    std::cout << "D_Init : " << sb_mpc->getDInit() << std::endl;
//    std::cout << "KChiSB : " << sb_mpc->getKChiSB() << std::endl;
//    std::cout << "KChiP : " << sb_mpc->getKChiP() << std::endl;
	
	sb_mpc->setDInit(400.0);
	sb_mpc->setKChiSB(400.0);
	sb_mpc->setKChiP(400.0);

//    std::cout << "D_Init : " << sb_mpc->getDInit() << std::endl;
//    std::cout << "KChiSB : " << sb_mpc->getKChiSB() << std::endl;
//    std::cout << "KChiP : " << sb_mpc->getKChiP() << std::endl;
//
//    sb_mpc->setDInit(300.0);
//    std::cout << "D_Init : " << sb_mpc->getDInit() << std::endl;
//
//    sb_mpc->setKChiSB(1.5);
//    std::cout << "KChiSB : " << sb_mpc->getKChiSB() << std::endl;
//
//    sb_mpc->setKChiP(100.5);
//    std::cout << "KChiP : " << sb_mpc->getKChiP() << std::endl;
//    std::cout << "G : " << sb_mpc->getG() << std::endl;
//    std::cout << "T_stat : " << sb_mpc->getT_stat() << std::endl;


	return 0;
};
