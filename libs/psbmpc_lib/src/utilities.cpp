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
#include "fstream"
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0f
#endif
#ifndef RAD2DEG
#define RAD2DEG 180.0f / M_PI
#endif

void save_matrix_to_file(const Eigen::MatrixXd &in)
{
    std::ofstream outdata("/home/trymte/Desktop/cybercolav_cxx/src/matlab_scripts/matrix.csv", std::ofstream::trunc);
    int n_rows = in.rows();
    int n_cols = in.cols();

    if(!outdata) 
    {
        std::cerr << "Error: file could not be opened" << std::endl;
        exit(1);
    }
    for (int i = 0; i < n_rows; i++)
    {
        for (int j = 0; j < n_cols; j++)
        {
            outdata << in(i, j);
            if (j != n_cols - 1) { outdata << ","; }
        }
        if (i != n_rows - 1) { outdata << std::endl; }
    }
}

