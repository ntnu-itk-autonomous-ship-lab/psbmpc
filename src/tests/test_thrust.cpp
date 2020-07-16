/****************************************************************************************
*
*  File name : test_thrust.cpp
*
*  Function  : Test file for the thrust library functionality
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

#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <functional>

#include "xoshiro.hpp"

void do_work()
{

}

int main(void) 
{
  std::random_device seed;

  xoshiro256plus64 eng1(seed());

  thrust::host_vector<Eigen::VectorXd> samples_host;
	thrust::device_vector<Eigen::VectorXd> samples_gpu;

  thrust::transform(samples_gpu.begin(), samples_gpu.end(), samples_gpu, [samples_gpu](){ samples_gpu[i].resize(4); });
  return 0;
}


