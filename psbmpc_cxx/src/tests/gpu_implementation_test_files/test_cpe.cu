/****************************************************************************************
*
*  File name : test_cpe.cu
*
*  Function  : Test file for the CUDA-tailored CPE class for PSB-MPC, using Matlab for 
*			   visualization 
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


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include "gpu/cpe_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include "cpu/utilities_cpu.h"
#include "mrou.h"
#include "gpu/ownship_gpu.cuh"
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include "xoshiro.hpp"
#include "Eigen/Dense"
#include "engine.h"

#define BUFSIZE 1000000

// Host only due to stderr usage
#define cuda_check_errors(msg) \
    do { \
        cudaError_t __err = cudaGetLastError(); \
        if (__err != cudaSuccess) { \
            fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
                msg, cudaGetErrorString(__err), \
                __FILE__, __LINE__); \
            fprintf(stderr, "*** FAILED - ABORTING\n"); \
            exit(1); \
        } \
    } while (0)


class CPE_functor
{
private:
	PSBMPC_LIB::GPU::CPE *cpe;
	PSBMPC_LIB::CPE_Method cpe_method;
	float dt;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_p; 
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_i_p;
	TML::PDMatrix<float, 16, MAX_N_SAMPLES> *P_i_p;

	TML::PDMatrix<float, 1, MAX_N_SAMPLES> *P_c_i;

	// Allocate predicted ownship state and predicted obstacle i state and covariance for their prediction scenarios (ps)
	// If cpe_method = MCSKF, then dt_seg must be equal to dt;
	// If cpe_method = CE, then only the first column in these matrices are used (only the current predicted time is considered)
	TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_seg; 
	TML::PDMatrix<float, 4, MAX_N_SEG_SAMPLES> xs_i_seg;
	TML::PDMatrix<float, 16, MAX_N_SEG_SAMPLES> P_i_seg;

public:

	__host__ CPE_functor(
		PSBMPC_LIB::GPU::CPE *cpe, 
		PSBMPC_LIB::CPE_Method cpe_method, 
		TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_p,
		TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_i_p,
		TML::PDMatrix<float, 16, MAX_N_SAMPLES> *P_i_p, 
		TML::PDMatrix<float, 1, MAX_N_SAMPLES> *P_c_i,
		const float dt) : cpe(cpe), cpe_method(cpe_method), xs_p(xs_p), xs_i_p(xs_i_p), P_i_p(P_i_p), P_c_i(P_c_i), dt(dt) {}

	__host__ __device__ ~CPE_functor() { cpe = nullptr; xs_p = nullptr; xs_i_p = nullptr; P_i_p = nullptr; P_c_i = nullptr; }

	__device__ TML::PDMatrix<float, 1, MAX_N_SAMPLES>* operator()(const thrust::tuple<unsigned int> &tuple)
	{
		unsigned int seed = thrust::get<0>(tuple);
		cpe->seed_prng(seed);

		float d_safe_i = 50;

		int n_samples = xs_p->get_cols();
		int n_seg_samples = round(cpe->get_segment_discretization_time() / dt) + 1;

		xs_seg.resize(4, n_seg_samples);
		xs_i_seg.resize(4, n_seg_samples);
		P_i_seg.resize(16, n_seg_samples);

		P_c_i->resize(1, n_samples);

		// For the CE-method:
		TML::Vector2f p_os, p_i, v_os_prev, v_i_prev;
		TML::Matrix2f P_i_2D;

		for (int k = 0; k < n_samples; k++)
		{
			xs_seg.shift_columns_left();
			xs_seg.set_col(n_seg_samples - 1, xs_p->get_col(k));

			P_i_seg.shift_columns_left();
			P_i_seg.set_col(n_seg_samples - 1, P_i_p->get_col(k));

			xs_i_seg.shift_columns_left();
			xs_i_seg.set_col(n_seg_samples - 1, xs_i_p->get_col(k));

			if (k == 0)
			{
				cpe->initialize(xs_seg.get_col(n_seg_samples - 1), xs_i_seg.get_col(n_seg_samples - 1), P_i_seg.get_col(n_seg_samples - 1), d_safe_i);
			}

			/* printf("xs_p = %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", 
				xs_seg(0, n_seg_samples - 1), 
				xs_seg(1, n_seg_samples - 1), 
				xs_seg(2, n_seg_samples - 1), 
				xs_seg(3, n_seg_samples - 1), 
				xs_seg(4, n_seg_samples - 1), 
				xs_seg(5, n_seg_samples - 1));
			printf("xs_i_p = %.1f, %.1f, %.1f, %.1f\n", 
				xs_i_seg(0, n_seg_samples - 1), 
				xs_i_seg(1, n_seg_samples - 1), 
				xs_i_seg(2, n_seg_samples - 1), 
				xs_i_seg(3, n_seg_samples - 1));

			printf("P_i_p = %.1f, %.1f, %.1f, %.1f\n", P_i_seg(0, n_seg_samples - 1), P_i_seg(1, n_seg_samples - 1), P_i_seg(2, n_seg_samples - 1), P_i_seg(3, n_seg_samples - 1));
			printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_seg(4, n_seg_samples - 1), P_i_seg(5, n_seg_samples - 1), P_i_seg(6, n_seg_samples - 1), P_i_seg(7, n_seg_samples - 1));
			printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_seg(8, n_seg_samples - 1), P_i_seg(9, n_seg_samples - 1), P_i_seg(10, n_seg_samples - 1), P_i_seg(11, n_seg_samples - 1));
			printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_seg(12, n_seg_samples - 1), P_i_seg(13, n_seg_samples - 1), P_i_seg(14, n_seg_samples - 1), P_i_seg(15, n_seg_samples - 1)); */

			switch(cpe_method)
			{
				case PSBMPC_LIB::CE :	
					if (k > 0)
					{
						v_os_prev = xs_seg.get_block<2, 1>(3, n_seg_samples - 2, 2, 1);
						v_os_prev = PSBMPC_LIB::GPU::rotate_vector_2D(v_os_prev, xs_seg(2, n_seg_samples - 2));
                    	v_i_prev = xs_i_seg.get_block<2, 1>(2, n_seg_samples - 2, 2, 1);
					}
					p_os = xs_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);
					p_i = xs_i_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);

					P_i_2D = PSBMPC_LIB::GPU::reshape<16, 1, 4, 4>(P_i_seg.get_col(n_seg_samples - 1), 4, 4).get_block<2, 2>(0, 0, 2, 2);

					P_c_i->operator()(k) = cpe->CE_estimate(p_os, p_i, P_i_2D, v_os_prev, v_i_prev, dt);
					printf("k = %d | P_c_i = %.6f\n", k, P_c_i->operator()(k));
					
					break;
				case PSBMPC_LIB::MCSKF4D :                
					if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
					{
						P_c_i->operator()(k) = cpe->MCSKF4D_estimate(xs_seg, xs_i_seg, P_i_seg);						
						printf("k = %d | P_c_i = %.6f\n", k, P_c_i->operator()(k));
					}
					break;
				default :
					// Throw
					break;
			}
		}
		return P_c_i;
	}

	
};

int main(){
	// Matlab engine setup
	Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFSIZE+1];

	std::random_device seed;

	xoshiro256plus64 eng1(seed());

	std::normal_distribution<float> std_norm_pdf(0, 1);
	std::uniform_int_distribution<int> distribution(0, 1000);

	/* // test div cpe functions
	CPE cpe1(MCSKF4D, 0.5);
	TML::PDMatrix<float, 4, MAX_N_CPE_SAMPLES> samples(4, 1000);

	TML::PDVector4f mu(4, 1);
	mu(0) = 68.0f; mu(1) = 75.0f; mu(2) = -2.0f; mu(3) = 0.0f;

	TML::PDMatrix4f sigma(4, 4);
	sigma(0, 0) = 108.6589f; 	sigma(0, 1) = 0.0f; 		sigma(0, 2) = 3.1344f; 		sigma(0, 3) = 0.0f;
	sigma(1, 0) = 0.0f; 		sigma(1, 1) = 108.6589f; 	sigma(1, 2) = 0.0f; 		sigma(1, 3) = 3.1344f;
	sigma(2, 0) = 3.1344f; 		sigma(2, 1) = 0.0f; 		sigma(2, 2) = 1.9365f;	 	sigma(2, 3) = 0.0f;
	sigma(3, 0) = 0.0f; 		sigma(3, 1) = 3.1344f; 		sigma(3, 2) = 0.0f; 		sigma(3, 3) = 1.9365f;

	save_matrix_to_file<float, 4, 4>(sigma);

	cpe1.update_L(sigma);
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 1000; j++)
		{
			samples(i, j) = std_norm_pdf(eng1);
		}
	}
	samples = cpe1.L * samples + mu;

	save_matrix_to_file<float, 4, 1000>(samples);

	TML::Vector2f p_cpa; p_cpa(0) = 24.0f; p_cpa(1) = 0.0f; 
	float t_cpa = 0.5;
	
	cpe1.set_samples(samples);
	cpe1.determine_sample_validity_4D(p_cpa, t_cpa);

	save_matrix_to_file<float, 1, 1000>(cpe1.valid); */
	//*****************************************************************************************************************
	// Own-ship prediction setup
	//*****************************************************************************************************************

	Eigen::Matrix<double, 6, 1> xs_os_0;
	xs_os_0 << 0, 0, 0, 6, 0, 0;

	double T = 50; double dt = 0.5;

	double u_d = 6.0; double chi_d = 0.0;

	Eigen::VectorXd offset_sequence(6);
	Eigen::Vector3d maneuver_times;
	offset_sequence << 1, -90 * M_PI / 180.0, 1, -30 * M_PI / 180.0, 1, 0;
	offset_sequence << 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0, 1, 0 * M_PI / 180.0;
	maneuver_times << 0, 100, 150;

	std::unique_ptr<PSBMPC_LIB::GPU::Ownship> asv(new PSBMPC_LIB::GPU::Ownship()); 

	Eigen::Matrix<double, 6, -1> trajectory; 
	Eigen::Matrix<double, 2, -1> waypoints;

	int n_samples = std::round(T / dt);
	std::cout << "n_samples = " << n_samples << std::endl;

	trajectory.resize(6, n_samples);
	trajectory.block<6, 1>(0, 0) << xs_os_0;

	waypoints.resize(2, 2); 
	//waypoints << 0, 200, 200, 0,    0, 300, 1000,
	//			 0, -50,  -200, -200,  0, 300, 0;
	waypoints << 0, 1000,
				 0, 0;

	//*****************************************************************************************************************
	// Obstacle prediction setup
	//*****************************************************************************************************************

	double sigma_x(0.8), sigma_xy(0),sigma_y(0.8), gamma_x(0.1), gamma_y(0.1);

	Eigen::Vector4d xs_0;
	xs_0 << 75, 75, -2, 0;

	Eigen::Matrix4d P_0;
	P_0 << 100, 0, 0, 0,
			0, 100, 0, 0,
			0, 0, 0.025, 0,
			0, 0, 0, 0.025;

	// Predicted state for each prediction scenario: n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> xs_i_p;

	// Mean predicted velocity for the obstacle (MROU): n_ps x n x n_samples, where n = 4
	std::vector<Eigen::MatrixXd> v_p;

	// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step
	Eigen::MatrixXd P_i_p; 

	std::unique_ptr<PSBMPC_LIB::MROU> mrou(new PSBMPC_LIB::MROU(sigma_x, sigma_xy, sigma_y, gamma_x, gamma_y));

	// n_ps = 1
	xs_i_p.resize(1); xs_i_p[0].resize(4, n_samples);
	xs_i_p[0].col(0) = xs_0;
	P_i_p.resize(16, n_samples);
	P_i_p.col(0) = PSBMPC_LIB::CPU::flatten(P_0);

	v_p.resize(1); v_p[0].resize(2, n_samples);
	
	int n_ps = xs_i_p.size();

	Eigen::Vector2d v_0, v;
	v_0 << -2, 0;
	v_p[0].col(0) = v_0;

	Eigen::VectorXd turn_times(1);
	turn_times << 300;

	int tt_count = 0;
	double chi = 0; 
	v = v_0;
	for(int k = 0; k < n_samples; k++)
	{
		if (k == turn_times(tt_count))
		{
			chi = atan2(v(1), v(0));
			v(0) = v.norm() * cos(chi + 30 * M_PI / 180.0);
			v(1) = v.norm() * sin(chi + 30 * M_PI / 180.0);
			v(0) = -2.0; v(1) = 1.0;
			if (tt_count < turn_times.size() - 1) tt_count += 1;
		}
		if (k < n_samples - 1)	v_p[0].col(k + 1) = v;
	}

	//*****************************************************************************************************************
	// Collision Probability Estimator setup
	//*****************************************************************************************************************
	double dt_seg = 0.5;

	//Eigen::MatrixXf P_c_i_CE(n_ps, n_samples), P_c_i_MCSKF(n_ps, n_samples);
	Eigen::MatrixXd P_c_i_CE(n_ps, n_samples), P_c_i_MCSKF(n_ps, n_samples);
	

	//*****************************************************************************************************************
	// Prediction
	//*****************************************************************************************************************

	asv->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS, T, dt);

	double t = 0;
	int n_seg_samples = std::round(dt_seg / dt) + 1;

	TML::PDMatrix<float, 4, MAX_N_SAMPLES> xs_p_copy(4, n_samples);
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> xs_i_p_copy(4, n_samples);
	TML::PDMatrix<float, 16, MAX_N_SAMPLES> P_i_p_copy(16, n_samples);

	Eigen::Vector2d v_os_p;
	for (int k = 0; k < n_samples; k++)
	{
		v_os_p = trajectory.block<2, 1>(3, k);
		v_os_p = PSBMPC_LIB::CPU::rotate_vector_2D(v_os_p, trajectory(2, k));
		xs_p_copy(0, k) = trajectory(0, k);
		xs_p_copy(1, k) = trajectory(1, k);
		xs_p_copy(2, k) = v_os_p(0);
		xs_p_copy(3, k) = v_os_p(1);
	}
	

	//===========================================================================================
	// Allocate and copy data for use on the device
	//===========================================================================================
	//thrust::device_vector<float> P_c_i_dvec(n_samples);
	thrust::device_vector<TML::PDMatrix<float, 1, MAX_N_SAMPLES>*> P_c_i_dvec(1);

	PSBMPC_LIB::GPU::CPE cpe(PSBMPC_LIB::CE, dt);
	PSBMPC_LIB::GPU::CPE *cpe_device_ptr;
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_p_device_ptr;
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> *xs_i_p_device_ptr;
	TML::PDMatrix<float, 16, MAX_N_SAMPLES> *P_i_p_device_ptr;
	TML::PDMatrix<float, 1, MAX_N_SAMPLES> *P_c_i_device_ptr, P_c_i_host_CE, P_c_i_host_MCSKF, *P_c_i_temp;

	size_t limit = 0;

	cudaDeviceSetLimit(cudaLimitStackSize, 100000);
	cuda_check_errors("Setting cudaLimitStackSize failed.");

	cudaDeviceGetLimit(&limit, cudaLimitStackSize);
	cuda_check_errors("Reading cudaLimitStackSize failed.");
	std::cout << "Set device max stack size : " << limit << std::endl;

	cudaMalloc((void**)&cpe_device_ptr, sizeof(PSBMPC_LIB::GPU::CPE));
    cuda_check_errors("CudaMalloc of CPE failed.");

	std::cout << "sizeof(CPE) = " << sizeof(PSBMPC_LIB::GPU::CPE) << std::endl;
	
	cudaMemcpy(cpe_device_ptr, &cpe, sizeof(PSBMPC_LIB::GPU::CPE), cudaMemcpyHostToDevice);
    cuda_check_errors("CudaMemCpy of CPE failed.");

	cudaMalloc((void**)&xs_p_device_ptr, sizeof(TML::PDMatrix<float, 6, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of xs_p failed.");

	cudaMemcpy(xs_p_device_ptr, &xs_p_copy, sizeof(TML::PDMatrix<float, 6, MAX_N_SAMPLES>), cudaMemcpyHostToDevice);
	cuda_check_errors("CudaMemCpy of xs_p failed.");

	cudaMalloc((void**)&xs_i_p_device_ptr, sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of xs_i_p failed.");

	cudaMalloc((void**)&P_i_p_device_ptr, sizeof(TML::PDMatrix<float, 16, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of P_i_p failed.");

	cudaMalloc((void**)&P_c_i_device_ptr, sizeof(TML::PDMatrix<float, 1, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of P_c_i failed.");


	CPE_functor cpe_functor_CE(cpe_device_ptr, PSBMPC_LIB::CE, xs_p_device_ptr, xs_i_p_device_ptr, P_i_p_device_ptr, P_c_i_device_ptr, (float)dt);

	thrust::device_vector<unsigned int> sample_dvec(1);
	thrust::sequence(sample_dvec.begin(), sample_dvec.end(), distribution(eng1));
	//thrust::sequence(sample_dvec.begin(), sample_dvec.end(), 3);

	auto cpe_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(sample_dvec.begin()));
	auto cpe_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(sample_dvec.end()));

	//===========================================================================================
	for (int ps = 0; ps < n_ps; ps++)
	{
		//=======================================================================================
		// MROU Prediction
		//=======================================================================================
		for (int k = 0; k < n_samples; k++)
		{
			t = (k + 1) * dt;
			if (k < n_samples - 1)
			{
				xs_i_p[ps].col(k + 1) = mrou->predict_state(xs_i_p[ps].col(k), v_p[ps].col(k), dt);
				P_i_p.col(k + 1) = PSBMPC_LIB::CPU::flatten(mrou->predict_covariance(P_0, t));
			}
		}
		TML::assign_eigen_object(xs_i_p_copy, xs_i_p[ps]);
		TML::assign_eigen_object(P_i_p_copy, P_i_p);

		cudaMemcpy(xs_i_p_device_ptr, &xs_i_p_copy, sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>), cudaMemcpyHostToDevice);
		cuda_check_errors("CudaMemCpy of xs_i_p failed.");

		cudaMemcpy(P_i_p_device_ptr, &P_i_p_copy, sizeof(TML::PDMatrix<float, 16, MAX_N_SAMPLES>), cudaMemcpyHostToDevice);
		cuda_check_errors("CudaMemCpy of P_i_p failed.");

		//=======================================================================================
		// Estimate with CE
		//=======================================================================================
		thrust::transform(cpe_tuple_begin, cpe_tuple_end, P_c_i_dvec.begin(), cpe_functor_CE);

		P_c_i_temp = P_c_i_dvec[0];
		cudaMemcpy(&P_c_i_host_CE, P_c_i_temp, sizeof(TML::PDMatrix<float, 1, MAX_N_SAMPLES>), cudaMemcpyDeviceToHost);
		cuda_check_errors("CudaMemCpy of P_c_i_CE failed.");

		for (int k = 0; k < n_samples; k++)
		{
			P_c_i_CE(ps, k) = P_c_i_host_CE(k);
		}
	}

	CPE_functor cpe_functor_MCSKF(cpe_device_ptr, PSBMPC_LIB::MCSKF4D, xs_p_device_ptr, xs_i_p_device_ptr, P_i_p_device_ptr, P_c_i_device_ptr, (float)dt);
	cpe.set_method(PSBMPC_LIB::MCSKF4D);

	cudaMemcpy(cpe_device_ptr, &cpe, sizeof(PSBMPC_LIB::GPU::CPE), cudaMemcpyHostToDevice);
	cuda_check_errors("CudaMemCpy of CPE failed.");

	//=======================================================================================
	// Estimate with MCSKF
	//=======================================================================================
	for (int ps = 0; ps < n_ps; ps++)
	{
		thrust::transform(cpe_tuple_begin, cpe_tuple_end, P_c_i_dvec.begin(), cpe_functor_MCSKF);

		P_c_i_temp = P_c_i_dvec[0];
		cudaMemcpy(&P_c_i_host_MCSKF, P_c_i_temp, sizeof(TML::PDMatrix<float, 1, MAX_N_SAMPLES>), cudaMemcpyDeviceToHost);
		cuda_check_errors("CudaMemCpy of P_c_i_MCSKF failed.");

		for (int k = 0; k < n_samples; k++)
		{
			P_c_i_MCSKF(ps, k) = P_c_i_host_MCSKF(k);
		}
	}

	//=======================================================================================
	// Free device memoery
	//=======================================================================================
	cudaFree(cpe_device_ptr);
	cuda_check_errors("CudaFree of CPE failed.");

	cudaFree(xs_p_device_ptr);
	cuda_check_errors("CudaFree of xs_p failed.");

	cudaFree(xs_i_p_device_ptr);
	cuda_check_errors("CudaFree of xs_i_p failed.");

	cudaFree(P_i_p_device_ptr);
	cuda_check_errors("CudaFree of P_i_p failed.");

	cudaFree(P_c_i_device_ptr);
	cuda_check_errors("CudaFree of P_c_i failed.");

	


	//*****************************************************************************************************************
	// Send data to matlab
	//*****************************************************************************************************************
	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps_mx = mxCreateDoubleMatrix(2, 7, mxREAL);

	double *p_traj_os = mxGetPr(traj_os_mx);
	double *p_wps = mxGetPr(wps_mx);

	mxArray *traj_i_mx = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *v_traj_i_mx = mxCreateDoubleMatrix(2, n_samples, mxREAL);
	mxArray *P_traj_i_mx = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *p_traj_i = mxGetPr(traj_i_mx);
	double *p_v_traj_i = mxGetPr(v_traj_i_mx);
	double *p_P_traj_i = mxGetPr(P_traj_i_mx);

	mwSize dims[1] = {n_samples};
	mxArray *fPcoll_CE = mxCreateNumericArray(1, dims, mxSINGLE_CLASS, mxREAL);
	mxArray *fPcoll_MCSKF = mxCreateNumericArray(1, dims, mxSINGLE_CLASS, mxREAL);
	mxArray *Pcoll_CE = mxCreateDoubleMatrix(1, n_samples, mxREAL);
	mxArray *Pcoll_MCSKF = mxCreateDoubleMatrix(1, n_samples, mxREAL);
	
	float *fp_CE = mxGetSingles(fPcoll_CE);
	float *fp_MCSKF = mxGetSingles(fPcoll_MCSKF);
	double *p_CE = mxGetPr(Pcoll_CE);
	double *p_MCSKF = mxGetPr(Pcoll_MCSKF);

	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, 6, n_samples);
	map_traj_os = trajectory;

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps, 2, 2);
	map_wps = waypoints;

	Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i, 4, n_samples);
	map_traj_i = xs_i_p[0];

	Eigen::Map<Eigen::MatrixXd> map_v_traj_i(p_v_traj_i, 2, n_samples);
	map_v_traj_i = v_p[0];

	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);
	map_P_traj_i = P_i_p;

	/* Eigen::Map<Eigen::MatrixXf> map_Pcoll_CE(fp_CE, 1, n_samples);
	map_Pcoll_CE = P_c_i_CE; */
	Eigen::Map<Eigen::MatrixXd> map_Pcoll_CE(p_CE, 1, n_samples);
	map_Pcoll_CE = P_c_i_CE;

	/* Eigen::Map<Eigen::MatrixXf> map_Pcoll_MCSKF(fp_MCSKF, 1, n_samples);
	map_Pcoll_MCSKF = P_c_i_MCSKF; */
	Eigen::Map<Eigen::MatrixXd> map_Pcoll_MCSKF(p_MCSKF, 1, n_samples);
	map_Pcoll_MCSKF = P_c_i_MCSKF;

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);

	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "WPs", wps_mx);

	engPutVariable(ep, "X_i", traj_i_mx);
	engPutVariable(ep, "v", v_traj_i_mx);
	engPutVariable(ep, "P_flat", P_traj_i_mx);

	engPutVariable(ep, "P_c_CE", Pcoll_CE);
	engPutVariable(ep, "P_c_MCSKF", Pcoll_MCSKF);
	engEvalString(ep, "test_cpe_plot");
	
	//save_matrix_to_file(P_c_i_CE[0]);
	//save_matrix_to_file(P_c_i_MCSKF[0]);

	printf("%s", buffer);
	mxDestroyArray(traj_os_mx);
	mxDestroyArray(wps_mx);

	mxDestroyArray(traj_i_mx);
	mxDestroyArray(v_traj_i_mx);
	mxDestroyArray(P_traj_i_mx);

	mxDestroyArray(fPcoll_CE);
	mxDestroyArray(fPcoll_MCSKF);
	mxDestroyArray(Pcoll_CE);
	mxDestroyArray(Pcoll_MCSKF);
	engClose(ep);

	return 0;
}