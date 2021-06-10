/****************************************************************************************
*
*  File name : psbmpc.cu
*
*  Function  : Class functions for Probabilistic Scenario-based Model Predictive Control
*			   where the GPU is used (hence .cu)
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

#include "gpu/psbmpc_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include "cpu/utilities_cpu.hpp"
#include "gpu/cuda_obstacle.cuh"
#include "gpu/cpe_gpu.cuh"
#include "gpu/obstacle_sbmpc_gpu.cuh"
#include "gpu/mpc_cost_gpu.cuh"
#include "gpu/cb_cost_functor.cuh"
#include "gpu/prediction_obstacle_gpu.cuh"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>

#include <iostream>
#include <iomanip>
#include <chrono>
#include "engine.h"

#define BUFFSIZE 100000

namespace PSBMPC_LIB
{
namespace GPU
{

/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC() 
	: 
	u_opt_last(1.0), chi_opt_last(0.0), min_cost(1e12), ownship(Ownship()), pars(PSBMPC_Parameters()), trajectory_device_ptr(nullptr), 
	pars_device_ptr(nullptr), fdata_device_ptr(nullptr), obstacles_device_ptr(nullptr), cpe_device_ptr(nullptr), ownship_device_ptr(nullptr), 
	polygons_device_ptr(nullptr), mpc_cost_device_ptr(nullptr)
	
{
	opt_offset_sequence.resize(2 * pars.n_M);
	maneuver_times.resize(pars.n_M);

	cpe_host = CPU::CPE(pars.cpe_method, pars.dt);

	mpc_cost = CPU::MPC_Cost<PSBMPC_Parameters>(pars);

	preallocate_device_data();
}

/****************************************************************************************
*  Name     : ~PSBMPC
*  Function : Class destructor
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::~PSBMPC() 
{
	cudaFree(trajectory_device_ptr);
	cuda_check_errors("CudaFree of trajectory failed.");

	cudaFree(pars_device_ptr);
	cuda_check_errors("CudaFree of CB_Functor_Pars failed.");

	cudaFree(fdata_device_ptr); 
	cuda_check_errors("CudaFree of CB_Functor_Data failed.");

	cudaFree(obstacles_device_ptr);
	cuda_check_errors("CudaFree of Cuda_Obstacles failed.");

	cudaFree(cpe_device_ptr);
	cuda_check_errors("CudaFree of CPE failed.");

	cudaFree(ownship_device_ptr);
	cuda_check_errors("CudaFree of Ownship failed.");	

	cudaFree(polygons_device_ptr);
	cuda_check_errors("CudaFree of Basic_Polygon`s failed.");

	cudaFree(mpc_cost_device_ptr);
	cuda_check_errors("CudaFree of MPC_Cost failed.");
};

/****************************************************************************************
*  Name     : calculate_optimal_offsets
*  Function : Calculate optimal surge and course offsets for PSB-MPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_optimal_offsets(									
	double &u_opt, 															// In/out: Optimal surge offset
	double &chi_opt, 														// In/out: Optimal course offset
	Eigen::Matrix<double, 2, -1> &predicted_trajectory,						// In/out: Predicted optimal ownship trajectory
	const double u_d, 														// In: Surge reference
	const double chi_d, 													// In: Course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,							// In: Next waypoints
	const Eigen::VectorXd &ownship_state, 									// In: Current ship state
	const double V_w,														// In: Estimated wind speed
	const Eigen::Vector2d &wind_direction,									// In: Unit vector in NE describing the estimated wind direction
	const std::vector<polygon_2D> &polygons,								// In: Static obstacle information
	Obstacle_Data<Tracked_Obstacle> &data									// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(ownship_state.size(), n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = data.obstacles.size();
	int n_static_obst = polygons.size();

	// Predict nominal trajectory first, assign as optimal if no need for
	// COLAV, or use in the prediction initialization
	for (int M = 0; M < pars.n_M; M++)
	{
		opt_offset_sequence(2 * M) = 1.0; opt_offset_sequence(2 * M + 1) = 0.0;
	}
	maneuver_times.setZero();
	ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1.0; 		u_opt_last = u_opt;
		chi_opt = 0.0; 		chi_opt_last = chi_opt;

		assign_optimal_trajectory(predicted_trajectory);

		return;
	}
	
	setup_prediction(data);
	
	//prune_obstacle_scenarios(data);

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	} */
	/*
	mxArray *init_state_os_mx = mxCreateDoubleMatrix(ownship_state.size(), 1, mxREAL);
 	mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

	double *p_init_state_os = mxGetPr(init_state_os_mx); 
	double *ptraj_os_mx = mxGetPr(traj_os_mx); 
	double *p_wps_os = mxGetPr(wps_os); 
	
	Eigen::Map<Eigen::VectorXd> map_init_state_os(p_init_state_os, ownship_state.size(), 1);
	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
	map_init_state_os = ownship_state;
	map_wps = waypoints;

	mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_obst_mx, *n_static_obst_mx, *i_mx, *ps_mx, *d_safe_mx;
	dt_sim = mxCreateDoubleScalar(pars.dt);
	T_sim = mxCreateDoubleScalar(pars.T);
	n_ps_mx = mxCreateDoubleScalar(n_ps[0]);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
	n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);
	
	engPutVariable(ep, "ownship_state", init_state_os_mx);
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "dt_sim", dt_sim);
	engPutVariable(ep, "T_sim", T_sim);
	engPutVariable(ep, "WPs", wps_os);
	engPutVariable(ep, "d_safe", d_safe_mx);
	engEvalString(ep, "inside_psbmpc_init_plot");

	Eigen::Matrix<double, 2, -1> polygon_matrix;
    int n_total_vertices = 0;
    mxArray *polygon_matrix_mx(nullptr);
	mxArray *d_0j_mx = mxCreateDoubleMatrix(2, 1, mxREAL);
	mxArray *j_mx(nullptr);
    double *p_polygon_matrix(nullptr);
	double *p_d_0j = mxGetPr(d_0j_mx);
    
	Eigen::Map<Eigen::Vector2d> map_d_0j(p_d_0j, 2, 1);
	int pcount;
	Eigen::Vector2d d_0j;
	for (int j = 0; j < n_static_obst; j++)
	{
		j_mx = mxCreateDoubleScalar(j + 1);
		n_total_vertices = 0;
		for(auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
		{
			n_total_vertices += 1;
		}
		polygon_matrix.resize(2, n_total_vertices);
		pcount = 0;
		for(auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
		{
			polygon_matrix(0, pcount) = boost::geometry::get<0>(*it);
			polygon_matrix(1, pcount) = boost::geometry::get<1>(*it);
			
			pcount += 1;
		}
		d_0j = mpc_cost.distance_to_polygon(ownship_state.block<2, 1>(0, 0), polygons[j]);
		map_d_0j = d_0j;

		polygon_matrix_mx = mxCreateDoubleMatrix(2, n_total_vertices, mxREAL);
		p_polygon_matrix = mxGetPr(polygon_matrix_mx);
		Eigen::Map<Eigen::MatrixXd> map_polygon_matrix(p_polygon_matrix, 2, n_total_vertices);
		map_polygon_matrix = polygon_matrix;
		engPutVariable(ep, "j", j_mx);
		engPutVariable(ep, "d_0j", d_0j_mx);
		engPutVariable(ep, "polygon_matrix_j", polygon_matrix_mx);
		engEvalString(ep, "inside_psbmpc_static_obstacle_plot");
	}

	mxArray *traj_i = mxCreateDoubleMatrix(4, n_samples, mxREAL);
	mxArray *P_traj_i = mxCreateDoubleMatrix(16, n_samples, mxREAL);

	double *ptraj_i = mxGetPr(traj_i);
	double *p_P_traj_i = mxGetPr(P_traj_i);
	double *p_P_c_i;

	Eigen::Map<Eigen::MatrixXd> map_traj_i(ptraj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_P_traj_i(p_P_traj_i, 16, n_samples);

	std::vector<mxArray*> P_c_i_mx(n_obst);

 	for(int i = 0; i < n_obst; i++)
	{
		P_c_i_mx[i] = mxCreateDoubleMatrix(n_ps[i], n_samples, mxREAL);

		Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
		std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

		i_mx = mxCreateDoubleScalar(i + 1);
		engPutVariable(ep, "i", i_mx);

		map_P_traj_i = P_i_p;
		engPutVariable(ep, "P_i_flat", P_traj_i);
		for (int ps = 0; ps < n_ps[i]; ps++)
		{
			ps_mx = mxCreateDoubleScalar(ps + 1);
			engPutVariable(ep, "ps", ps_mx);

			map_traj_i = xs_i_p[ps];
			
			engPutVariable(ep, "X_i", traj_i);
			engEvalString(ep, "inside_psbmpc_obstacle_plot");
		}
	}  */
	
	//===============================================================================================================

	set_up_temporary_device_memory(u_d, chi_d, waypoints, V_w, wind_direction, polygons, data);
	//===============================================================================================================
	// Ownship trajectory prediction for all control behaviours 
	//===============================================================================================================
	cb_cost_functor_1.reset(new CB_Cost_Functor_1(
		pars_device_ptr, 
		fdata_device_ptr, 
		ownship_device_ptr, 
		trajectory_device_ptr, 
		mpc_cost_device_ptr));

	cb_costs_1_dvec.resize(pars.n_cbs);
	cb_index_dvec.resize(pars.n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

	map_offset_sequences();

	auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), cb_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), cb_dvec.end()));
	
	thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs_1_dvec.begin(), *cb_cost_functor_1);
	cuda_check_errors("Thrust transform for cost calculation part one failed.");

	//===============================================================================================================
	// Prepare thrust device vectors for the second cost functor call
	//===============================================================================================================
	map_thrust_dvecs(polygons);

	//===============================================================================================================
	// Cost evaluation for all control behaviours wrt one static obstacle OR a dynamic obstacle behaving as in 
	// a certain prediction scenario
	//===============================================================================================================
	cb_cost_functor_2.reset(new CB_Cost_Functor_2(
		pars_device_ptr, 
		fdata_device_ptr, 
		polygons_device_ptr,
		obstacles_device_ptr, 
		cpe_device_ptr, 
		ownship_device_ptr,
		trajectory_device_ptr, 
		mpc_cost_device_ptr));

	auto input_tuple_2_begin = thrust::make_zip_iterator(thrust::make_tuple(
		thread_index_dvec.begin(), 
		cb_dvec.begin(),
		cb_index_dvec.begin(),
		sobstacle_index_dvec.begin(),
		dobstacle_index_dvec.begin(),
		dobstacle_ps_index_dvec.begin(),
		cpe_index_dvec.begin()));
    auto input_tuple_2_end = thrust::make_zip_iterator(thrust::make_tuple(
		thread_index_dvec.end(), 
		cb_dvec.end(),
		cb_index_dvec.end(),
		sobstacle_index_dvec.end(),
		dobstacle_index_dvec.end(),
		dobstacle_ps_index_dvec.end(),
		cpe_index_dvec.end()));

    thrust::transform(input_tuple_2_begin, input_tuple_2_end, cb_costs_2_dvec.begin(), *cb_cost_functor_2);
	cuda_check_errors("Thrust transform for cost calculation part three failed.");

	//===============================================================================================================
	// Stitch together the GPU calculated costs to form the cost function for each control behaviour
	// and find the optimal solution
	//===============================================================================================================
	find_optimal_control_behaviour(data, polygons);

	ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
	assign_optimal_trajectory(predicted_trajectory);
	//===============================================================================================================

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Eigen::Map<Eigen::MatrixXd> map_traj(ptraj_os, trajectory.rows(), n_samples);
	map_traj = trajectory;

	k_s = mxCreateDoubleScalar(n_samples);
	engPutVariable(ep, "k", k_s);

	engPutVariable(ep, "X", traj_os);
	engEvalString(ep, "inside_psbmpc_upd_ownship_plot");  

	engClose(ep);*/
	//===============================================================================================================

	u_opt = opt_offset_sequence(0); 		u_opt_last = u_opt;
	chi_opt = opt_offset_sequence(1); 		chi_opt_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
void PSBMPC::preallocate_device_data()
{
	std::cout << "CB_Functor_Pars size: " << sizeof(CB_Functor_Pars) << std::endl;
	std::cout << "CB_Functor_Data size: " << sizeof(CB_Functor_Data) << std::endl;
	std::cout << "Ownship size: " << sizeof(Ownship) << std::endl;
	std::cout << "Ownship trajectory size: " << sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>) << std::endl; 
	std::cout << "CPE size: " << sizeof(CPE) << std::endl;
	std::cout << "Cuda Obstacle size: " << sizeof(Cuda_Obstacle) << std::endl; 
	std::cout << "Obstacle Ship size: " << sizeof(Obstacle_Ship) << std::endl;
	std::cout << "Basic polygon size: " << sizeof(Basic_Polygon) << std::endl;
	std::cout << "MPC_Cost<CB_Functor_Pars> size: " << sizeof(MPC_Cost<CB_Functor_Pars>) << std::endl;

	//================================================================================
	// Cuda device memory allocation, preallocated to save computation time
	//================================================================================
	// Allocate for use by all threads a read-only Cuda_Obstacle array
	cudaMalloc((void**)&obstacles_device_ptr, MAX_N_OBST * sizeof(Cuda_Obstacle));
    cuda_check_errors("CudaMalloc of Cuda_Obstacle's failed.");
	
	// Allocate for use by all threads a control behaviour parameter object
	CB_Functor_Pars temp_pars(pars); 
	cudaMalloc((void**)&pars_device_ptr, sizeof(CB_Functor_Pars));
	cuda_check_errors("CudaMalloc of CB_Functor_Pars failed.");

	cudaMemcpy(pars_device_ptr, &temp_pars, sizeof(CB_Functor_Pars), cudaMemcpyHostToDevice);
    cuda_check_errors("CudaMemCpy of CB_Functor_Pars failed.");

	// Allocate for use by all threads a control behaviour data object
	cudaMalloc((void**)&fdata_device_ptr, sizeof(CB_Functor_Data));
	cuda_check_errors("CudaMalloc of CB_Functor_Data failed.");
	
	// Allocate for each control behaviour an own-ship trajectory
	cudaMalloc((void**)&trajectory_device_ptr, pars.n_cbs * sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of trajectory failed.");

	// Allocate for each control behaviour an ownship
	cudaMalloc((void**)&ownship_device_ptr, pars.n_cbs * sizeof(Ownship));
	cuda_check_errors("CudaMalloc of Ownship failed.");

	// Allocate for each thread that considers dynamic obstacles a Collision Probability Estimator
	CPE temp_cpe(pars.cpe_method, pars.dt);
	cudaMalloc((void**)&cpe_device_ptr, pars.n_cbs * MAX_N_OBST * pars.n_r * sizeof(CPE));
    cuda_check_errors("CudaMalloc of CPE failed.");

	// Allocate for each thread an mpc cost object
	MPC_Cost<CB_Functor_Pars> temp_mpc_cost(temp_pars);
	int max_n_threads = pars.n_cbs * (MAX_N_POLYGONS + MAX_N_OBST * pars.n_r);
	cudaMalloc((void**)&mpc_cost_device_ptr, max_n_threads * sizeof(MPC_Cost<CB_Functor_Pars>));
	cuda_check_errors("CudaMalloc of MPC_Cost failed.");

	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cudaMemcpy(&ownship_device_ptr[cb], &ownship, sizeof(Ownship), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of Ownship failed.");		
	}

	for (int thread = 0; thread < pars.n_cbs * MAX_N_OBST * pars.n_r; thread++)
	{
		cudaMemcpy(&cpe_device_ptr[thread], &temp_cpe, sizeof(CPE), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of CPE failed.");
	}

	for (int thread = 0; thread < max_n_threads; thread++)
	{
		cudaMemcpy(&mpc_cost_device_ptr[thread], &temp_mpc_cost, sizeof(MPC_Cost<CB_Functor_Pars>), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of MPC_Cost failed.");
	}

	// Allocate for use by all threads that consider static obstacles a read-only Basic_Polygon array
	cudaMalloc((void**)&polygons_device_ptr, MAX_N_POLYGONS * sizeof(Basic_Polygon));
    cuda_check_errors("CudaMalloc of Basic_Polygon`s failed.");
}

/****************************************************************************************
*  Name     : determine_colav_active
*  Function : Uses the dynamic obstacle vector and the number of static 
*			  obstacles to determine whether it is necessary to run the PSBMPC
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
bool PSBMPC::determine_colav_active(
	const Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const int n_static_obst 												// In: Number of static obstacles
	)
{
	Eigen::VectorXd xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].kf.get_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].kf.get_state()(1) - xs(1);
		if (d_0i.norm() < pars.d_init) colav_active = true;

		// If all obstacles are passed, even though inside colav range,
		// then no need for colav
		if (data.IP_0[i]) 	{ colav_active = false; }
		else 				{ colav_active = true; }
	}
	colav_active = colav_active || n_static_obst > 0;

	return colav_active;
}

/****************************************************************************************
*  Name     : map_offset_sequences
*  Function : Maps the currently set surge and course modifications into a matrix of 
*			  offset sequences or control behaviours
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::map_offset_sequences()
{
	Eigen::VectorXd offset_sequence_counter(2 * pars.n_M), offset_sequence(2 * pars.n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);

	TML::PDMatrix<float, 2 * MAX_N_M, 1> tml_offset_sequence(2 * pars.n_M, 1);

	cb_dvec.resize(pars.n_cbs);
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		TML::assign_eigen_object(tml_offset_sequence, offset_sequence);

		cb_dvec[cb] = tml_offset_sequence;

		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
	std::cout << "Number of control behaviours: " << pars.n_cbs << std::endl;
}

/****************************************************************************************
*  Name     : reset_control_behavior
*  Function : Sets the offset sequence back to the initial starting point, i.e. the 
*			  leftmost branch of the control behavior tree
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::reset_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
)
{
	offset_sequence_counter.setZero();
	for (int M = 0; M < pars.n_M; M++)
	{
		offset_sequence(2 * M) = pars.u_offsets[M](0);
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](0);
	}
}

/****************************************************************************************
*  Name     : increment_control_behavior
*  Function : Increments the control behavior counter and changes the offset sequence 
*			  accordingly. Backpropagation is used for the incrementation
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::increment_control_behaviour(
	Eigen::VectorXd &offset_sequence_counter, 									// In/out: Counter to keep track of current offset sequence
	Eigen::VectorXd &offset_sequence 											// In/out: Control behaviour to increment
	)
{
	for (int M = pars.n_M - 1; M > -1; M--)
	{
		// Only increment counter for "leaf node offsets" on each iteration, which are the
		// course offsets in the last maneuver
		if (M == pars.n_M - 1)
		{
			offset_sequence_counter(2 * M + 1) += 1;
		}

		// If one reaches the end of maneuver M's course offsets, reset corresponding
		// counter and increment surge offset counter above
		if (offset_sequence_counter(2 * M + 1) == pars.chi_offsets[M].size())
		{
			offset_sequence_counter(2 * M + 1) = 0;
			offset_sequence_counter(2 * M) += 1;
		}
		offset_sequence(2 * M + 1) = pars.chi_offsets[M](offset_sequence_counter(2 * M + 1));

		// If one reaches the end of maneuver M's surge offsets, reset corresponding
		// counter and increment course offset counter above (if any)
		if (offset_sequence_counter(2 * M) == pars.u_offsets[M].size())
		{
			offset_sequence_counter(2 * M) = 0;
			if (M > 0)
			{
				offset_sequence_counter(2 * M - 1) += 1;
			}
		}
		offset_sequence(2 * M) = pars.u_offsets[M](offset_sequence_counter(2 * M));
	}
}

/****************************************************************************************
*  Name     : map_thrust_input_dvecs
*  Function : Fills the device vectors in/out of the thrust calls with the proper
*			  flattened values. Number of threads will be 
*			  n_threads_1 = n_cbs * n_so for the second kernel, and 
*			  n_threads_2 = n_cbs * (n_ps^1 + ... + n_ps^n_obst) for the third kernel
*			  
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::map_thrust_dvecs(
	const std::vector<polygon_2D> &polygons 								// In: Static obstacle information
	)
{
	// Figure out how many threads to schedule
	int n_obst = n_ps.size();
	int n_obst_ps_total(0);
	int n_static_obst = polygons.size();
	for (int i = 0; i < n_obst; i++)
	{
		n_obst_ps_total += n_ps[i];
	}
	// Total number of GPU threads to schedule
	int n_threads = pars.n_cbs * (n_static_obst + n_obst_ps_total);
	std::cout << "n_threads = " << n_threads << " | n_static_obst = " << n_static_obst << " | n_obst_ps_total = " << n_obst_ps_total << std::endl;

	cb_dvec.resize(n_threads);
	cb_index_dvec.resize(n_threads);
	sobstacle_index_dvec.resize(n_threads);
	dobstacle_index_dvec.resize(n_threads);
	dobstacle_ps_index_dvec.resize(n_threads);
	cpe_index_dvec.resize(n_threads);
	thread_index_dvec.resize(n_threads);
	thrust::sequence(thread_index_dvec.begin(), thread_index_dvec.end(), 0);
	cb_costs_2_dvec.resize(n_threads);

	int thread_index(0), cpe_index(0);
	TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence_tml(2 * pars.n_M);
	Eigen::VectorXd offset_sequence_counter(2 * pars.n_M), offset_sequence(2 * pars.n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		TML::assign_eigen_object(offset_sequence_tml, offset_sequence);

		for (int j = 0; j < n_static_obst; j++)
		{
			cb_dvec[thread_index] = offset_sequence_tml;
			cb_index_dvec[thread_index] = cb;

			sobstacle_index_dvec[thread_index] = j;

			// Set dynamic obstacle, prediction scenario index  and cpe index to -1 
			// for threads where only the grounding cost should be computed
			dobstacle_index_dvec[thread_index] = -1;
			dobstacle_ps_index_dvec[thread_index] = -1;
			cpe_index_dvec[thread_index] = -1;
			thread_index += 1;

			/* printf("thread %d | ", thread_index - 1);
			printf("cb = ");
			for (int M = 0; M < pars.n_M; M++)
			{
				printf("%.2f, %.2f", offset_sequence(2 * M), RAD2DEG * offset_sequence(2 * M + 1));
				if (M < pars.n_M - 1) printf(", ");
			}
			printf(" | cb_index = %d | j = %d | i = %d | ps = %d\n", cb, j, -1, -1); */
		}

		for (int i = 0; i < n_obst; i++)
		{	
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				cb_dvec[thread_index] = offset_sequence_tml;
				cb_index_dvec[thread_index] = cb;

				// Set static obstacle index to -1 for threads where only dynamic
				// obstacle related costs/indicators should be computed
				sobstacle_index_dvec[thread_index] = -1;

				dobstacle_index_dvec[thread_index] = i;
				dobstacle_ps_index_dvec[thread_index] = ps;
				cpe_index_dvec[thread_index] = cpe_index;
				
				thread_index += 1;
				cpe_index += 1;

				/* printf("thread %d | ", thread_index - 1);
				printf("cb = ");
				for (int M = 0; M < pars.n_M; M++)
				{
					printf("%.2f, %.2f", offset_sequence(2 * M), RAD2DEG * offset_sequence(2 * M + 1));
					if (M < pars.n_M - 1) printf(", ");
				}
				printf(" | cb_index = %d | j = %d | i = %d | ps = %d\n", cb, -1, i, ps); */
			}
		}
		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
}

/****************************************************************************************
*  Name     : find_optimal_control_behaviour
*  Function : Goes through the GPU calculated costs, extracts the total cost for each
*			  control behaviour and finds the optimal one giving minimal cost
*			  
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::find_optimal_control_behaviour(
	Obstacle_Data<Tracked_Obstacle> &data,									// In/Out: Dynamic obstacle information
	const std::vector<polygon_2D> &polygons 								// In: Static obstacle information
)
{
	int n_obst = data.obstacles.size();
	int n_static_obst = polygons.size();
	int thread_index(0);
	Eigen::VectorXd offset_sequence_counter(2 * pars.n_M), offset_sequence(2 * pars.n_M);
	reset_control_behaviour(offset_sequence_counter, offset_sequence);

	Eigen::VectorXd max_cost_i_ps, max_cost_j(n_static_obst), mu_i_ps, mu_i(n_obst), cost_do(n_obst);

	double cost(0.0), h_do, h_colregs, h_so, h_path;
	min_cost = 1e12;

	Eigen::MatrixXd cost_do_matrix(n_obst, pars.n_cbs);
	Eigen::MatrixXd cost_colregs_matrix(1, pars.n_cbs);
	Eigen::MatrixXd max_cost_i_ps_matrix(n_obst * pars.n_r, pars.n_cbs);
	Eigen::MatrixXd max_cost_j_matrix(n_static_obst, pars.n_cbs);
	Eigen::MatrixXd mu_i_ps_matrix(n_obst * pars.n_r, pars.n_cbs);
	Eigen::MatrixXd cb_matrix(2 * pars.n_M, pars.n_cbs);
	Eigen::MatrixXd cost_so_path_matrix(2, pars.n_cbs);
	Eigen::MatrixXd total_cost_matrix(1, pars.n_cbs);
	Eigen::MatrixXd n_ps_matrix(1, n_obst);
	Eigen::MatrixXd Pr_s_i_matrix(n_obst, n_ps[0]);
	for (int i = 0; i < n_obst; i++)
	{
		n_ps_matrix(0, i) = n_ps[i];
		for (int ps = 0; ps < n_ps[i]; ps++)
		{
			Pr_s_i_matrix(i, ps) = data.obstacles[i].get_scenario_probabilities()(ps);
		}
	}
	int curr_ps_index(0);
	//==================================================================
	// MATLAB PLOTTING FOR DEBUGGING AND TUNING
	//==================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	mxArray *total_cost_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
 	mxArray *cost_do_mx = mxCreateDoubleMatrix(n_obst, pars.n_cbs, mxREAL);
	mxArray *cost_colregs_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
	mxArray *max_cost_i_ps_mx = mxCreateDoubleMatrix(n_obst * pars.n_r, pars.n_cbs, mxREAL);
	mxArray *max_cost_j_mx = mxCreateDoubleMatrix(n_static_obst, pars.n_cbs, mxREAL);
	mxArray *mu_i_ps_mx = mxCreateDoubleMatrix(n_obst * pars.n_r, pars.n_cbs, mxREAL);
	mxArray *cost_so_path_mx = mxCreateDoubleMatrix(2, pars.n_cbs, mxREAL);
	mxArray *n_ps_mx = mxCreateDoubleMatrix(1, n_obst, mxREAL);
	mxArray *cb_matrix_mx = mxCreateDoubleMatrix(2 * pars.n_M, pars.n_cbs, mxREAL);
	mxArray *Pr_s_i_mx = mxCreateDoubleMatrix(n_obst, n_ps[0], mxREAL);
	
	double *ptr_total_cost = mxGetPr(total_cost_mx); 
	double *ptr_cost_do = mxGetPr(cost_do_mx); 
	double *ptr_cost_colregs = mxGetPr(cost_colregs_mx); 
	double *ptr_max_cost_i_ps = mxGetPr(max_cost_i_ps_mx); 
	double *ptr_max_cost_j = mxGetPr(max_cost_j_mx); 
	double *ptr_mu_i_ps = mxGetPr(mu_i_ps_mx); 
	double *ptr_cost_so_path = mxGetPr(cost_so_path_mx); 
	double *ptr_n_ps = mxGetPr(n_ps_mx); 
	double *ptr_cb_matrix = mxGetPr(cb_matrix_mx); 
	double *ptr_Pr_s_i = mxGetPr(Pr_s_i_mx); 

	Eigen::Map<Eigen::MatrixXd> map_total_cost(ptr_total_cost, 1, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_do(ptr_cost_do, n_obst, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_colregs(ptr_cost_colregs, 1, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_max_cost_i_ps(ptr_max_cost_i_ps, n_obst * pars.n_r, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_max_cost_j(ptr_max_cost_j, n_static_obst, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_mu_i_ps(ptr_mu_i_ps, n_obst * pars.n_r, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_cost_so_path(ptr_cost_so_path, 2, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_n_ps(ptr_n_ps, 1, n_obst);
	Eigen::Map<Eigen::MatrixXd> map_cb_matrix(ptr_cb_matrix, 2 * pars.n_M, pars.n_cbs);
	Eigen::Map<Eigen::MatrixXd> map_Pr_s_i(ptr_Pr_s_i, n_obst, n_ps[0]);

	mxArray *n_obst_mx = mxCreateDoubleScalar(n_obst), *opt_cb_index_mx(nullptr); */
	//==================================================================
	std::tuple<double, double> tup;
	thrust::tuple<float, float, float> dev_tup;
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		for (int M = 0; M < pars.n_M; M++)
		{
			cb_matrix(2 * M, cb) = offset_sequence(2 * M);
			cb_matrix(2 * M + 1, cb) = RAD2DEG * offset_sequence(2 * M + 1);
		}
		//std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
		//std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
		cost = 0.0;

		for (int j = 0; j < n_static_obst; j++)
		{
			dev_tup = cb_costs_2_dvec[thread_index];
			max_cost_j(j) = (double)thrust::get<0>(dev_tup);
			thread_index += 1;
			/* printf("Thread %d | j = %d | i = -1 | ps = -1 | max cost j : %.4f | max cost i ps : DC | mu_i_ps : DC | cb_index : %d | cb : %.1f, %.1f \n", 
					thread_index, j, max_cost_j(j), cb, offset_sequence(0), RAD2DEG * offset_sequence(1)); */
		}
		max_cost_j_matrix.block(0, cb, n_static_obst, 1) = max_cost_j;
		h_so = max_cost_j.maxCoeff();
		cost_so_path_matrix(0, cb) = h_so;

		curr_ps_index = 0;
		for (int i = 0; i < n_obst; i++)
		{
			max_cost_i_ps.resize(n_ps[i]); mu_i_ps.resize(n_ps[i]);
			for (int ps = 0; ps < n_ps[i]; ps++)
			{
				dev_tup = cb_costs_2_dvec[thread_index];
				max_cost_i_ps(ps) = (double)thrust::get<1>(dev_tup);
				mu_i_ps(ps) = (double)thrust::get<2>(dev_tup);
				thread_index += 1;
				/* printf("Thread %d | j = -1 | i = %d | ps = %d | max cost j : DC | max cost i ps : %.4f | mu_i_ps : %.1f | cb_index : %d | cb : %.1f, %.1f \n", 
					thread_index, i, ps, max_cost_i_ps(ps), mu_i_ps(ps), cb, offset_sequence(0), RAD2DEG * offset_sequence(1)); */
			}

			tup = mpc_cost.calculate_dynamic_obstacle_cost(max_cost_i_ps, mu_i_ps, data, i);
			cost_do(i) = std::get<0>(tup);
			mu_i(i) = std::get<1>(tup);

			// Matlab related data structure
			max_cost_i_ps_matrix.block(curr_ps_index, cb, n_ps[i], 1) = max_cost_i_ps;
			mu_i_ps_matrix.block(curr_ps_index, cb, n_ps[i], 1) = mu_i_ps;
			curr_ps_index += n_ps[i];
		}

		h_do = cost_do.sum();
		cost_do_matrix.col(cb) = cost_do;

		h_colregs = pars.kappa * std::min(1.0, mu_i.sum());
		cost_colregs_matrix(0, cb) = h_colregs;

		h_path = cb_costs_1_dvec[cb];
		cost_so_path_matrix(1, cb) = h_path;

		cost = h_do + h_colregs + h_so + h_path;
		total_cost_matrix(cb) = cost;

		if (cost < min_cost)
		{
			min_cost = cost;
			min_index = cb;
			opt_offset_sequence = offset_sequence;
		}

		increment_control_behaviour(offset_sequence_counter, offset_sequence);
	}
	//==================================================================
	// MATLAB PLOTTING FOR DEBUGGING AND TUNING
	//==================================================================
	/* opt_cb_index_mx = mxCreateDoubleScalar(min_index + 1);
	map_total_cost = total_cost_matrix;
	map_cost_do = cost_do_matrix;
	map_cost_colregs = cost_colregs_matrix;
	map_max_cost_i_ps = max_cost_i_ps_matrix;
	map_max_cost_j = max_cost_j_matrix;
	map_mu_i_ps = mu_i_ps_matrix;
	map_cost_so_path = cost_so_path_matrix;
	map_n_ps = n_ps_matrix;
	map_cb_matrix = cb_matrix;
	map_Pr_s_i = Pr_s_i_matrix;

	mxArray *is_gpu_mx = mxCreateDoubleScalar(1);
	engPutVariable(ep, "is_gpu", is_gpu_mx);
	engPutVariable(ep, "Pr_s_i", Pr_s_i_mx);
	engPutVariable(ep, "total_cost", total_cost_mx);
	engPutVariable(ep, "cost_do", cost_do_mx);
	engPutVariable(ep, "cost_colregs", cost_colregs_mx);
	engPutVariable(ep, "max_cost_i_ps", max_cost_i_ps_mx);
	engPutVariable(ep, "max_cost_j", max_cost_j_mx);
	engPutVariable(ep, "mu_i_ps", mu_i_ps_mx);
	engPutVariable(ep, "cost_so_path", cost_so_path_mx);
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "cb_matrix", cb_matrix_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "opt_cb_index", opt_cb_index_mx);
	engEvalString(ep, "psbmpc_cost_plotting");

	mxDestroyArray(is_gpu_mx);
	mxDestroyArray(total_cost_mx);
	mxDestroyArray(cost_do_mx);
	mxDestroyArray(cost_colregs_mx);
	mxDestroyArray(max_cost_i_ps_mx);
	mxDestroyArray(max_cost_j_mx);
	mxDestroyArray(mu_i_ps_mx);
	mxDestroyArray(cost_so_path_mx);
	mxDestroyArray(n_ps_mx);
	mxDestroyArray(cb_matrix_mx);
	mxDestroyArray(n_obst_mx);
	mxDestroyArray(opt_cb_index_mx);
	
	engClose(ep); */
	//==================================================================
}

/****************************************************************************************
*  Name     : setup_prediction
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation, and predicts
*			  independent obstacle trajectories using the predictor class.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::setup_prediction(
	Obstacle_Data<Tracked_Obstacle> &data							// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();
	n_ps.resize(n_obst);
	for (int i = 0; i < n_obst; i++)
	{
		n_ps[i] = data.obstacles[i].get_scenario_probabilities().size();
	}
	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa, v_0;
	Eigen::Vector4d xs_0, xs_i_0;
	if (trajectory.rows() == 4)
	{
		v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
		v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
	}
	else
	{
		v_0(0) = trajectory(3, 0); v_0(1) = trajectory(4, 0);
		v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
	}
	xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
	xs_0(2) = v_0(0); xs_0(3) = v_0(1);
	// First avoidance maneuver is always at t0
	maneuver_times.setZero();

	double t_cpa_min(0.0), d_safe_i(0.0);
	std::vector<bool> maneuvered_by(n_obst);
	int index_closest(-1);
	for (int M = 1; M < pars.n_M; M++)
	{
		// This is the solution so far if n_obst = 0. And also:
		// If a predicted collision occurs with the closest obstacle, avoidance maneuver 
		// M is taken right after the obstacle possibly maneuvers (which will be at t_0 + M * t_ts
		// if the independent obstacle prediction scheme is used), given that t_cpa > t_ts. 
		// If t_cpa < t_ts, the subsequent maneuver is taken at t_0 + M * t_ts + 1 anyways (simplification)
		maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);
		
		// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
		t_cpa_min = 1e10; index_closest = -1;
		for (int i = 0; i < n_obst; i++)
		{
			xs_i_0 = data.obstacles[i].kf.get_state();
			CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());
			// For the current avoidance maneuver, determine which obstacle that should be
			// considered, i.e. the closest obstacle that is not already passed (which means
			// that the previous avoidance maneuver happened before CPA with this obstacle)
			if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min && t_cpa(i) <= pars.T)
			{	
				t_cpa_min = t_cpa(i);
				index_closest = i;
			}	
		}

		if (index_closest != -1)
		{
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[index_closest].get_width());
			// If no predicted collision,  avoidance maneuver M with the closest
			// obstacle (that is not passed) is taken at t_cpa_min
			if (d_cpa(index_closest) > d_safe_i)
			{
				//std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
				maneuvered_by[index_closest] = true;
				maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
			}
		}
	}
	
	std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
}

/****************************************************************************************
*  Name     : prune_obstacle_scenarios
*  Function : Goes through all generated obstacle prediction scenarios, and selects the
*			  N_r scenarios with highest collision risk for keeping, discarding all others.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::prune_obstacle_scenarios(
	Obstacle_Data<Tracked_Obstacle> &data							// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();

	int p_step = 2;   
	double dt_r = (double)p_step * pars.dt;
	int n_samples = std::round(pars.T / pars.dt);
	int n_ps_new(0);
	int relevant_joint_pred_scenarios_count(0);

	Eigen::MatrixXd P_c_i;
	Eigen::VectorXi risk_sorted_ps_indices_i;
	Eigen::VectorXi kept_ps_indices_i;
	Eigen::VectorXd R_c_i, P_c_i_ps, C_i;
	for (int i = 0; i < n_obst; i++)
	{			
		P_c_i.resize(n_ps[i], n_samples); P_c_i_ps.resize(n_ps[i]); 
		C_i.resize(n_ps[i]); R_c_i.resize(n_ps[i]);
		risk_sorted_ps_indices_i.resize(n_ps[i]);
		
		calculate_collision_probabilities(P_c_i, data, i, dt_r, p_step);

		calculate_ps_collision_probabilities(P_c_i_ps, P_c_i, i);

		calculate_ps_collision_consequences(C_i, data, i, dt_r, p_step);

		calculate_ps_collision_risks(R_c_i, risk_sorted_ps_indices_i, C_i, P_c_i_ps, data, i);

		//std::cout << risk_sorted_ps_indices_i.transpose() << std::endl;

		// Keep only the n_r prediction scenarios with the highest collision risk
		if (n_ps[i] < pars.n_r)
		{
			kept_ps_indices_i = risk_sorted_ps_indices_i;
		}
		else
		{
			kept_ps_indices_i = risk_sorted_ps_indices_i.block(0, 0, pars.n_r, 1);		
		}

		n_ps_new = kept_ps_indices_i.size(); 
		// Sort indices of ps that are to be kept
		std::sort(kept_ps_indices_i.data(), kept_ps_indices_i.data() + kept_ps_indices_i.size());
		//std::cout << kept_ps_indices_i.transpose() << std::endl;

		n_ps[i] = n_ps_new;
		data.obstacles[i].prune_ps(kept_ps_indices_i);
	}
}

/****************************************************************************************
*  Name     : calculate_collision_probabilities
*  Function : Estimates collision probabilities for the own-ship and an obstacle i in
*			  consideration. Can use a larger sample time than used in predicting
*			  the vessel trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_collision_probabilities(
	Eigen::MatrixXd &P_c_i,								// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const Obstacle_Data<Tracked_Obstacle> &data,		// In: Dynamic obstacle information
	const int i, 										// In: Index of obstacle
	const double dt, 									// In: Sample time for estimation
	const int p_step                                    // In: Step between trajectory samples, matches the input prediction time step
	)
{
	Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
	std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

	// Increase safety zone by half the max obstacle dimension and ownship length
	double d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());

	int n_samples = P_i_p.cols();
	// Non-optimal temporary row-vector storage solution
	Eigen::Matrix<double, 1, -1> P_c_i_row(n_samples);
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		cpe_host.estimate_over_trajectories(P_c_i_row, trajectory, xs_i_p[ps], P_i_p, d_safe_i, dt, p_step);

		P_c_i.block(ps, 0, 1, P_c_i_row.cols()) = P_c_i_row;
	}		
}

/****************************************************************************************
*  Name     : calculate_ps_collision_probabilities
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated collision probabilities.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_probabilities(
	Eigen::VectorXd &P_c_i_ps,											// In/out: Vector of collision consequences, size n_ps_i x 1
	const Eigen::MatrixXd &P_c_i,										// In: Predicted obstacle collision probabilities for all prediction scenarios, size n_ps[i] x n_samples
	const int i															// In: Index of obstacle
	)
{
	P_c_i_ps.setZero();

	//double product(0.0);
	int n_samples = P_c_i.cols();
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k++)
		{
			/* if (k == 0)	{ product = 1 - P_c_i(ps, k); }
			else		{ product *= (1 - P_c_i(ps, k)); } */
			if (P_c_i(ps, k) > P_c_i_ps(ps))
			{
				P_c_i_ps(ps) = P_c_i(ps, k);
			}
		}
		//P_c_i_ps(ps) = 1 - product;
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_consequences
*  Function : Goes through all generated obstacle prediction scenarios, and calculates
*			  the associated consequence of collision.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_consequences(
	Eigen::VectorXd &C_i, 												// In/out: Vector of collision consequences, size n_ps_i x 1
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i,														// In: Index of obstacle
	const double dt,													// In: Time step between predicted trajectory samples
	const int p_step													// In: Step between trajectory samples, matches the input prediction time step
	)
{
	C_i.setZero();

	double collision_consequence(0.0), t(0.0), t_cpa(0.0), d_cpa(0.0);

	std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();

	Eigen::Vector2d p_cpa, v_0_p, v_i_p;

	int n_samples = xs_i_p[0].cols();

	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k += p_step)
		{
			t = k * dt;

			if (trajectory.rows() == 4)
			{
				v_0_p(0) = trajectory(3, k) * cos(trajectory(2, k));
				v_0_p(1) = trajectory(3, k) * sin(trajectory(2, k));
			}
			else
			{
				v_0_p = trajectory.block<2, 1>(3, k);
				v_0_p = CPU::rotate_vector_2D(v_0_p, trajectory(2, k));
			}

			v_i_p = xs_i_p[ps].block<2, 1>(2, k);

			CPU::calculate_cpa(p_cpa, t_cpa, d_cpa, trajectory.col(k), xs_i_p[ps].col(k));		

			collision_consequence = pow((v_0_p - v_i_p).norm(), 2) * exp(- abs(t - t_cpa));
			//collision_consequence = pow((v_0_p - v_i_p).norm(), 2) * exp(- abs(t));

			if (C_i(ps) < collision_consequence)
			{
				C_i(ps) = collision_consequence;
			}
		}
	}
}

/****************************************************************************************
*  Name     : calculate_ps_collision_risks
*  Function : Goes through all generated obstacle i prediction scenarios, and calculates
*			  the predicted collision risk.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::calculate_ps_collision_risks(
	Eigen::VectorXd &R_c_i,												// In/out: Vector of collision risks, size n_ps_i x 1
	Eigen::VectorXi &ps_indices_i,										// In/out: Vector of indices for the ps, in decending order wrt collision risk, size n_ps_i x 1
	const Eigen::VectorXd &C_i,											// In: Vector of collision consequences, size n_ps_i x 1
	const Eigen::VectorXd &P_c_i_ps,									// In: Vector of collision probabilities, size n_ps_i x 1
	const Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const int i															// In: Index of obstacle	
	)
{
	R_c_i.setZero();

	double Pr_ps_i_conditional(0.0);
	Eigen::VectorXd Pr_c_i_conditional(n_ps[i]);
	Eigen::VectorXd Pr_s_i = data.obstacles[i].get_scenario_probabilities();
	
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		ps_indices_i[ps] = ps;

		Pr_c_i_conditional(ps) = P_c_i_ps(ps) * Pr_s_i(ps);

		R_c_i(ps) = C_i(ps) * Pr_c_i_conditional(ps);
	}

	// Sort vector of ps indices to determine collision risk in sorted order
	std::sort(ps_indices_i.data(), ps_indices_i.data() + n_ps[i], [&](const int index_lhs, const int index_rhs) { return R_c_i(index_lhs) > R_c_i(index_rhs); });
	
	/* std::ios::fmtflags old_settings = std::cout.flags();
	int old_precision = std::cout.precision(); 
	int cw = 20;
	//std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout << std::fixed << std::setprecision(7);

	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
	std::cout << "Obstacle i = " << i << " prediction scenario information:" << std::endl;
	// ps : Prediction scenario, R: Collision risk, C: Collision consequence,
	// P_c^{i, ps}: Collision probability for that scenario
	// Pr{C^i | ps, I^i}: Conditional collision probability for that scenario, on
	// available information in I^i
	
	std::cout << "ps" << std::setw(cw - 4) << "R" << std::setw(cw - 2) << "C" << std::setw(cw + 6) << "P_c^{i, ps}" << std::setw(cw + 4) << "Pr{C^i | s, I^i}" << std::endl;
	for (int j = 0; j < n_ps[i]; j++)
	{
		std::cout 	<< ps_indices_i[j] << std::setw(cw) << R_c_i(ps_indices_i(j)) << std::setw(cw) << C_i(ps_indices_i(j)) << std::setw(cw) 
					<< P_c_i_ps(ps_indices_i(j)) << std::setw(cw) << Pr_c_i_conditional(ps_indices_i(j)) << std::endl;
	}
	std::cout.flags(old_settings);
	std::cout << std::setprecision(old_precision);
	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl; */
}

/****************************************************************************************
*  Name     : determine_situation_type
*  Function : Determines the situation type for vessel A and B  \in {A, B, C, D, E, F}
*  Author   : Trym Tengesdal
*  Modified :
****************************************************************************************/
void PSBMPC::determine_situation_type(
	ST& st_A,																// In/out: Situation type of vessel A
	ST& st_B,																// In/out: Situation type of vessel B	
	const TML::Vector2f &v_A,												// In: (NE) Velocity vector of vessel A 
	const float psi_A, 														// In: Heading of vessel A
	const TML::Vector2f &v_B, 												// In: (NE) Velocity vector of vessel B
	const TML::Vector2f &L_AB, 												// In: LOS vector pointing from vessel A to vessel B
	const float d_AB 														// In: Distance from vessel A to vessel B
	)
{
	// Crash situation or outside consideration range
	if(d_AB < pars.d_safe || d_AB > pars.d_close)
	{
		st_A = A; st_B = A;
		return;
	} 
	// Inside consideration range
	else
	{
		bool is_ahead(false), is_head_on(false), is_crossing(false), is_passed(false);
		bool A_is_overtaken(false), B_is_overtaken(false), B_is_starboard(false);
		is_ahead = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

		A_is_overtaken = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
						v_A.norm() < v_B.norm()							  		&&
						v_A.norm() > 0.25;

		B_is_overtaken = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
						v_B.norm() < v_A.norm()							  		&&
						v_B.norm() > 0.25;

		B_is_starboard = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

		is_passed = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()			&& // Vessel A's perspective	
					!A_is_overtaken) 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 		&& // Vessel B's perspective	
					!B_is_overtaken)) 											&&
					d_AB > pars.d_safe;

		is_head_on = v_A.dot(v_B) < - cos(pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() > 0.25											&&
					v_B.norm() > 0.25											&&
					is_ahead;

		is_crossing = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm()  	&&
					v_A.norm() > 0.25											&&
					v_B.norm() > 0.25											&&
					!is_head_on 												&&
					!is_passed;
		
		if (A_is_overtaken) 
		{ 
			st_A = B; st_B = D;
		} 
		else if (B_is_overtaken) 
		{ 
			st_A = D; st_B = B; 
		} 
		else if (is_head_on) 
		{ 
			st_A = E; st_B = E; 
		} 
		else if (is_crossing)
		{
			if (B_is_starboard) 
			{
				st_A = F; st_B = C;
			} else
			{
				st_A = C; st_B = F;
			}
		} 
		else 
		{
			st_A = A; st_B = A;
		}
	}
}

/****************************************************************************************
*  Name     : assign_optimal_trajectory
*  Function : Set the optimal trajectory to the current predicted trajectory
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::assign_optimal_trajectory(
	Eigen::Matrix<double, 2, -1> &optimal_trajectory 									// In/out: Optimal PSB-MPC trajectory
	)
{
	int n_samples = std::round(pars.T / pars.dt);
	// Set current optimal x-y position trajectory, downsample if linear prediction was not used
	if (false) //(pars.prediction_method > Linear)
	{
		int count = 0;
		optimal_trajectory.resize(2, n_samples / pars.p_step);
		for (int k = 0; k < n_samples; k += pars.p_step)
		{
			optimal_trajectory.col(count) = trajectory.block<2, 1>(0, k);
			if (count < std::round(n_samples / pars.p_step) - 1) count++;					
		}
	} 
	else
	{
		optimal_trajectory.resize(2, n_samples);
		optimal_trajectory = trajectory.block(0, 0, 2, n_samples);
	}
}

/****************************************************************************************
*  Name     : set_up_cuda_memory_transfer
*  Function : Allocates device memory for data that is required on the GPU for the
*			  PSB-MPC calculations. 
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_temporary_device_memory(
	const double u_d,												// In: Own-ship surge reference
	const double chi_d, 											// In: Own-ship course reference
	const Eigen::Matrix<double, 2, -1> &waypoints,					// In: Own-ship waypoints to follow
	const double V_w,												// In: Estimated wind speed
	const Eigen::Vector2d &wind_direction,							// In: Unit vector in NE describing the estimated wind direction
	const std::vector<polygon_2D> &polygons,						// In: Static obstacle information represented as polygons
	const Obstacle_Data<Tracked_Obstacle> &data 					// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();
	int n_static_obst = polygons.size();
	
	size_t limit = 0;

	cudaDeviceSetLimit(cudaLimitStackSize, 100000);
	cuda_check_errors("Setting cudaLimitStackSize failed.");

	cudaDeviceGetLimit(&limit, cudaLimitStackSize);
	cuda_check_errors("Reading cudaLimitStackSize failed.");
	std::cout << "Set device max stack size : " << limit << std::endl;

	// Transfer Functor_Data to the kernels, which is read-only => only need one on the device
	CB_Functor_Data temporary_fdata(
		trajectory, 
		maneuver_times, 
		u_opt_last, 
		chi_opt_last, 
		u_d, chi_d, 
		ownship.get_wp_counter(),
		ownship.get_length(),
		waypoints, 
		V_w, 
		wind_direction,
		polygons,
		n_ps, 
		data);
	cudaMemcpy(fdata_device_ptr, &temporary_fdata, sizeof(CB_Functor_Data), cudaMemcpyHostToDevice);
    cuda_check_errors("CudaMemCpy of CB_Functor_Data failed.");
	
	// Dynamic obstacles
	Cuda_Obstacle temp_transfer_cobstacle;
	for (int i = 0; i < n_obst; i++)
	{
		temp_transfer_cobstacle = data.obstacles[i];

		cudaMemcpy(&obstacles_device_ptr[i], &temp_transfer_cobstacle, sizeof(Cuda_Obstacle), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of Cuda Obstacle i failed.");
	}

	// Static obstacles
	Basic_Polygon temp_transfer_poly;
	for (int j = 0; j < n_static_obst; j++)
	{
		temp_transfer_poly = polygons[j];

		cudaMemcpy(&polygons_device_ptr[j], &temp_transfer_poly, sizeof(Basic_Polygon), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of Basic Polygon j failed.");
	}
}

}
}