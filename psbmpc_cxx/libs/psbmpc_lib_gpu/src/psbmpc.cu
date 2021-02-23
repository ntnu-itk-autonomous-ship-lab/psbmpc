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

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>

#include "utilities.cuh"
#include "psbmpc.cuh"
#include "cuda_obstacle.cuh"
#include "obstacle_ship.cuh"
#include "obstacle_sbmpc.cuh"
#include "mpc_cost.cuh"
#include "cb_cost_functor.cuh"
#include "cb_cost_functor_structures.cuh"

#include <iostream>
#include <iomanip>
#include <chrono>
#include "engine.h"

#define BUFFSIZE 100000

/****************************************************************************************
*  Name     : PSBMPC
*  Function : Class constructor, initializes parameters and variables
*  Author   : 
*  Modified :
*****************************************************************************************/
PSBMPC::PSBMPC() 
	: 
	ownship(Ownship()), pars(PSBMPC_Parameters()), fdata_device_ptr(nullptr), pobstacles_device_ptr(nullptr), obstacles_device_ptr(nullptr), 
	obstacle_ship_device_ptr(nullptr), obstacle_sbmpc_device_ptr(nullptr), mpc_cost_device_ptr(nullptr)
{
	maneuver_times.resize(pars.n_M);
	
	u_opt_last = 1.0; chi_opt_last = 0.0;

	min_cost = 1e12;

	map_offset_sequences();

	cpe_host = CPE_CPU(pars.cpe_method, pars.dt);

	mpc_cost = MPC_Cost<PSBMPC_Parameters>(pars);

	std::cout << "CB_Functor_Pars size: " << sizeof(CB_Functor_Pars) << std::endl;
	std::cout << "CB_Functor_Data size: " << sizeof(CB_Functor_Data) << std::endl;
	std::cout << "Ownship size: " << sizeof(Ownship) << std::endl;
	std::cout << "CPE size: " << sizeof(CPE_GPU) << std::endl;
	std::cout << "Cuda Obstacle size: " << sizeof(Cuda_Obstacle) << std::endl; 
	std::cout << "Prediction Obstacle size: " << sizeof(Prediction_Obstacle) << std::endl;
	std::cout << "Obstacle Ship size: " << sizeof(Obstacle_Ship) << std::endl;
	std::cout << "Obstacle SBMPC size: " << sizeof(Obstacle_SBMPC) << std::endl;
	std::cout << "MPC_Cost<CB_Functor_Pars> size: " << sizeof(MPC_Cost<CB_Functor_Pars>) << std::endl;

	//================================================================================
	// Cuda device memory allocation
	//================================================================================
	// CB_Functor_Data and Cuda_Obstacles are read-only and changed between each 
	// PSBMPC iteration, so are allocated before the thrust transform call 
	

	// Only need to allocate trajectory, ownship, CB_Functor_Pars, CPE and mpc cost device memory once 
	// One trajectory, ownship, CB_Functor_Pars, CPE and mpc cost for each thread, as these are both read and write objects.

	cudaMalloc((void**)&trajectory_device_ptr, pars.n_cbs * sizeof(TML::PDMatrix<float, 6, MAX_N_SAMPLES>));
	cuda_check_errors("CudaMalloc of trajectory failed.");

	// Allocate for use by all threads a control behaviour parameter object
	CB_Functor_Pars temp_pars(pars); 
	cudaMalloc((void**)&pars_device_ptr, sizeof(CB_Functor_Pars));
	cuda_check_errors("CudaMalloc of CB_Functor_Pars failed.");

	cudaMemcpy(pars_device_ptr, &temp_pars, sizeof(CB_Functor_Pars), cudaMemcpyHostToDevice);
    cuda_check_errors("CudaMemCpy of CB_Functor_Pars failed.");

	// Allocate for each thread an ownship
	cudaMalloc((void**)&ownship_device_ptr, pars.n_cbs * sizeof(Ownship));
	cuda_check_errors("CudaMalloc of Ownship failed.");

	// Allocate for each thread a Collision Probability Estimator
	CPE_GPU temp_cpe(pars.cpe_method, pars.dt);
	cudaMalloc((void**)&cpe_device_ptr, pars.n_cbs * sizeof(CPE_GPU));
    cuda_check_errors("CudaMalloc of CPE failed.");

	// Allocate for each thread an mpc cost object
	MPC_Cost<CB_Functor_Pars> temp_mpc_cost(temp_pars);
	cudaMalloc((void**)&mpc_cost_device_ptr, pars.n_cbs * sizeof(MPC_Cost<CB_Functor_Pars>));
	cuda_check_errors("CudaMalloc of MPC_Cost failed.");

	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		cudaMemcpy(&ownship_device_ptr[cb], &ownship, sizeof(Ownship), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of Ownship failed.");

		cudaMemcpy(&cpe_device_ptr[cb], &temp_cpe, sizeof(CPE_GPU), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of CPE failed.");

		cudaMemcpy(&mpc_cost_device_ptr[cb], &temp_mpc_cost, sizeof(MPC_Cost<CB_Functor_Pars>), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of MPC_Cost failed.");
	}

	if (pars.obstacle_colav_on)
	{
		// Allocate for each thread a max number of prediction obstacles
		Prediction_Obstacle temp_pobstacle;
		cudaMalloc((void**)&pobstacles_device_ptr, MAX_N_OBST * pars.n_cbs * sizeof(Prediction_Obstacle));
		cuda_check_errors("CudaMalloc of Prediction obstacles failed.");

		// Allocate for each thread an obstacle ship and obstacle sbmpc
		Obstacle_Ship obstacle_ship; 
		cudaMalloc((void**)&obstacle_ship_device_ptr, pars.n_cbs * sizeof(Obstacle_Ship));
		cuda_check_errors("CudaMalloc of Obstacle_Ship failed.");

		Obstacle_SBMPC obstacle_sbmpc;
		cudaMalloc((void**)&obstacle_sbmpc_device_ptr, pars.n_cbs * sizeof(Obstacle_SBMPC));
		cuda_check_errors("CudaMalloc of Obstacle_SBMPC failed.");

		for (int cb = 0; cb < pars.n_cbs; cb++)
		{
			cudaMemcpy(&obstacle_ship_device_ptr[cb], &obstacle_ship, sizeof(Obstacle_Ship), cudaMemcpyHostToDevice);
			cuda_check_errors("CudaMemCpy of Obstacle_Ship failed.");

			cudaMemcpy(&obstacle_sbmpc_device_ptr[cb], &obstacle_sbmpc, sizeof(Obstacle_SBMPC), cudaMemcpyHostToDevice);
			cuda_check_errors("CudaMemCpy of Obstacle_SBMPC failed.");
		}
	}
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

	fdata_device_ptr = nullptr;

	obstacles_device_ptr = nullptr;	

	cudaFree(cpe_device_ptr);
	cuda_check_errors("CudaFree of CPE failed.");

	cudaFree(ownship_device_ptr);
	cuda_check_errors("CudaFree of Ownship failed.");	

	if (pars.obstacle_colav_on)
	{
		pobstacles_device_ptr = nullptr;

		cudaFree(obstacle_ship_device_ptr);
		cuda_check_errors("CudaFree of Obstacle_Ship failed.");

		cudaFree(obstacle_sbmpc_device_ptr);
		cuda_check_errors("CudaFree of Obstacle_SBMPC failed.");
	}

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
	const Eigen::Matrix<double, 6, 1> &ownship_state, 						// In: Current ship state
	const Eigen::Matrix<double, 4, -1> &static_obstacles,					// In: Static obstacle information
	Obstacle_Data<Tracked_Obstacle> &data									// In/Out: Dynamic obstacle information
	)
{	
	int n_samples = std::round(pars.T / pars.dt);

	trajectory.resize(6, n_samples);
	trajectory.col(0) = ownship_state;

	ownship.determine_active_waypoint_segment(waypoints, ownship_state);

	int n_obst = data.obstacles.size();
	int n_static_obst = static_obstacles.cols();

	Eigen::VectorXd opt_offset_sequence_e(2 * pars.n_M);
	// Predict nominal trajectory first, assign as optimal if no need for
	// COLAV, or use in the prediction initialization
	for (int M = 0; M < pars.n_M; M++)
	{
		opt_offset_sequence_e(2 * M) = 1.0; opt_offset_sequence_e(2 * M + 1) = 0.0;
	}
	maneuver_times.setZero();
	ownship.predict_trajectory(trajectory, opt_offset_sequence_e, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

	bool colav_active = determine_colav_active(data, n_static_obst);
	if (!colav_active)
	{
		u_opt = 1.0; 		u_opt_last = u_opt;
		chi_opt = 0.0; 		chi_opt_last = chi_opt;

		assign_optimal_trajectory(predicted_trajectory);

		return;
	}
	
	initialize_prediction(data, static_obstacles);

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
 	mxArray *traj_os = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

	double *ptraj_os = mxGetPr(traj_os); 
	double *p_wps_os = mxGetPr(wps_os); 

	Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
	map_wps = waypoints;

	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, n_static_obst, mxREAL);
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 
	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, n_static_obst);
	map_static_obst = static_obstacles;

	mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_obst_mx, *i_mx, *ps_mx, *n_static_obst_mx;
	dt_sim = mxCreateDoubleScalar(pars.dt);
	T_sim = mxCreateDoubleScalar(pars.T);
	n_ps_mx = mxCreateDoubleScalar(n_ps[0]);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(n_static_obst);

	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_ps", n_ps_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "dt_sim", dt_sim);
	engPutVariable(ep, "T_sim", T_sim);
	engPutVariable(ep, "WPs", wps_os);
	engEvalString(ep, "inside_psbmpc_init_plot");

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
	} */
	
	//===============================================================================================================

	//===============================================================================================================
	// Cost evaluation
	//===============================================================================================================
	set_up_temporary_device_memory(u_d, chi_d, waypoints, static_obstacles, data);
    
	Eigen::VectorXd HL_0(n_obst); HL_0.setZero();
	
	// Allocate device vector for computing CB costs
	thrust::device_vector<float> cb_costs(pars.n_cbs);

	// Allocate iterator for passing the index of the control behavior to the kernels
	thrust::device_vector<unsigned int> cb_index_dvec(pars.n_cbs);
	thrust::sequence(cb_index_dvec.begin(), cb_index_dvec.end(), 0);

	auto cb_tuple_begin = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.begin(), control_behavior_dvec.begin()));
    auto cb_tuple_end = thrust::make_zip_iterator(thrust::make_tuple(cb_index_dvec.end(), control_behavior_dvec.end()));

	// Perform the calculations on the GPU
	cb_cost_functor.reset(new CB_Cost_Functor(
		pars_device_ptr, 
		fdata_device_ptr, 
		obstacles_device_ptr, 
		pobstacles_device_ptr,
		cpe_device_ptr, 
		ownship_device_ptr,
		trajectory_device_ptr, 
		obstacle_ship_device_ptr,
		obstacle_sbmpc_device_ptr,
		mpc_cost_device_ptr));
    thrust::transform(cb_tuple_begin, cb_tuple_end, cb_costs.begin(), *cb_cost_functor);

	// Extract minimum cost
	thrust::device_vector<float>::iterator min_cost_iter = thrust::min_element(cb_costs.begin(), cb_costs.end());
	min_index = min_cost_iter - cb_costs.begin();
	min_cost = cb_costs[min_index];

	// Assign optimal offset sequence/control behaviour
	TML::PDMatrix<float, 2 * MAX_N_M, 1> opt_offset_sequence = control_behavior_dvec[min_index];
	TML::assign_tml_object(opt_offset_sequence_e, opt_offset_sequence);

	// Set the trajectory to the optimal one and assign to the output trajectory
	ownship.predict_trajectory(trajectory, opt_offset_sequence_e, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
	assign_optimal_trajectory(predicted_trajectory);

	clear_temporary_device_memory();
	//===============================================================================================================

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Eigen::Map<Eigen::MatrixXd> map_traj(ptraj_os, 6, n_samples);
	map_traj = trajectory;

	k_s = mxCreateDoubleScalar(n_samples);
	engPutVariable(ep, "k", k_s);

	engPutVariable(ep, "X", traj_os);
	engEvalString(ep, "inside_psbmpc_upd_ownship_plot");  

	engClose(ep);*/
	//===============================================================================================================

	u_opt = opt_offset_sequence_e(0); 		u_opt_last = u_opt;
	chi_opt = opt_offset_sequence_e(1); 	chi_opt_last = chi_opt;

	std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence_e(2 * M) << ", " << opt_offset_sequence_e(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl;
}

/****************************************************************************************
	Private functions
****************************************************************************************/
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

	control_behavior_dvec.resize(pars.n_cbs);
	for (int cb = 0; cb < pars.n_cbs; cb++)
	{
		TML::assign_eigen_object(tml_offset_sequence, offset_sequence);

		control_behavior_dvec[cb] = tml_offset_sequence;

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
*  Name     : initialize_prediction
*  Function : Sets up the own-ship maneuvering times and number of prediction scenarios 
*			  for each obstacle based on the current situation, and predicts
*			  independent obstacle trajectories.
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::initialize_prediction(
	Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const Eigen::Matrix<double, 4, -1> &static_obstacles			// In: Static obstacle information
	)
{
	int n_obst = data.obstacles.size();
	n_ps.resize(n_obst);

	// pobstacles with the ownship as the last element, for the joint prediction
	pobstacles.resize(n_obst + 1);
	int n_a(0);
	if (n_obst > 0)
	{
		n_a = data.obstacles[0].get_intention_probabilities().size();
	} 

	// Add the ownship as the last prediction obstacle
	TML::PDMatrix<float, 9, 1>  xs_aug(9, 1); TML::Vector2f v_os_0;
	xs_aug(0) = trajectory(0, 0); xs_aug(1) = trajectory(1, 0);
	v_os_0(0) = trajectory(3, 0); v_os_0(1) = trajectory(4, 0);
	v_os_0 = rotate_vector_2D(v_os_0, trajectory(2, 0));
	xs_aug(2) = v_os_0(0); xs_aug(3) = v_os_0(1);
	xs_aug(4) = ownship.get_length() / 2; 
	xs_aug(5) = ownship.get_length() / 2; 
	xs_aug(6) = ownship.get_width() / 2; 
	xs_aug(7) = ownship.get_width() / 2; 
	xs_aug(8) = n_obst;

	pobstacles[n_obst] = Prediction_Obstacle(xs_aug, pars.T, pars.dt);
	//***********************************************************************************
	// Obstacle prediction initialization
	//***********************************************************************************
	std::vector<Intention> ps_ordering_i;
	Eigen::VectorXd ps_course_changes_i;
	Eigen::VectorXd ps_maneuver_times_i;

	Eigen::VectorXd t_cpa(n_obst), d_cpa(n_obst);
	Eigen::Vector2d p_cpa, d_AB, v_0;
	Eigen::Vector4d xs_i_0;
	TML::PDMatrix<double, 2, MAX_N_WPS> waypoints_i;

	// only use intelligent prediction if n_a > 1 intentions are considered
	// and obstacle colav is on
	use_joint_prediction = false; 
	for (int i = 0; i < n_obst; i++)
	{
		n_ps[i] = 1;
		
		xs_i_0 = data.obstacles[i].kf->get_state();
		/* std::cout << "xs_i_0 = " << xs_i_0.transpose() << std::endl;
		std::cout << "xs_0 = " << trajectory.col(0).transpose() << std::endl; */
		calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), trajectory.col(0), xs_i_0);
		/* std::cout << "p_cpa = " << p_cpa.transpose() << std::endl;
		std::cout << "t_cpa(i) = " << t_cpa(i) << std::endl;
		std::cout << "d_cpa(i) = " << d_cpa(i)<< std::endl; */
		if (n_a == 1 || data.IP_0[i])
		{
			ps_ordering_i.resize(1);
			ps_ordering_i[0] = KCC;			
			ps_course_changes_i.resize(1);
			ps_course_changes_i[0] = 0;
			ps_maneuver_times_i.resize(1);
			ps_maneuver_times_i(0) = 0;
		}
		else
		{
			set_up_independent_obstacle_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i, t_cpa(i), data, i);

			pobstacles[i] = Prediction_Obstacle(data.obstacles[i]);
			if (pars.obstacle_colav_on)
			{
				use_joint_prediction = true;

				// Set obstacle waypoints to a straight line out from its current time position 
				// if no future obstacle trajectory is available
				waypoints_i.resize(2, 2);
				TML::Vector4d xs_i_0_temp = pobstacles[i].get_initial_state();
				waypoints_i.set_col(0,xs_i_0_temp.get_block<2, 1>(0, 0));
				waypoints_i.set_col(1, waypoints_i.get_col(0) + xs_i_0_temp.get_block<2, 1>(2, 0) * pars.T);
				pobstacles[i].set_waypoints(waypoints_i);
				std::cout << waypoints_i << std::endl;
				n_ps[i] += 1;
			}

			
		}
		data.obstacles[i].initialize_independent_prediction(ps_ordering_i, ps_course_changes_i, ps_maneuver_times_i);	

		data.obstacles[i].predict_independent_trajectories(pars.T, pars.dt, trajectory.col(0), *this);
	}

	if (use_joint_prediction)
	{
		predict_trajectories_jointly(data, static_obstacles);
	}
	
	prune_obstacle_scenarios(data);

	//***********************************************************************************
	// Own-ship prediction initialization
	//***********************************************************************************
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
			d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());
			// For the current avoidance maneuver, determine which obstacle that should be
			// considered, i.e. the closest obstacle that is not already passed (which means
			// that the previous avoidance maneuver happened before CPA with this obstacle)
			if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min)
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
*  Name     : set_up_independent_obstacle_prediction_variables
*  Function : 
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::set_up_independent_obstacle_prediction(
	std::vector<Intention> &ps_ordering_i,									// In/out: Intention ordering of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_course_changes_i, 									// In/out: Course changes of the independent obstacle prediction scenarios
	Eigen::VectorXd &ps_maneuver_times_i, 									// In/out: Time of maneuvering for the independent obstacle prediction scenarios
	const double t_cpa_i, 													// In: Time to Closest Point of Approach for obstacle i wrt own-ship
	const Obstacle_Data<Tracked_Obstacle> &data,							// In: Dynamic obstacle information
	const int i 															// In: Index of obstacle in consideration
	)
{
	int turn_count(0), turn_start(0), n_turns(0), course_change_count(0);

	// The out-commented stuff is too ad-hoc to be used.
	/*
	double d_AB, d_AB_prev;
	Eigen::Vector4d xs_i_0 = data.obstacles[i].kf->get_state();
	Eigen::Vector2d v_0, v_i_0;

	d_AB = (trajectory.col(0).block<2, 1>(0, 0) - xs_i_0.block<2, 1>(0, 0)).norm();
	v_0 = rotate_vector_2D(trajectory.col(0).block<2, 1>(3, 0), trajectory(2, 0));
	v_i_0 = xs_i_0.block<2, 1>(2, 0);

	
	 // Alternative obstacle maneuvers (other than the straight line prediction) 
	// are only allowed inside the COLREGS consideration zone, i.e. inside d_close
	while (d_AB > pars.d_close)
	{
		d_AB_prev = d_AB;
		turn_start += 1;
		// calculate new distance between own-ship and obstacle i, given that both keep
		// their course for k * t_ts seconds, k = 1, 2, 3, ...
		d_AB = ((trajectory.col(0).block<2, 1>(0, 0) + v_0 * turn_start * pars.t_ts) - 
				(xs_i_0.block<2, 1>(0, 0) + v_i_0 * turn_start * pars.t_ts)).norm();
		if (d_AB > d_AB_prev)
		{
			turn_start = -1;
			break;
		}
	} */
	if (turn_start >= 0) // and alternative maneuvers are only up until cpa with the own-ship
	{
		n_turns = std::ceil((t_cpa_i - turn_start * pars.t_ts) / pars.t_ts);
	}
	else 							// or no alternative maneuvers at all if the obstacle never enters
	{ 								// the own-ship COLREGS consideration zone
		n_turns = 0;
	}
	n_ps[i] = 1 + 2 * pars.obstacle_course_changes.size() * n_turns;

	ps_ordering_i.resize(n_ps[i]);
	ps_ordering_i[0] = KCC;
	ps_maneuver_times_i.resize(n_ps[i]);
	ps_maneuver_times_i[0] = 0;
	ps_course_changes_i.resize(n_ps[i]);
	ps_course_changes_i[0] = 0;
	
	for (int ps = 1; ps < n_ps[i]; ps++)
	{
		// Starboard maneuvers
		if (ps < (n_ps[i] - 1) / 2 + 1)
		{
			ps_ordering_i[ps] = SM;

			ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}
		// Port maneuvers
		else
		{
			ps_ordering_i[ps] = PM;

			ps_maneuver_times_i[ps] = (turn_start + turn_count) * std::floor(pars.t_ts / pars.dt);

			ps_course_changes_i(ps) = - pars.obstacle_course_changes(course_change_count);
			if (++course_change_count == pars.obstacle_course_changes.size())
			{
				if(++turn_count == n_turns) turn_count = 0;
				course_change_count = 0;
			} 
		}	
	}
	std::cout << "Obstacle PS course changes : " << ps_course_changes_i.transpose() << std::endl;
	std::cout << "Obstacle PS maneuver times : " << ps_maneuver_times_i.transpose() << std::endl;
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
		
		calculate_instantaneous_collision_probabilities(P_c_i, data, i, dt_r, p_step);

		calculate_ps_collision_probabilities(P_c_i_ps, P_c_i, i);

		calculate_ps_collision_consequences(C_i, data, i, dt_r, p_step);

		calculate_ps_collision_risks(R_c_i, risk_sorted_ps_indices_i, C_i, P_c_i_ps, data, i);

		std::cout << risk_sorted_ps_indices_i.transpose() << std::endl;

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
		std::cout << kept_ps_indices_i.transpose() << std::endl;

		// For n_a > 1: Joint prediction/intelligent scenario is the last one in the original set
		if (use_joint_prediction && (kept_ps_indices_i(kept_ps_indices_i.size() - 1) == n_ps[i] - 1))
		{
			relevant_joint_pred_scenarios_count += 1;
			// The intelligent prediction scenario is not added to the tracked obstacle
			// data structure in this GPU version, 
			// so it should not be considered in the Tracked Obstacle
			kept_ps_indices_i.conservativeResize(kept_ps_indices_i.size() - 1);
		}

		n_ps[i] = n_ps_new;
		data.obstacles[i].prune_ps(kept_ps_indices_i);
	}

	// For n_a > 1: All intelligent obstacle scenarios are in this case pruned away
	if (!relevant_joint_pred_scenarios_count) 
	{
		use_joint_prediction = false;
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
void PSBMPC::calculate_instantaneous_collision_probabilities(
	Eigen::MatrixXd &P_c_i,								// In/out: Predicted obstacle collision probabilities for all prediction scenarios, n_ps[i] x n_samples
	const Obstacle_Data<Tracked_Obstacle> &data,		// In: Dynamic obstacle information
	const int i, 										// In: Index of obstacle
	const double dt, 									// In: Sample time for estimation
	const int p_step                                    // In: Step between trajectory samples, matches the input prediction time step
	)
{
	Eigen::MatrixXd P_i_p = data.obstacles[i].get_trajectory_covariance();
	std::vector<Eigen::MatrixXd> xs_i_p = data.obstacles[i].get_trajectories();
	Eigen::MatrixXd xs_i_colav_p;
	if (use_joint_prediction)
	{
		TML::assign_tml_object(xs_i_colav_p, pobstacles[i].get_trajectory());
	}

	// Increase safety zone by half the max obstacle dimension and ownship length
	double d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + data.obstacles[i].get_length());

	int n_samples = P_i_p.cols();
	// Non-optimal temporary row-vector storage solution
	Eigen::Matrix<double, 1, -1> P_c_i_row(n_samples);
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		if (ps == n_ps[i] - 1 && use_joint_prediction)
		{
			cpe_host.estimate_over_trajectories(P_c_i_row, trajectory, xs_i_colav_p, P_i_p, d_safe_i, dt, p_step);
		}
		else
		{
			cpe_host.estimate_over_trajectories(P_c_i_row, trajectory, xs_i_p[ps], P_i_p, d_safe_i, dt, p_step);
		}
		
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
	Eigen::MatrixXd xs_i_colav_p;
	if (use_joint_prediction)
	{
		TML::assign_tml_object(xs_i_colav_p, pobstacles[i].get_trajectory());
	}

	Eigen::Vector2d p_cpa, v_0_p, v_i_p;

	int n_samples = xs_i_p[0].cols();

	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		for (int k = 0; k < n_samples; k += p_step)
		{
			t = k * dt;

			v_0_p = trajectory.block<2, 1>(3, k);
			v_0_p = rotate_vector_2D(v_0_p, trajectory(2, k));

			if (ps == n_ps[i] - 1 && use_joint_prediction) // Intelligent prediction is the last prediction scenario
			{
				v_i_p = xs_i_colav_p.block<2, 1>(2, k);

				calculate_cpa(p_cpa, t_cpa, d_cpa, trajectory.col(k), xs_i_colav_p.col(k));
			}
			else
			{
				v_i_p = xs_i_p[ps].block<2, 1>(2, k);

				calculate_cpa(p_cpa, t_cpa, d_cpa, trajectory.col(k), xs_i_p[ps].col(k));
			}			

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
	Eigen::VectorXd Pr_a_i = data.obstacles[i].get_intention_probabilities();
	Eigen::VectorXi ps_intention_count_i = data.obstacles[i].get_ps_intention_count();
	int n_a = Pr_a_i.size();

	// The conditional probability of collision with obstacle i is predicted
	// as the product of the conditional probability Pr(ps | a, I) of prediction scenario ps 
	// (here taken as a uniform distribution) weighted by intention probabilities Pr(a | I), and the
	// predicted scenario collision probability Pr(collision | i, s)
	for (int a = 0; a < n_a; a++)
	{
		// Conditional probability of a prediction scenario is taken from a discrete uniform distribution
		// over the number of prediction scenarios which corresponds to intention a, and is
		// weighted by the corresponding intention probability
		if (ps_intention_count_i(a))
		{
			Pr_ps_i_conditional += Pr_a_i(a) / (double)ps_intention_count_i(a);
		}
	}
	
	for (int ps = 0; ps < n_ps[i]; ps++)
	{
		ps_indices_i[ps] = ps;

		Pr_c_i_conditional(ps) = P_c_i_ps(ps) * Pr_ps_i_conditional;

		R_c_i(ps) = C_i(ps) * Pr_c_i_conditional(ps);
		if (ps == n_ps[i] - 1 && use_joint_prediction)
		{
			R_c_i(ps) = 1e12;
		}
	}

	// Sort vector of ps indices to determine collision risk in sorted order
	std::sort(ps_indices_i.data(), ps_indices_i.data() + n_ps[i], [&](const int index_lhs, const int index_rhs) { return R_c_i(index_lhs) > R_c_i(index_rhs); });
	
	 std::ios::fmtflags old_settings = std::cout.flags();
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
	std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
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
*  Name     : update_conditional_obstacle_data
*  Function : Updates the situation type for the calling obstacle (wrt all other 
*			   obstacles) and obstacles (wrt own-ship) and the transitional cost 
*			   indicators O, Q, X, S at the current time t0 wrt all obstacles.
*  Author   : Trym Tengesdal
*  Modified :
****************************************************************************************/
void PSBMPC::update_conditional_obstacle_data(
	Obstacle_Data_GPU_Friendly &data, 												// In: Joint prediction information on other obstacles (including the own-ship) than obst i_caller
	const int i_caller, 															// In: Index of obstacle asking for a situational awareness update
	const int k																		// In: Index of the current predicted time t_k
	)
{
	// A : Obstacle i_caller, B : Obstacle i
	TML::Vector2f p_A, v_A, p_B, v_B, L_AB;
	float psi_A, psi_B, d_AB;

	p_A(0) = pobstacles[i_caller].get_trajectory_sample(k)(0);
	p_A(1) = pobstacles[i_caller].get_trajectory_sample(k)(1);
	v_A(0) = pobstacles[i_caller].get_trajectory_sample(k)(2);
	v_A(1) = pobstacles[i_caller].get_trajectory_sample(k)(3);
	psi_A = atan2(v_A(1), v_A(0));

	int n_obst = n_ps.size(), i_count = 0;
	data.ST_0.resize(n_obst, 1);   data.ST_i_0.resize(n_obst, 1);
	
	data.AH_0.resize(n_obst, 1);   data.S_TC_0.resize(n_obst, 1); data.S_i_TC_0.resize(n_obst, 1); 
	data.O_TC_0.resize(n_obst, 1); data.Q_TC_0.resize(n_obst, 1); data.IP_0.resize(n_obst, 1); 
	data.H_TC_0.resize(n_obst, 1); data.X_TC_0.resize(n_obst, 1);

	bool is_close(false);
	for (int i = 0; i < n_obst + 1; i++)
	{
		if (i != i_caller)
		{
			p_B(0) = pobstacles[i].get_trajectory_sample(k)(0);
			p_B(1) = pobstacles[i].get_trajectory_sample(k)(1);
			v_B(0) = pobstacles[i].get_trajectory_sample(k)(2);
			v_B(1) = pobstacles[i].get_trajectory_sample(k)(3);
			psi_B = atan2(v_B(1), v_B(0));

			L_AB = p_B - p_A;
			d_AB = L_AB.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_AB = d_AB - 0.5 * (pobstacles[i_caller].get_length() + pobstacles[i].get_length()); 				
			L_AB = L_AB.normalized();

			determine_situation_type(data.ST_0[i_count], data.ST_i_0[i_count], v_A, psi_A, v_B, L_AB, d_AB);
			
			//std::cout << "Obstacle i = " << i << " situation type wrt obst j = " << j << " ? " << data[i].ST_0[j] << std::endl;
			//std::cout << "Obst j = " << j << " situation type wrt obstacle i = " << i << " ? " << data[i].ST_i_0[j] << std::endl;

			//=====================================================================
			// Transitional variable update
			//=====================================================================
			is_close = d_AB <= pars.d_close;

			data.AH_0[i_count] = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

			//std::cout << "Obst j = " << j << " ahead at t0 ? " << data[i].AH_0[j] << std::endl;
			
			// Obstacle on starboard side
			data.S_TC_0[i_count] = angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << data[i].S_TC_0[j] << std::endl;

			// Ownship on starboard side of obstacle
			data.S_i_TC_0[i_count] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			//std::cout << "Obstacle i = " << i << " on starboard side of obstacle j = " << j << " at t0 ? " << data[i].S_i_TC_0[j] << std::endl;

			// Ownship overtaking the obstacle
			data.O_TC_0[i_count] = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() 	&&
					v_B.norm() < v_A.norm()							    						&&
					v_B.norm() > 0.25															&&
					is_close 																	&&
					data.AH_0[i_count];

			//std::cout << "Own-ship overtaking obst j = " << j << " at t0 ? " << data[i].O_TC_0[j] << std::endl;

			// Obstacle overtaking the ownship
			data.Q_TC_0[i_count] = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() < v_B.norm()							  							&&
					v_A.norm() > 0.25 															&&
					is_close 																	&&
					!data.AH_0[i_count];

			//std::cout << "Obst j = " << j << " overtaking obstacle i = " << i << " at t0 ? " << data[i].Q_TC_0[j] << std::endl;

			// Determine if the obstacle is passed by
			data.IP_0[i_count] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm()		&& // Ownship's perspective	
					!data.Q_TC_0[i_count])		 											||
					(v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() 					&& // Obstacle's perspective	
					!data.O_TC_0[i_count]))		 											&&
					d_AB > pars.d_safe;
			
			//std::cout << "Obst j = " << j << " passed by at t0 ? " << data[i].IP_0[j] << std::endl;

			// This is not mentioned in article, but also implemented here..				
			data.H_TC_0[i_count] = v_A.dot(v_B) < - cos(pars.phi_HO) * v_A.norm() * v_B.norm() 	&&
					v_A.norm() > 0.25																&&
					v_B.norm() > 0.25																&&
					data.AH_0[i_count];
			
			//std::cout << "Head-on at t0 wrt obst j = " << j << " ? " << data[i].H_TC_0[j] << std::endl;

			// Crossing situation, a bit redundant with the !is_passed condition also, 
			// but better safe than sorry (could be replaced with B_is_ahead also)
			data.X_TC_0[i_count] = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm()		&&
					!data.H_TC_0[i_count]															&& 
					!data.IP_0[i_count]																&&
					v_A.norm() > 0.25																&&
					v_B.norm() > 0.25;

			//std::cout << "Crossing at t0 wrt obst j = " << j << " ? " << data[i].X_TC_0[j] << std::endl;

			i_count += 1;
		}
	}	
}

/****************************************************************************************
*  Name     : predict_trajectories_jointly
*  Function : Predicts the trajectory of the obstacles with an active COLAV system,  
*		      considering the fixed current control behaviour, and adds or overwrites
*			  this information to the obstacle data structure
*  Author   : Trym Tengesdal
*  Modified :
*****************************************************************************************/
void PSBMPC::predict_trajectories_jointly(
	Obstacle_Data<Tracked_Obstacle> &data,						// In: Dynamic obstacle information
	const Eigen::Matrix<double, 4, -1>& static_obstacles		// In: Static obstacle information
	)
{
	int n_samples = trajectory.cols();
	int n_obst = data.obstacles.size();
	
	Obstacle_Data_GPU_Friendly data_jp;
	Obstacle_SBMPC obstacle_sbmpc;
	Obstacle_Ship obstacle_ship;

	std::vector<Eigen::Matrix<double, 4, -1>> predicted_trajectory_i(n_obst);
	Eigen::MatrixXd traj_i;
	TML::PDMatrix<float, 4, MAX_N_SAMPLES> pred_traj_i;

	TML::PDMatrix<float, MAX_N_OBST, 1> u_opt_last_i(n_obst), chi_opt_last_i(n_obst), u_d_i(n_obst), chi_d_i(n_obst);
	TML::Vector4f xs_i_p, xs_i_p_transformed;
	
	TML::PDMatrix<float, 4, MAX_N_OBST> static_obstacles_jp;
	TML::assign_eigen_object(static_obstacles_jp, static_obstacles);

	Eigen::Matrix<double, 2, -1> waypoints_i;
	TML::Vector4f xs_i_0;
 	for(int i = 0; i < n_obst; i++)
	{
		xs_i_0 = pobstacles[i].get_initial_state();
		u_d_i(i) = xs_i_0.get_block<2, 1>(2, 0).norm();
		chi_d_i(i) = atan2(xs_i_0(3), xs_i_0(2));

		u_opt_last_i(i) = 1.0f; chi_opt_last_i(i) = 0.0f;

		pobstacles[i].set_intention(KCC);
	}

	//===============================================================================================================
	// MATLAB PLOTTING FOR DEBUGGING
	//===============================================================================================================
	/* Engine *ep = engOpen(NULL);
	if (ep == NULL)
	{
		std::cout << "engine start failed!" << std::endl;
	}
	char buffer[BUFFSIZE+1]; 

 	mxArray *traj_os_mx = mxCreateDoubleMatrix(6, n_samples, mxREAL);
	mxArray *static_obst_mx = mxCreateDoubleMatrix(4, static_obstacles.cols(), mxREAL);
	mxArray *dt_sim_mx(nullptr), *T_sim_mx(nullptr), *k_s_mx(nullptr), *d_safe_mx(nullptr), *n_obst_mx(nullptr), *i_mx(nullptr), *n_static_obst_mx(nullptr);

	std::vector<mxArray*> wps_i_mx(n_obst);
	std::vector<mxArray*> traj_i_mx(n_obst);
	std::vector<mxArray*> pred_traj_i_mx(n_obst);

	double *p_traj_os = mxGetPr(traj_os_mx); 
	double *p_static_obst_mx = mxGetPr(static_obst_mx); 
	double *p_wps_i = nullptr; 
	double *p_traj_i = nullptr;
	double *p_pred_traj_i = nullptr;

	int n_wps_i = 2;
	Eigen::Map<Eigen::MatrixXd> map_traj_os(p_traj_os, 6, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_wps_i(p_wps_i, 2, n_wps_i);
	Eigen::Map<Eigen::MatrixXd> map_traj_i(p_traj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_pred_traj_i(p_pred_traj_i, 4, n_samples);
	Eigen::Map<Eigen::MatrixXd> map_static_obst(p_static_obst_mx, 4, static_obstacles.cols());

	map_traj_os = trajectory;
	map_static_obst = static_obstacles;

	dt_sim_mx = mxCreateDoubleScalar(pars.dt);
	T_sim_mx = mxCreateDoubleScalar(pars.T);
	d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
	n_obst_mx = mxCreateDoubleScalar(n_obst);
	n_static_obst_mx = mxCreateDoubleScalar(static_obstacles.cols());

	engPutVariable(ep, "X", traj_os_mx);
	engPutVariable(ep, "X_static", static_obst_mx);
	engPutVariable(ep, "n_static_obst", n_static_obst_mx);
	engPutVariable(ep, "n_obst", n_obst_mx);
	engPutVariable(ep, "d_safe", d_safe_mx);
	engPutVariable(ep, "dt_sim", dt_sim_mx);
	engPutVariable(ep, "T_sim", T_sim_mx);
	engEvalString(ep, "joint_pred_init_plotting");

 	for(int i = 0; i < n_obst; i++)
	{
		TML::assign_tml_object(waypoints_i, pobstacles[i].get_waypoints());
		wps_i_mx[i] = mxCreateDoubleMatrix(2, 2, mxREAL);
		traj_i_mx[i] = mxCreateDoubleMatrix(4, n_samples, mxREAL);

		p_wps_i = mxGetPr(wps_i_mx[i]);
		new (&map_wps_i) Eigen::Map<Eigen::MatrixXd>(p_wps_i, 2, n_wps_i);
		map_wps_i = waypoints_i;

		engPutVariable(ep, "WPs_i", wps_i_mx[i]);
		engEvalString(ep, "joint_pred_init_obstacle_plot");
	}	 */
	//===============================================================================================================
	auto start = std::chrono::system_clock::now(), end = start;
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//double mean_t(0.0);

	double t(0.0), u_c_i(0.0), chi_c_i(0.0), chi_i(0.0);
	float chi_opt_i(0.0), u_opt_i(0.0);
	TML::Vector2f v_os_k, p_os_k;
	TML::Vector4f xs_os_k;
	for (int k = 0; k < n_samples; k++)
	{	
		t = k * pars.dt;

		v_os_k(0) = trajectory(3, k);
		v_os_k(1) = trajectory(4, k);
		v_os_k = rotate_vector_2D(v_os_k, trajectory(2, k));
		xs_os_k(0) = trajectory(0, k);
		xs_os_k(1) = trajectory(1, k);
		xs_os_k(2) = v_os_k(0);
		xs_os_k(3) = v_os_k(1);

		pobstacles[n_obst].set_trajectory_sample(xs_os_k, k);
		//std::cout << "xs_os_aug_k = " << xs_os_aug_k.transpose() << std::endl;
	
		for (int i = 0; i < n_obst; i++)
		{
			// Update Obstacle Data for each prediction obstacle, taking all other obstacles
			// including the ownship into account
			update_conditional_obstacle_data(data_jp, i, k);

			xs_i_p = pobstacles[i].get_trajectory_sample(k);

			//std::cout << "xs_i_p = " << xs_i_p.transpose() << std::endl;

			// Convert from X_i = [x, y, Vx, Vy] to X_i = [x, y, chi, U]
			xs_i_p_transformed(0) = xs_i_p(0);
			xs_i_p_transformed(1) = xs_i_p(1);
			xs_i_p_transformed(2) = atan2(xs_i_p(3), xs_i_p(2));
			xs_i_p_transformed(3) = xs_i_p.get_block<2, 1>(2, 0).norm();

			// Determine the intention that obstacle i`s predicted trajectory
			// corresponds to
			chi_i = xs_i_p_transformed(2);
			if (t < 30)
			{
				if (chi_i > 15 * DEG2RAD)								{ pobstacles[i].set_intention(SM); }
				else if (chi_i < -15 * DEG2RAD)							{ pobstacles[i].set_intention(PM); }
			}

			obstacle_ship.update_guidance_references(
				u_d_i(i), 
				chi_d_i(i), 
				pobstacles[i].get_waypoints(),
				xs_i_p,
				pars.dt,
				pars.guidance_method);

			if (fmod(t, 5) == 0)
			{
				start = std::chrono::system_clock::now();	

				obstacle_sbmpc.calculate_optimal_offsets(
					u_opt_i, 
					chi_opt_i, 
					pred_traj_i,
					u_opt_last_i(i),
					chi_opt_last_i(i),
					u_d_i(i), 
					chi_d_i(i),
					pobstacles[i].get_waypoints(),
					xs_i_p_transformed,
					static_obstacles_jp,
					data_jp,
					pobstacles.data(),
					i,
					k);

				end = std::chrono::system_clock::now();
				elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

				/* mean_t = elapsed.count();
				std::cout << "Obstacle_SBMPC time usage : " << mean_t << " milliseconds" << std::endl; */
				u_opt_last_i(i) = u_opt_i; chi_opt_last_i(i) = chi_opt_i;
			}
			u_c_i = u_d_i(i) * u_opt_i; chi_c_i = chi_d_i(i) + chi_opt_i;

			if (k < n_samples - 1)
			{
				xs_i_p_transformed = obstacle_ship.predict(xs_i_p_transformed, u_c_i, chi_c_i, pars.dt, pars.prediction_method);
				
				// Convert from X_i = [x, y, chi, U] to X_i = [x, y, Vx, Vy]
				xs_i_p(0) = xs_i_p_transformed(0);
				xs_i_p(1) = xs_i_p_transformed(1);
				xs_i_p(2) = xs_i_p_transformed(3) * cos(xs_i_p_transformed(2));
				xs_i_p(3) = xs_i_p_transformed(3) * sin(xs_i_p_transformed(2));

				pobstacles[i].set_trajectory_sample(xs_i_p, k + 1);
			}
			
		//============================================================================================
		// Send data to matlab for live plotting
		//============================================================================================
		/* 	
			k_s_mx = mxCreateDoubleScalar(k + 1);
			engPutVariable(ep, "k", k_s_mx);

			buffer[BUFFSIZE] = '\0';
			engOutputBuffer(ep, buffer, BUFFSIZE);

			TML::assign_tml_object(traj_i, pobstacles[i].get_trajectory());
			TML::assign_tml_object(predicted_trajectory_i[i], pred_traj_i);
			
			pred_traj_i_mx[i] = mxCreateDoubleMatrix(4, predicted_trajectory_i[i].cols(), mxREAL);

			p_traj_i = mxGetPr(traj_i_mx[i]);
			p_pred_traj_i = mxGetPr(pred_traj_i_mx[i]);

			new (&map_traj_i) Eigen::Map<Eigen::MatrixXd>(p_traj_i, 4, n_samples);
			new (&map_pred_traj_i) Eigen::Map<Eigen::MatrixXd>(p_pred_traj_i, 4, predicted_trajectory_i[i].cols());
			
			
			map_traj_i = traj_i;
			map_pred_traj_i = predicted_trajectory_i[i];

			i_mx = mxCreateDoubleScalar(i + 1);

			engPutVariable(ep, "i", i_mx);
			engPutVariable(ep, "X_i", traj_i_mx[i]);			
			engPutVariable(ep, "X_i_pred", pred_traj_i_mx[i]);

			engEvalString(ep, "update_joint_pred_obstacle_plot");

			printf("%s", buffer);	 */
			//============================================================================================				
			
		}
		

		/* engEvalString(ep, "update_joint_pred_ownship_plot"); */
		//============================================================================================
	}

	//============================================================================================
	// Clean up matlab arrays
	//============================================================================================
	/* mxDestroyArray(traj_os_mx);
	mxDestroyArray(static_obst_mx);
	mxDestroyArray(T_sim_mx);
	mxDestroyArray(dt_sim_mx);
	mxDestroyArray(i_mx);
	mxDestroyArray(d_safe_mx);
	mxDestroyArray(k_s_mx);

	for (int i = 0; i < n_obst; i++)
	{
		mxDestroyArray(wps_i_mx[i]);
		mxDestroyArray(traj_i_mx[i]);
		mxDestroyArray(pred_traj_i_mx[i]);
	}
	engClose(ep); */
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
	Eigen::Matrix<double, 6, 1> xs = trajectory.col(0);
	bool colav_active = false;
	Eigen::Vector2d d_0i;
	for (size_t i = 0; i < data.obstacles.size(); i++)
	{
		d_0i(0) = data.obstacles[i].kf->get_state()(0) - xs(0);
		d_0i(1) = data.obstacles[i].kf->get_state()(1) - xs(1);
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
	if (pars.prediction_method > Linear)
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
	const Eigen::Matrix<double, 4, -1> &static_obstacles,			// In: Static obstacle information
	const Obstacle_Data<Tracked_Obstacle> &data 					// In: Dynamic obstacle information
	)
{
	int n_obst = data.obstacles.size();

	/* std::cout << "CB_Functor_Pars size: " << sizeof(CB_Functor_Pars) << std::endl;
	std::cout << "CB_Functor_Data size: " << sizeof(CB_Functor_Data) << std::endl;
	std::cout << "Ownship size: " << sizeof(Ownship) << std::endl;
	std::cout << "CPE size: " << sizeof(CPE) << std::endl;
	std::cout << "Cuda Obstacle size: " << sizeof(Cuda_Obstacle) << std::endl; 
	std::cout << "Prediction Obstacle size: " << sizeof(Cuda_Obstacle) << std::endl; */
	
	size_t limit = 0;

	cudaDeviceSetLimit(cudaLimitStackSize, 100000);
	cuda_check_errors("Setting cudaLimitStackSize failed.");

	cudaDeviceGetLimit(&limit, cudaLimitStackSize);
	cuda_check_errors("Reading cudaLimitStackSize failed.");
	std::cout << "Set device max stack size : " << limit << std::endl;

	/* cudaDeviceGetLimit(&limit, cudaLimitMallocHeapSize);
	cuda_check_errors("Reading cudaLimitMallocHeapSize failed.");
	std::cout << "Device max heap size : " << limit << std::endl; */

	// Allocation of device memory for control behaviour functor data and obstacles
	// Both are read-only, so only need one on the device
	cudaMalloc((void**)&fdata_device_ptr, sizeof(CB_Functor_Data));
    cuda_check_errors("CudaMalloc of CB_Functor_Data failed.");

	CB_Functor_Data temporary_fdata( 
		trajectory, 
		maneuver_times, 
		u_opt_last, 
		chi_opt_last, 
		u_d, chi_d, 
		use_joint_prediction, 
		ownship.get_wp_counter(),
		waypoints, 
		static_obstacles, 
		n_ps, 
		data);
	cudaMemcpy(fdata_device_ptr, &temporary_fdata, sizeof(CB_Functor_Data), cudaMemcpyHostToDevice);
    cuda_check_errors("CudaMemCpy of CB_Functor_Data failed.");
	
	// Obstacles
	cudaMalloc((void**)&obstacles_device_ptr, n_obst * sizeof(Cuda_Obstacle));
    cuda_check_errors("CudaMalloc of Cuda_Obstacle's failed.");

	Cuda_Obstacle temp_transfer_cobstacle;
	for (int i = 0; i < n_obst; i++)
	{
		temp_transfer_cobstacle = data.obstacles[i];

		cudaMemcpy(&obstacles_device_ptr[i], &temp_transfer_cobstacle, sizeof(Cuda_Obstacle), cudaMemcpyHostToDevice);
    	cuda_check_errors("CudaMemCpy of Cuda_Obstacle i failed.");
	}

	Prediction_Obstacle temp_transfer_pobstacle;
	if (use_joint_prediction)
	{
		for (int i = 0; i < n_obst + 1; i++)
		{
			temp_transfer_pobstacle = pobstacles[i];

			cudaMemcpy(&pobstacles_device_ptr[i], &temp_transfer_pobstacle, sizeof(Prediction_Obstacle), cudaMemcpyHostToDevice);
			cuda_check_errors("CudaMemCpy of Prediction_Obstacle i failed.");
		}
	}
}

/****************************************************************************************
*  Name     : clear_temporary_device_memory
*  Function : Clears/frees CB Functor and obstacle device memory used in one iteration 
*			  in the PSB-MPC. 
*  Author   :
*  Modified :
*****************************************************************************************/
void PSBMPC::clear_temporary_device_memory()
{
	cudaFree(fdata_device_ptr); 
	cuda_check_errors("cudaFree of fdata_device_ptr failed.");
    
	cudaFree(obstacles_device_ptr);
    cuda_check_errors("cudaFree of obstacles_device_ptr failed.");
}