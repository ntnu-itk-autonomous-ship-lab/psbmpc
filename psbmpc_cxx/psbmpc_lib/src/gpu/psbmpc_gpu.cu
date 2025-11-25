#include "gpu/psbmpc_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include "cpu/utilities_cpu.hpp"
#include "gpu/cuda_obstacle.cuh"
#include "gpu/cpe_gpu.cuh"
#include "gpu/mpc_cost_gpu.cuh"
#include "gpu/cb_cost_functor.cuh"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/counting_iterator.h>

#include <iostream>
#include <iomanip>
#include <chrono>

#if ENABLE_PSBMPC_DEBUGGING
#include "engine.h"
#define BUFFSIZE 100000
#endif

namespace PSBMPC_LIB
{
	namespace GPU
	{

		PSBMPC::PSBMPC()
			: u_opt_last(1.0), chi_opt_last(0.0), min_cost(1e12), ownship(Ownship()), pars(PSBMPC_Parameters()), mpc_cost(pars),
			  trajectory_device_ptr(nullptr), pars_device_ptr(nullptr), fdata_device_ptr(nullptr), obstacles_device_ptr(nullptr),
			  cpe_device_ptr(nullptr), ownship_device_ptr(nullptr), polygons_device_ptr(nullptr), mpc_cost_device_ptr(nullptr),
			  colregs_violation_evaluators_device_ptr(nullptr)

		{
			cpe_gpu = CPE(pars.cpe_method, pars.dt);

			opt_offset_sequence.resize(2 * pars.n_M);
			maneuver_times.resize(pars.n_M);

			preallocate_device_data();
		}

		PSBMPC::PSBMPC(
			const Ownship &ownship,				  // In: Own-ship with specific parameter set
			const CPU::CPE &cpe,				  // In: CPE with specific parameter set
			const PSBMPC_Parameters &psbmpc_pars, // In: Parameter object to initialize the PSB-MPC
			const CVE_Pars<float> &cve_pars		  // In: Parameter object to initialize the COLREGS Violation Evaluators
			) : u_opt_last(1.0), chi_opt_last(0.0), min_cost(1e12), ownship(ownship), cpe_gpu(cpe), colregs_violation_evaluator_gpu(cve_pars),
				pars(psbmpc_pars), mpc_cost(pars),
				trajectory_device_ptr(nullptr), pars_device_ptr(nullptr), fdata_device_ptr(nullptr), obstacles_device_ptr(nullptr),
				cpe_device_ptr(nullptr), ownship_device_ptr(nullptr), polygons_device_ptr(nullptr), mpc_cost_device_ptr(nullptr),
				colregs_violation_evaluators_device_ptr(nullptr)
		{
			opt_offset_sequence.resize(2 * pars.n_M);
			maneuver_times.resize(pars.n_M);

			preallocate_device_data();
		}

		PSBMPC::PSBMPC(const PSBMPC &other)
			: trajectory_device_ptr(nullptr), pars_device_ptr(nullptr), fdata_device_ptr(nullptr), obstacles_device_ptr(nullptr),
			  cpe_device_ptr(nullptr), ownship_device_ptr(nullptr), polygons_device_ptr(nullptr), mpc_cost_device_ptr(nullptr),
			  colregs_violation_evaluators_device_ptr(nullptr)
		{
			assign_data(other);

			preallocate_device_data();
		}

		PSBMPC &PSBMPC::operator=(const PSBMPC &other)
		{
			if (this != &other)
			{
				free();

				assign_data(other);

				preallocate_device_data();
			}
			return *this;
		}

		PSBMPC::~PSBMPC()
		{
			free();
		};

		void PSBMPC::calculate_optimal_offsets(
			double &u_opt,								   // In/out: Optimal surge offset
			double &chi_opt,							   // In/out: Optimal course offset
			Eigen::MatrixXd &predicted_trajectory,		   // In/out: Predicted optimal ownship trajectory
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Next waypoints
			const Eigen::VectorXd &ownship_state,		   // In: Current ship state
			const double V_w,							   // In: Estimated wind speed
			const Eigen::Vector2d &wind_direction,		   // In: Unit vector in NE describing the estimated wind direction
			const Static_Obstacles &polygons,			   // In: Static obstacle information
			const Dynamic_Obstacles &obstacles,			   // In: Dynamic obstacle information
			const bool disable							   // In: Disable the COLAV functionality or not
		)
		{
			int n_samples = std::round(pars.T / pars.dt);

			trajectory.resize(ownship_state.size(), n_samples);
			trajectory.col(0) = ownship_state;

			ownship.determine_active_waypoint_segment(waypoints, ownship_state);

			int n_do = obstacles.size();
			int n_so = polygons.size();
			int n_ps(0);
			// Predict nominal trajectory first, assign as optimal if no need for
			// COLAV, or use in the prediction initialization
			for (int M = 0; M < pars.n_M; M++)
			{
				opt_offset_sequence(2 * M) = 1.0;
				opt_offset_sequence(2 * M + 1) = 0.0;
			}
			maneuver_times.setZero();
			ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

			bool colav_active = determine_colav_active(obstacles, n_so, disable);
			if (!colav_active)
			{
				u_opt = 1.0;
				u_opt_last = u_opt;
				chi_opt = 0.0;
				chi_opt_last = chi_opt;

				assign_optimal_trajectory(predicted_trajectory);

				return;
			}

			setup_prediction(obstacles);

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			Engine *ep = engOpen(NULL);
			if (ep == NULL)
			{
				std::cout << "engine start failed!" << std::endl;
			}
			char buffer[BUFFSIZE + 1];
			buffer[BUFFSIZE] = '\0';
			engOutputBuffer(ep, buffer, BUFFSIZE);
			mxArray *init_state_os_mx = mxCreateDoubleMatrix(ownship_state.size(), 1, mxREAL);
			mxArray *traj_os_mx = mxCreateDoubleMatrix(trajectory.rows(), n_samples, mxREAL);
			mxArray *wps_os = mxCreateDoubleMatrix(2, waypoints.cols(), mxREAL);

			double *p_init_state_os = mxGetPr(init_state_os_mx);
			double *p_traj_os = mxGetPr(traj_os_mx);
			double *p_wps_os = mxGetPr(wps_os);

			Eigen::Map<Eigen::VectorXd> map_init_state_os(p_init_state_os, ownship_state.size(), 1);
			Eigen::Map<Eigen::MatrixXd> map_wps(p_wps_os, 2, waypoints.cols());
			map_init_state_os = ownship_state;
			map_wps = waypoints;

			int n_ps_max(0);
			for (int i = 0; i < n_do; i++)
			{
				n_ps = obstacles[i].get_trajectories().size();
				if (n_ps_max < n_ps)
				{
					n_ps_max = n_ps;
				}
			}

			mxArray *dt_sim, *T_sim, *k_s, *n_ps_mx, *n_do_mx, *n_so_mx, *i_mx, *ps_mx, *d_safe_mx, *n_patches_mx;
			dt_sim = mxCreateDoubleScalar(pars.dt);
			T_sim = mxCreateDoubleScalar(pars.T);
			n_ps_mx = mxCreateDoubleScalar(n_ps_max);
			n_do_mx = mxCreateDoubleScalar(n_do);
			d_safe_mx = mxCreateDoubleScalar(pars.d_safe);
			n_so_mx = mxCreateDoubleScalar(n_so);
			n_patches_mx = mxCreateDoubleScalar(5);

			engPutVariable(ep, "ownship_state", init_state_os_mx);
			engPutVariable(ep, "n_ps", n_ps_mx);
			engPutVariable(ep, "n_do", n_do_mx);
			engPutVariable(ep, "n_so", n_so_mx);
			engPutVariable(ep, "dt_sim", dt_sim);
			engPutVariable(ep, "T_sim", T_sim);
			engPutVariable(ep, "WPs", wps_os);
			engPutVariable(ep, "d_safe", d_safe_mx);
			engPutVariable(ep, "n_patches", n_patches_mx);
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
			for (int j = 0; j < n_so; j++)
			{
				j_mx = mxCreateDoubleScalar(j + 1);
				n_total_vertices = 0;
				for (auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
				{
					n_total_vertices += 1;
				}
				polygon_matrix.resize(2, n_total_vertices);
				pcount = 0;
				for (auto it = boost::begin(boost::geometry::exterior_ring(polygons[j])); it != boost::end(boost::geometry::exterior_ring(polygons[j])) - 1; ++it)
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

			std::vector<mxArray *> P_c_i_mx(n_do);

			for (int i = 0; i < n_do; i++)
			{
				P_c_i_mx[i] = mxCreateDoubleMatrix(n_ps, n_samples, mxREAL);

				Eigen::MatrixXd P_i_p = obstacles[i].get_trajectory_covariance();
				std::vector<Eigen::MatrixXd> xs_i_p = obstacles[i].get_trajectories();

				i_mx = mxCreateDoubleScalar(i + 1);
				engPutVariable(ep, "i", i_mx);

				map_P_traj_i = P_i_p;
				engPutVariable(ep, "P_i_flat", P_traj_i);
				for (int ps = 0; ps < n_ps; ps++)
				{
					ps_mx = mxCreateDoubleScalar(ps + 1);
					engPutVariable(ep, "ps", ps_mx);

					map_traj_i = xs_i_p[ps];

					engPutVariable(ep, "X_i", traj_i);
					engEvalString(ep, "inside_psbmpc_obstacle_plot");
				}
			}
#endif
			//===============================================================================================================

			set_up_temporary_device_memory(u_d, chi_d, waypoints, V_w, wind_direction, polygons, obstacles);
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
			map_thrust_dvecs(obstacles, polygons);

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
				mpc_cost_device_ptr,
				colregs_violation_evaluators_device_ptr));

			auto input_tuple_2_begin = thrust::make_zip_iterator(thrust::make_tuple(
				thread_index_dvec.begin(),
				cb_dvec.begin(),
				cb_index_dvec.begin(),
				sobstacle_index_dvec.begin(),
				dobstacle_index_dvec.begin(),
				dobstacle_ps_index_dvec.begin(),
				os_do_ps_pair_index_dvec.begin()));

			auto input_tuple_2_end = thrust::make_zip_iterator(thrust::make_tuple(
				thread_index_dvec.end(),
				cb_dvec.end(),
				cb_index_dvec.end(),
				sobstacle_index_dvec.end(),
				dobstacle_index_dvec.end(),
				dobstacle_ps_index_dvec.end(),
				os_do_ps_pair_index_dvec.end()));

			thrust::transform(input_tuple_2_begin, input_tuple_2_end, cb_costs_2_dvec.begin(), *cb_cost_functor_2);
			cuda_check_errors("Thrust transform for cost calculation part three failed.");

			//===============================================================================================================
			// Stitch together the GPU calculated costs to form the cost function for each control behaviour
			// and find the optimal solution
			//===============================================================================================================
			find_optimal_control_behaviour(obstacles, polygons);

			ownship.predict_trajectory(trajectory, opt_offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);
			assign_optimal_trajectory(predicted_trajectory);
//===============================================================================================================

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			Eigen::Map<Eigen::MatrixXd> map_traj(p_traj_os, trajectory.rows(), n_samples);
			map_traj = trajectory;

			k_s = mxCreateDoubleScalar(n_samples);
			engPutVariable(ep, "k", k_s);

			engPutVariable(ep, "X", traj_os_mx);
			engEvalString(ep, "inside_psbmpc_upd_ownship_plot");

			printf("%s", buffer);

			engClose(ep);
#endif
			//====================================================================

			u_opt = opt_offset_sequence(0);
			u_opt_last = u_opt;
			chi_opt = opt_offset_sequence(1);
			chi_opt_last = chi_opt;

			std::cout << "Optimal offset sequence : ";
			for (int M = 0; M < pars.n_M; M++)
			{
				std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
				if (M < pars.n_M - 1)
					std::cout << ", ";
			}
			std::cout << std::endl;

			std::cout << "Cost at optimum : " << min_cost << std::endl;
		}

		/****************************************************************************************
			Private functions
		****************************************************************************************/

		void PSBMPC::preallocate_device_data()
		{
			/* std::cout << "CB_Functor_Pars size: " << sizeof(CB_Functor_Pars) << std::endl;
			std::cout << "CB_Functor_Data size: " << sizeof(CB_Functor_Data) << std::endl;
			std::cout << "Ownship size: " << sizeof(Ownship) << std::endl;
			std::cout << "Ownship trajectory size: " << sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>) << std::endl;
			std::cout << "CPE size: " << sizeof(CPE) << std::endl;
			std::cout << "Cuda Obstacle size: " << sizeof(Cuda_Obstacle) << std::endl;
			std::cout << "Obstacle Ship size: " << sizeof(Obstacle_Ship) << std::endl;
			std::cout << "Basic polygon size: " << sizeof(Basic_Polygon) << std::endl;
			std::cout << "COLREGS Violation Evaluator size: " << sizeof(COLREGS_Violation_Evaluator) << std::endl;
			std::cout << "MPC_Cost<CB_Functor_Pars> size: " << sizeof(MPC_Cost<CB_Functor_Pars>) << std::endl; */

			//================================================================================
			// Cuda device memory allocation, preallocated to save computation time
			//================================================================================
			// Allocate for use by all threads a read-only Cuda_Obstacle array
			cudaMalloc((void **)&obstacles_device_ptr, MAX_N_DO * sizeof(Cuda_Obstacle));
			cuda_check_errors("CudaMalloc of Cuda_Obstacle's failed.");

			// Allocate for use by all threads a control behaviour parameter object
			CB_Functor_Pars temp_pars(pars);
			cudaMalloc((void **)&pars_device_ptr, sizeof(CB_Functor_Pars));
			cuda_check_errors("CudaMalloc of CB_Functor_Pars failed.");

			cudaMemcpy(pars_device_ptr, &temp_pars, sizeof(CB_Functor_Pars), cudaMemcpyHostToDevice);
			cuda_check_errors("CudaMemCpy of CB_Functor_Pars failed.");

			// Allocate for use by all threads a control behaviour data object
			cudaMalloc((void **)&fdata_device_ptr, sizeof(CB_Functor_Data));
			cuda_check_errors("CudaMalloc of CB_Functor_Data failed.");

			// Allocate for each control behaviour an own-ship trajectory
			cudaMalloc((void **)&trajectory_device_ptr, pars.n_cbs * sizeof(TML::PDMatrix<float, 4, MAX_N_SAMPLES>));
			cuda_check_errors("CudaMalloc of trajectory failed.");

			// Allocate for each control behaviour an ownship
			cudaMalloc((void **)&ownship_device_ptr, pars.n_cbs * sizeof(Ownship));
			cuda_check_errors("CudaMalloc of Ownship failed.");

			// Allocate for each thread that considers dynamic obstacles a Collision Probability Estimator
			cudaMalloc((void **)&cpe_device_ptr, pars.n_cbs * MAX_N_DO * MAX_N_PS * sizeof(CPE));
			cuda_check_errors("CudaMalloc of CPE failed.");

			// Allocate for each thread that considers dynamic obstacles a COLREGS Violation Evaluator
			cudaMalloc((void **)&colregs_violation_evaluators_device_ptr, pars.n_cbs * MAX_N_DO * MAX_N_PS * sizeof(COLREGS_Violation_Evaluator));
			cuda_check_errors("CudaMalloc of CPE failed.");

			// Allocate for each thread an mpc cost object
			MPC_Cost<CB_Functor_Pars> temp_mpc_cost(temp_pars);
			int max_n_threads = pars.n_cbs * (MAX_N_SO + MAX_N_DO * MAX_N_PS);
			cudaMalloc((void **)&mpc_cost_device_ptr, max_n_threads * sizeof(MPC_Cost<CB_Functor_Pars>));
			cuda_check_errors("CudaMalloc of MPC_Cost failed.");

			for (int cb = 0; cb < pars.n_cbs; cb++)
			{
				cudaMemcpy(&ownship_device_ptr[cb], &ownship, sizeof(Ownship), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of Ownship failed.");
			}

			for (int thread = 0; thread < pars.n_cbs * MAX_N_DO * MAX_N_PS; thread++)
			{
				cudaMemcpy(&cpe_device_ptr[thread], &cpe_gpu, sizeof(CPE), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of CPE failed.");

				cudaMemcpy(&colregs_violation_evaluators_device_ptr[thread], &colregs_violation_evaluator_gpu, sizeof(COLREGS_Violation_Evaluator), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of COLREGS Violation Evaluator failed.");
			}

			for (int thread = 0; thread < max_n_threads; thread++)
			{
				cudaMemcpy(&mpc_cost_device_ptr[thread], &temp_mpc_cost, sizeof(MPC_Cost<CB_Functor_Pars>), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of MPC_Cost failed.");
			}

			// Allocate for use by all threads that consider static obstacles a read-only Basic_Polygon array
			cudaMalloc((void **)&polygons_device_ptr, MAX_N_SO * sizeof(Basic_Polygon));
			cuda_check_errors("CudaMalloc of Basic_Polygon`s failed.");
		}

		bool PSBMPC::determine_colav_active(
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const int n_so,						// In: Number of static obstacles
			const bool disable					// In: Disable the COLAV functionality or not
		)
		{
			if (disable)
			{
				return false;
			}
			Eigen::VectorXd xs = trajectory.col(0);
			bool colav_active = false;
			Eigen::Vector2d d_0i;
			for (size_t i = 0; i < obstacles.size(); i++)
			{
				d_0i(0) = obstacles[i].kf.get_state()(0) - xs(0);
				d_0i(1) = obstacles[i].kf.get_state()(1) - xs(1);
				if (d_0i.norm() < pars.d_do_relevant)
					colav_active = true;

				// If all obstacles are passed, even though inside colav range,
				// then no need for colav
				/* if (CPU::ship_is_passed_by(xs, obstacles[i].kf.get_state(), pars.d_safe))
				{
					colav_active = false;
				}
				else
				{
					colav_active = true;
				} */
			}
			colav_active = colav_active || n_so > 0;

			return colav_active;
		}

		void PSBMPC::map_offset_sequences()
		{
			Eigen::VectorXd offset_sequence(2 * pars.n_M);
			Eigen::VectorXi offset_sequence_counter(2 * pars.n_M);
			reset_control_behaviour(offset_sequence_counter, offset_sequence);

			TML::PDMatrix<float, 2 * MAX_N_M, 1> tml_offset_sequence(2 * pars.n_M, 1);

			cb_dvec.resize(pars.n_cbs);
			for (int cb = 0; cb < pars.n_cbs; cb++)
			{
				TML::assign_eigen_object(tml_offset_sequence, offset_sequence);

				cb_dvec[cb] = tml_offset_sequence;

				increment_control_behaviour(offset_sequence_counter, offset_sequence);
			}
			// std::cout << "Number of control behaviours: " << pars.n_cbs << std::endl;
		}

		void PSBMPC::reset_control_behaviour(
			Eigen::VectorXi &offset_sequence_counter, // In/out: Counter to keep track of current offset sequence
			Eigen::VectorXd &offset_sequence		  // In/out: Control behaviour to increment
		)
		{
			offset_sequence_counter.setZero();
			for (int M = 0; M < pars.n_M; M++)
			{
				offset_sequence(2 * M) = pars.u_offsets[M](0);
				offset_sequence(2 * M + 1) = pars.chi_offsets[M](0);
			}
		}

		void PSBMPC::increment_control_behaviour(
			Eigen::VectorXi &offset_sequence_counter, // In/out: Counter to keep track of current offset sequence
			Eigen::VectorXd &offset_sequence		  // In/out: Control behaviour to increment
		)
		{
			double value(0.0);
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

				offset_sequence(2 * M + 1) = pars.chi_offsets[M](offset_sequence_counter.coeff(2 * M + 1));

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
				offset_sequence(2 * M) = pars.u_offsets[M](offset_sequence_counter.coeff(2 * M));
			}
		}

		void PSBMPC::map_thrust_dvecs(
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const Static_Obstacles &polygons	// In: Static obstacle information
		)
		{
			// Figure out how many threads to schedule
			int n_do = obstacles.size();
			int n_do_ps_total(0), n_ps(0);
			int n_so = polygons.size();
			for (int i = 0; i < n_do; i++)
			{
				n_ps = obstacles[i].get_trajectories().size();
				n_do_ps_total += n_ps;
			}
			// Total number of GPU threads to schedule
			int n_threads = pars.n_cbs * (n_so + n_do_ps_total);
			std::cout << "n_threads = " << n_threads << " | n_so = " << n_so << " | n_do_ps_total = " << n_do_ps_total << std::endl;

			cb_dvec.resize(n_threads);
			cb_index_dvec.resize(n_threads);
			sobstacle_index_dvec.resize(n_threads);
			dobstacle_index_dvec.resize(n_threads);
			dobstacle_ps_index_dvec.resize(n_threads);
			os_do_ps_pair_index_dvec.resize(n_threads);
			thread_index_dvec.resize(n_threads);
			thrust::sequence(thread_index_dvec.begin(), thread_index_dvec.end(), 0);
			cb_costs_2_dvec.resize(n_threads);

			int thread_index(0), os_do_ps_pair_index(0);
			TML::PDMatrix<float, 2 * MAX_N_M, 1> offset_sequence_tml(2 * pars.n_M);
			Eigen::VectorXd offset_sequence(2 * pars.n_M);
			Eigen::VectorXi offset_sequence_counter(2 * pars.n_M);
			reset_control_behaviour(offset_sequence_counter, offset_sequence);
			for (int cb = 0; cb < pars.n_cbs; cb++)
			{
				TML::assign_eigen_object(offset_sequence_tml, offset_sequence);

				for (int j = 0; j < n_so; j++)
				{
					cb_dvec[thread_index] = offset_sequence_tml;
					cb_index_dvec[thread_index] = cb;

					sobstacle_index_dvec[thread_index] = j;

					// Set dynamic obstacle, prediction scenario index  and cpe index to -1
					// for threads where only the grounding cost should be computed
					dobstacle_index_dvec[thread_index] = -1;
					dobstacle_ps_index_dvec[thread_index] = -1;
					os_do_ps_pair_index_dvec[thread_index] = -1;
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

				for (int i = 0; i < n_do; i++)
				{
					n_ps = obstacles[i].get_trajectories().size();
					for (int ps = 0; ps < n_ps; ps++)
					{
						cb_dvec[thread_index] = offset_sequence_tml;
						cb_index_dvec[thread_index] = cb;

						// Set static obstacle index to -1 for threads where only dynamic
						// obstacle related costs/indicators should be computed
						sobstacle_index_dvec[thread_index] = -1;

						dobstacle_index_dvec[thread_index] = i;
						dobstacle_ps_index_dvec[thread_index] = ps;
						os_do_ps_pair_index_dvec[thread_index] = os_do_ps_pair_index;

						thread_index += 1;
						os_do_ps_pair_index += 1;

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

		void PSBMPC::find_optimal_control_behaviour(
			const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
			const Static_Obstacles &polygons	// In: Static obstacle information
		)
		{
			int n_do = obstacles.size();
			int n_so = polygons.size();
			int thread_index(0), n_ps(0);
			Eigen::VectorXd offset_sequence(2 * pars.n_M);
			Eigen::VectorXi offset_sequence_counter(2 * pars.n_M);
			reset_control_behaviour(offset_sequence_counter, offset_sequence);

			Eigen::VectorXd h_do_i_ps, h_so_j(n_so), h_colregs_i_ps, cost_colregs(n_do), cost_do(n_do);

			double cost(0.0), h_do(0.0), h_colregs(0.0), h_so(0.0), h_path(0.0);
			min_cost = 1e12;

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			Engine *ep = engOpen(NULL);
			if (ep == NULL)
			{
				std::cout << "engine start failed!" << std::endl;
			}

			Eigen::MatrixXd n_ps_matrix(1, n_do);
			int n_ps_max(0);
			for (int i = 0; i < n_do; i++)
			{
				n_ps = obstacles[i].get_trajectories().size();
				n_ps_matrix(0, i) = n_ps;
				if (n_ps_max < n_ps)
				{
					n_ps_max = n_ps;
				}
			}
			Eigen::MatrixXd Pr_s_i_matrix(n_do, n_ps_max);
			for (int i = 0; i < n_do; i++)
			{
				for (int ps = 0; ps < n_ps; ps++)
				{
					Pr_s_i_matrix(i, ps) = obstacles[i].get_scenario_probabilities()(ps);
				}
			}
			int curr_ps_index(0), min_index(0);

			mxArray *total_cost_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
			mxArray *cost_do_mx = mxCreateDoubleMatrix(n_do, pars.n_cbs, mxREAL);
			mxArray *cost_colregs_mx = mxCreateDoubleMatrix(1, pars.n_cbs, mxREAL);
			mxArray *h_do_i_ps_mx = mxCreateDoubleMatrix(n_do * n_ps_max, pars.n_cbs, mxREAL);
			mxArray *h_so_j_mx = mxCreateDoubleMatrix(n_so, pars.n_cbs, mxREAL);
			mxArray *cost_so_path_mx = mxCreateDoubleMatrix(2, pars.n_cbs, mxREAL);
			mxArray *n_ps_mx = mxCreateDoubleMatrix(1, n_do, mxREAL);
			mxArray *cb_matrix_mx = mxCreateDoubleMatrix(2 * pars.n_M, pars.n_cbs, mxREAL);
			mxArray *Pr_s_i_mx = mxCreateDoubleMatrix(n_do, n_ps_max, mxREAL);

			double *ptr_total_cost = mxGetPr(total_cost_mx);
			double *ptr_cost_do = mxGetPr(cost_do_mx);
			double *ptr_cost_colregs = mxGetPr(cost_colregs_mx);
			double *ptr_h_do_i_ps = mxGetPr(h_do_i_ps_mx);
			double *ptr_h_so_j = mxGetPr(h_so_j_mx);
			double *ptr_cost_so_path = mxGetPr(cost_so_path_mx);
			double *ptr_n_ps = mxGetPr(n_ps_mx);
			double *ptr_cb_matrix = mxGetPr(cb_matrix_mx);
			double *ptr_Pr_s_i = mxGetPr(Pr_s_i_mx);

			Eigen::Map<Eigen::MatrixXd> map_total_cost(ptr_total_cost, 1, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_do(ptr_cost_do, n_do, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_colregs(ptr_cost_colregs, 1, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_h_do_i_ps(ptr_h_do_i_ps, n_do * n_ps_max, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_h_so_j(ptr_h_so_j, n_so, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_cost_so_path(ptr_cost_so_path, 2, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_n_ps(ptr_n_ps, 1, n_do);
			Eigen::Map<Eigen::MatrixXd> map_cb_matrix(ptr_cb_matrix, 2 * pars.n_M, pars.n_cbs);
			Eigen::Map<Eigen::MatrixXd> map_Pr_s_i(ptr_Pr_s_i, n_do, n_ps_max);

			mxArray *n_do_mx = mxCreateDoubleScalar(n_do), *n_so_mx = mxCreateDoubleScalar(n_so), *opt_cb_index_mx(nullptr);

			Eigen::MatrixXd cost_do_matrix(n_do, pars.n_cbs);
			Eigen::MatrixXd cost_colregs_matrix(1, pars.n_cbs);
			Eigen::MatrixXd h_do_i_ps_matrix(n_do * n_ps_max, pars.n_cbs);
			Eigen::MatrixXd h_so_j_matrix(n_so, pars.n_cbs);
			if (n_so == 0)
			{
				h_so_j.resize(1);
				h_so_j_matrix.resize(1, pars.n_cbs);
				h_so_j.setZero();
				h_so_j_matrix.setZero();
			}
			Eigen::MatrixXd cb_matrix(2 * pars.n_M, pars.n_cbs);
			Eigen::MatrixXd cost_so_path_matrix(2, pars.n_cbs);
			Eigen::MatrixXd total_cost_matrix(1, pars.n_cbs);
#endif
			//=============================================================================================================

			thrust::tuple<float, float, float> dev_tup;
			for (int cb = 0; cb < pars.n_cbs; cb++)
			{
				h_do = 0.0;
				h_colregs = 0.0;
				h_so = 0.0;
				h_path = 0.0;
				cost = 0.0;

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
				for (int M = 0; M < pars.n_M; M++)
				{
					cb_matrix(2 * M, cb) = offset_sequence(2 * M);
					cb_matrix(2 * M + 1, cb) = RAD2DEG * offset_sequence(2 * M + 1);
				}
				curr_ps_index = 0;
				// std::cout << "offset sequence counter = " << offset_sequence_counter.transpose() << std::endl;
				// std::cout << "offset sequence = " << offset_sequence.transpose() << std::endl;
#endif
				//===================================================================

				for (int j = 0; j < n_so; j++)
				{
					dev_tup = cb_costs_2_dvec[thread_index];
					h_so_j(j) = (double)thrust::get<0>(dev_tup);
					thread_index += 1;
					/* printf("Thread %d | j = %d | i = -1 | ps = -1 | h_so_j : %.4f | h_do_i_ps : DC | cb_index : %d | cb : %.1f, %.1f \n",
					thread_index, j, h_so_j(j), cb, offset_sequence(0), RAD2DEG * offset_sequence(1)); */
				}
				if (n_so > 0)
				{
					h_so = h_so_j.maxCoeff();
				}

				for (int i = 0; i < n_do; i++)
				{
					n_ps = obstacles[i].get_trajectories().size();
					h_do_i_ps.resize(n_ps);
					h_colregs_i_ps.resize(n_ps);
					for (int ps = 0; ps < n_ps; ps++)
					{
						dev_tup = cb_costs_2_dvec[thread_index];
						h_do_i_ps(ps) = (double)thrust::get<1>(dev_tup);
						h_colregs_i_ps(ps) = (double)thrust::get<2>(dev_tup);
						thread_index += 1;
						/* printf("Thread %d | j = -1 | i = %d | ps = %d | h_so_j : DC | h_do_i_ps : %.4f | cb_index : %d | cb : %.1f, %.1f \n",
					thread_index, i, ps, h_do_i_ps(ps), cb, offset_sequence(0), RAD2DEG * offset_sequence(1)); */
					}

					cost_do(i) = mpc_cost.calculate_dynamic_obstacle_cost(h_do_i_ps, obstacles, i);

					cost_colregs(i) = mpc_cost.calculate_colregs_violation_cost(h_colregs_i_ps, obstacles, i);

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
					h_do_i_ps_matrix.block(curr_ps_index, cb, n_ps, 1) = h_do_i_ps;
					curr_ps_index += n_ps;
#endif
					//==================================================================
				}
				if (n_do > 0)
				{
					h_do = cost_do.sum();

					h_colregs = cost_colregs.sum();
				}

				h_path = cb_costs_1_dvec[cb];

				cost = h_do + h_colregs + h_so + h_path;

//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
				if (n_so > 0)
				{
					h_so_j_matrix.block(0, cb, n_so, 1) = h_so_j;
				}
				cost_so_path_matrix(0, cb) = h_so;
				if (n_do > 0)
				{
					cost_do_matrix.col(cb) = cost_do;
				}
				cost_colregs_matrix(0, cb) = h_colregs;
				cost_so_path_matrix(1, cb) = h_path;
				total_cost_matrix(cb) = cost;
				if (cost < min_cost)
				{
					min_index = cb;
				}
#endif
				//===================================================================

				if (cost < min_cost)
				{
					min_cost = cost;
					opt_offset_sequence = offset_sequence;
				}

				increment_control_behaviour(offset_sequence_counter, offset_sequence);
			}
//==================================================================
// MATLAB PLOTTING FOR DEBUGGING AND TUNING
//==================================================================
#if ENABLE_PSBMPC_DEBUGGING
			opt_cb_index_mx = mxCreateDoubleScalar(min_index + 1);
			map_total_cost = total_cost_matrix;
			if (n_so > 0)
			{
				map_h_so_j = h_so_j_matrix;
			}
			if (n_do > 0)
			{
				map_cost_do = cost_do_matrix;
				map_h_do_i_ps = h_do_i_ps_matrix;
				map_Pr_s_i = Pr_s_i_matrix;
			}
			map_cost_colregs = cost_colregs_matrix;
			map_cost_so_path = cost_so_path_matrix;
			map_n_ps = n_ps_matrix;
			map_cb_matrix = cb_matrix;

			mxArray *is_gpu_mx = mxCreateDoubleScalar(1);
			engPutVariable(ep, "is_gpu", is_gpu_mx);
			engPutVariable(ep, "h_so_j", h_so_j_mx);
			engPutVariable(ep, "cost_do", cost_do_mx);
			engPutVariable(ep, "h_do_i_ps", h_do_i_ps_mx);
			engPutVariable(ep, "Pr_s_i", Pr_s_i_mx);
			engPutVariable(ep, "total_cost", total_cost_mx);
			engPutVariable(ep, "cost_colregs", cost_colregs_mx);
			engPutVariable(ep, "cost_so_path", cost_so_path_mx);
			engPutVariable(ep, "n_ps", n_ps_mx);
			engPutVariable(ep, "cb_matrix", cb_matrix_mx);
			engPutVariable(ep, "n_do", n_do_mx);
			engPutVariable(ep, "n_so", n_so_mx);
			engPutVariable(ep, "opt_cb_index", opt_cb_index_mx);
			engEvalString(ep, "psbmpc_cost_plotting");

			mxDestroyArray(is_gpu_mx);
			mxDestroyArray(total_cost_mx);
			mxDestroyArray(cost_do_mx);
			mxDestroyArray(cost_colregs_mx);
			mxDestroyArray(h_do_i_ps_mx);
			mxDestroyArray(h_so_j_mx);
			mxDestroyArray(cost_so_path_mx);
			mxDestroyArray(n_ps_mx);
			mxDestroyArray(cb_matrix_mx);
			mxDestroyArray(n_do_mx);
			mxDestroyArray(n_so_mx);
			mxDestroyArray(opt_cb_index_mx);

			engClose(ep);
#endif
			//==================================================================
		}

		void PSBMPC::setup_prediction(
			const Dynamic_Obstacles &obstacles // In: Dynamic obstacle information
		)
		{
			int n_do = obstacles.size();
			//***********************************************************************************
			// Own-ship prediction initialization
			//***********************************************************************************
			Eigen::VectorXd t_cpa(n_do), d_cpa(n_do);
			Eigen::Vector2d p_cpa, v_0;
			Eigen::Vector4d xs_0, xs_i_0;
			if (trajectory.rows() == 4)
			{
				v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
				v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
			}
			else
			{
				v_0(0) = trajectory(3, 0);
				v_0(1) = trajectory(4, 0);
				v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
			}
			xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
			xs_0(2) = v_0(0);
			xs_0(3) = v_0(1);
			// First avoidance maneuver is always at t0
			maneuver_times.setZero();

			double t_cpa_min(0.0), d_safe_i(0.0);
			std::vector<bool> maneuvered_by(n_do);
			int index_closest(-1);
			for (int M = 1; M < pars.n_M; M++)
			{
				// If a predicted collision occurs with the closest obstacle, avoidance maneuver
				// M is taken right after the obstacle possibly maneuvers (modelled to be at t_0 + M * t_ts
				// given that t_cpa > t_ts. If t_cpa < t_ts or n_do = 0, the subsequent maneuver is taken
				// at t_{M-1} + t_ts + 1 anyways (simplification)
				maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);

				// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
				t_cpa_min = 1e10;
				index_closest = -1;
				for (int i = 0; i < n_do; i++)
				{
					xs_i_0 = obstacles[i].kf.get_state();
					CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

					d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[i].get_length());
					// For the current avoidance maneuver, determine which obstacle that should be
					// considered, i.e. the closest obstacle that is not already passed (which means
					// that the previous avoidance maneuver happened before CPA with this obstacle)
					if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min && t_cpa(i) <= pars.T)
					{
						t_cpa_min = t_cpa(i);
						index_closest = i;
					}
				}

				/* if (index_closest != -1)
				{
					d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[index_closest].get_width());
					// If no predicted collision,  avoidance maneuver M with the closest
					// obstacle (that is not passed) is taken at t_cpa_min
					if (d_cpa(index_closest) > d_safe_i)
					{
						//std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
						maneuvered_by[index_closest] = true;
						maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
					}
				} */
			}

			// std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
		}

		void PSBMPC::assign_optimal_trajectory(
			Eigen::MatrixXd &optimal_trajectory // In/out: Optimal PSB-MPC trajectory
		)
		{
			int n_samples = std::round(pars.T / pars.dt);
			optimal_trajectory.resize(trajectory.rows(), n_samples);
			optimal_trajectory = trajectory.block(0, 0, trajectory.rows(), n_samples);
		}

		void PSBMPC::set_up_temporary_device_memory(
			const double u_d,							   // In: Own-ship surge reference
			const double chi_d,							   // In: Own-ship course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Own-ship waypoints to follow
			const double V_w,							   // In: Estimated wind speed
			const Eigen::Vector2d &wind_direction,		   // In: Unit vector in NE describing the estimated wind direction
			const Static_Obstacles &polygons,			   // In: Static obstacle information represented as polygons
			const Dynamic_Obstacles &obstacles			   // In: Dynamic obstacle information
		)
		{
			int n_do = obstacles.size();
			int n_so = polygons.size();

			size_t limit = 0;

			cudaDeviceSetLimit(cudaLimitStackSize, 100000);
			cuda_check_errors("Setting cudaLimitStackSize failed.");

			cudaDeviceGetLimit(&limit, cudaLimitStackSize);
			cuda_check_errors("Reading cudaLimitStackSize failed.");
			// std::cout << "Set device max stack size : " << limit << std::endl;

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
				obstacles);
			cudaMemcpy(fdata_device_ptr, &temporary_fdata, sizeof(CB_Functor_Data), cudaMemcpyHostToDevice);
			cuda_check_errors("CudaMemCpy of CB_Functor_Data failed.");

			// Dynamic obstacles and COLREGS Violation Evaluators
			Cuda_Obstacle temp_transfer_cobstacle;
			for (int i = 0; i < n_do; i++)
			{
				temp_transfer_cobstacle = obstacles[i];

				cudaMemcpy(&obstacles_device_ptr[i], &temp_transfer_cobstacle, sizeof(Cuda_Obstacle), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of Cuda Obstacle i failed.");
			}

			// Static obstacles
			Basic_Polygon temp_transfer_poly;
			for (int j = 0; j < n_so; j++)
			{
				temp_transfer_poly = polygons[j];

				cudaMemcpy(&polygons_device_ptr[j], &temp_transfer_poly, sizeof(Basic_Polygon), cudaMemcpyHostToDevice);
				cuda_check_errors("CudaMemCpy of Basic Polygon j failed.");
			}
		}

		void PSBMPC::assign_data(const PSBMPC &other)
		{
			opt_offset_sequence = other.opt_offset_sequence;
			maneuver_times = other.maneuver_times;

			u_opt_last = other.u_opt_last;
			chi_opt_last = other.chi_opt_last;

			min_cost = other.min_cost;

			ownship = other.ownship;

			trajectory = other.trajectory;

			cpe_gpu = other.cpe_gpu;
			colregs_violation_evaluator_gpu = other.colregs_violation_evaluator_gpu;

			pars = other.pars;
			mpc_cost = other.mpc_cost;
		}

		void PSBMPC::free()
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

			cudaFree(colregs_violation_evaluators_device_ptr);
			cuda_check_errors("CudaFree of COLREGS Violation Evaluators failed.");
		}

	}
}