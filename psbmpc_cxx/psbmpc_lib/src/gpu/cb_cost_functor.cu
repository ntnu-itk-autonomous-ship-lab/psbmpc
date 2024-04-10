/****************************************************************************************
 *
 *  File name : cb_cost_functor.cu
 *
 *  Function  : Class functions for the control behaviour cost evaluation functors. Used in
 *			   the thrust framework for GPU calculations.
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

#include "psbmpc_defines.hpp"
#include "gpu/cb_cost_functor.cuh"
#include "gpu/cuda_obstacle.cuh"
#include "gpu/cpe_gpu.cuh"
#include "gpu/utilities_gpu.cuh"
#include "gpu/mpc_cost_gpu.cuh"
#include "gpu/colregs_violation_evaluator.cuh"

#include <cmath>
#include <iostream>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		//=======================================================================================
		//  CB COST FUNCTOR 1 METHODS
		//=======================================================================================

		//=======================================================================================
		//  Name     : operator()
		//  Function : Predicts the trajectory of the ownship for a certain control behaviour,
		//			   and calculates the static obstacle and path related costs
		//  Author   : Trym Tengesdal
		//  Modified :
		//=======================================================================================
		__device__ float CB_Cost_Functor_1::operator()(
			const thrust::tuple<const unsigned int, TML::PDMatrix<float, 2 * MAX_N_M, 1>> &cb_tuple // In: Tuple consisting of the index and vector for the control behaviour evaluated in this kernel
		)
		{
			h_path = 0.0f;
			max_cross_track_error = 0.0f;
			cb_index = thrust::get<0>(cb_tuple);
			offset_sequence = thrust::get<1>(cb_tuple);

			ownship[cb_index].set_wp_counter(fdata->wp_c_0);
			ownship[cb_index].predict_trajectory(
				trajectory[cb_index],
				max_cross_track_error,
				fdata->ownship_state,
				offset_sequence,
				fdata->maneuver_times,
				fdata->u_d, fdata->chi_d,
				fdata->waypoints,
				pars->prediction_method,
				pars->guidance_method,
				pars->T, pars->dt);

			h_path += mpc_cost[cb_index].calculate_control_deviation_cost(
				offset_sequence,
				fdata->u_opt_last,
				fdata->chi_opt_last,
				max_cross_track_error);

			return h_path;
		}

		//=======================================================================================
		//  CB COST FUNCTOR 2 METHODS
		//=======================================================================================
		/****************************************************************************************
		 *  Name     : operator()
		 *  Function : Evaluates partial static and dynamic obstacle cost terms when following
		 *			  the control behaviour given by the input tuple, in addition to determining
		 *			  the COLREGS violation indicator in a certain scenario.
		 *  Author   : Trym Tengesdal
		 *  Modified :
		 *****************************************************************************************/
		__device__ thrust::tuple<float, float, float> CB_Cost_Functor_2::operator()(
			const thrust::tuple<
				const int,							  // Thread index
				TML::PDMatrix<float, 2 * MAX_N_M, 1>, // Control behaviour considered
				const int,							  // Control behaviour index
				const int,							  // Static obstacle j considered
				const int,							  // Dynamic obstacle i considered
				const int,							  // Prediction scenario ps for the dynamic obstacle
				const int> &input_tuple				  // CPE object and COLREGS Violation Evaluator object index
		)
		{
			int effective_p_step; // Fixing the last iteration by decreasing p_step, if necessary

			h_so_j = 0.0f;
			h_do_i_ps = 0.0f;
			h_colregs_i_ps = 0.0f;

			thread_index = thrust::get<0>(input_tuple);
			offset_sequence = thrust::get<1>(input_tuple);
			cb_index = thrust::get<2>(input_tuple);
			j = thrust::get<3>(input_tuple);
			i = thrust::get<4>(input_tuple);
			ps = thrust::get<5>(input_tuple);
			os_do_ps_pair_index = thrust::get<6>(input_tuple);

			// Check if the thread is supposed to calculated the grounding cost
			if (j >= 0 || i == -1 || ps == -1)
			{
				h_so_j = mpc_cost[thread_index].calculate_grounding_cost(trajectory[cb_index], fdata, polygons[j]);

				// Last two cost elements are dont care arguments for GPU threads only computing
				// the partial static obstacle cost
				return thrust::tuple<float, float, float>(h_so_j, -1.0f, -1.0f);
			}
			// Else: Calculate the partial dynamic obstacle cost and COLREGS violation indicator

			n_samples = round(pars->T / pars->dt);

			d_safe_i = 0.0;
			h_do_i_ps_k = 0.0;

			// Seed collision probability estimator using the cpe index
			cpe[os_do_ps_pair_index].seed_prng(os_do_ps_pair_index);

			// In case cpe_method = MCSKF4D, this is the number of samples in the segment considered
			n_seg_samples = round(cpe[os_do_ps_pair_index].get_segment_discretization_time() / pars->dt) + 1;

			xs_p_seg.resize(4, n_seg_samples);
			xs_i_p_seg.resize(4, n_seg_samples);
			P_i_p_seg.resize(16, n_seg_samples);

			d_safe_i = pars->d_safe + 0.5 * (fdata->ownship_length + obstacles[i].get_length());
			// printf("d_safe = %.2f | d_safe_i = %.6f\n", pars->d_safe, d_safe_i);
			v_os_prev.set_zero();
			v_i_prev.set_zero();

			d_cpa = 1e10;

			/* printf("Thread %d | i = %d | ps = %d | k = %d | n_ps = %d | n_samples = %d | cb : %.1f, %.1f\n", thread_index, i, ps,
				   0, obstacles[i].get_n_prediction_scenarios(), obstacles[i].get_n_trajectory_samples(), offset_sequence(0), RAD2DEG * offset_sequence(1));
 */
			colregs_violation_evaluators[os_do_ps_pair_index].reset();
			for (int k = 0; k < n_samples; k += pars->p_step_do)
			{
				xs_p_seg.shift_columns_left();
				xs_p_seg.set_col(n_seg_samples - 1, trajectory[cb_index].get_col(k));

				P_i_p_seg.shift_columns_left();
				P_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_covariance_sample(k));

				xs_i_p_seg.shift_columns_left();
				xs_i_p_seg.set_col(n_seg_samples - 1, obstacles[i].get_trajectory_sample(ps, k));

				p_os = xs_p_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);
				p_i = xs_i_p_seg.get_block<2, 1>(0, n_seg_samples - 1, 2, 1);

				if (k == 0)
				{
					chi_0 = xs_p_seg(2, n_seg_samples - 1);
					U_0 = xs_p_seg(3, n_seg_samples - 1);
					d_0i_0 = (p_os - p_i).norm();
					xs_cpa = xs_p_seg.get_col(n_seg_samples - 1);
					xs_i_cpa = xs_i_p_seg.get_col(n_seg_samples - 1);

					cpe[os_do_ps_pair_index].initialize(xs_cpa, xs_i_cpa, d_safe_i);

					colregs_violation_evaluators[os_do_ps_pair_index].update(xs_cpa, xs_i_cpa); //
				}

				/* printf("k = %d | xs_p = %.1f, %.1f, %.1f, %.1f\n", k, xs_p_seg(0, n_seg_samples - 1), xs_p_seg(1, n_seg_samples - 1), xs_p_seg(2, n_seg_samples - 1), xs_p_seg(3, n_seg_samples - 1));
				printf("k = %d | xs_i_p = %.1f, %.1f, %.1f, %.1f\n", k, xs_i_p_seg(0, n_seg_samples - 1), xs_i_p_seg(1, n_seg_samples - 1), xs_i_p_seg(2, n_seg_samples - 1), xs_i_p_seg(3, n_seg_samples - 1));
 */
				/* printf("P_i_p = %.1f, %.1f, %.1f, %.1f\n", P_i_p(0, n_seg_samples - 1), P_i_p(1, n_seg_samples - 1), P_i_p(2, n_seg_samples - 1), P_i_p(3, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(4, n_seg_samples - 1), P_i_p(5, n_seg_samples - 1), P_i_p(6, n_seg_samples - 1), P_i_p(7, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(8, n_seg_samples - 1), P_i_p(9, n_seg_samples - 1), P_i_p(10, n_seg_samples - 1), P_i_p(11, n_seg_samples - 1));
		printf("        %.1f, %.1f, %.1f, %.1f\n", P_i_p(12, n_seg_samples - 1), P_i_p(13, n_seg_samples - 1), P_i_p(14, n_seg_samples - 1), P_i_p(15, n_seg_samples - 1)); */

				switch (pars->cpe_method)
				{
				case CE:
					if (k > 0)
					{
						// Map [chi, U]^T to [Vx, Vy]
						v_os_prev(0) = xs_p_seg(3, n_seg_samples - 2) * cos(xs_p_seg(2, n_seg_samples - 2));
						v_os_prev(1) = xs_p_seg(3, n_seg_samples - 2) * sin(xs_p_seg(2, n_seg_samples - 2));

						v_i_prev = xs_i_p_seg.get_block<2, 1>(2, n_seg_samples - 2, 2, 1);
					}

					P_i_2D = reshape<16, 1, 4, 4>(P_i_p_seg.get_col(n_seg_samples - 1), 4, 4).get_block<2, 2>(0, 0, 2, 2);

					effective_p_step = std::min(p_step, n_samples - k);

					P_c_i = cpe[os_do_ps_pair_index].CE_estimate(p_os, p_i, P_i_2D, v_os_prev, v_i_prev, pars->dt * effective_p_step);
					break;
				case MCSKF4D:
					if (fmod(k, n_seg_samples - 1) == 0 && k > 0)
					{
						P_c_i = cpe[os_do_ps_pair_index].MCSKF4D_estimate(xs_p_seg, xs_i_p_seg, P_i_p_seg);
					}
					break;
				default:
					// Throw
					break;
				}

				h_do_i_ps_k = mpc_cost[thread_index].calculate_dynamic_obstacle_cost(
					obstacles,
					P_c_i,
					xs_p_seg.get_col(n_seg_samples - 1),
					xs_i_p_seg.get_col(n_seg_samples - 1),
					i,
					k);

				if (h_do_i_ps < h_do_i_ps_k)
				{
					h_do_i_ps = h_do_i_ps_k;
				}

				d_0i = (p_os - p_i).norm();
				if (d_0i < d_cpa)
				{
					d_cpa = d_0i;
					xs_cpa = xs_p_seg.get_col(n_seg_samples - 1);
					xs_i_cpa = xs_i_p_seg.get_col(n_seg_samples - 1);
				}

				colregs_violation_evaluators[os_do_ps_pair_index].evaluate_predicted_maneuver_changes(
					xs_p_seg(2, n_seg_samples - 1),
					xs_p_seg(3, n_seg_samples - 1));
				//======================================

				//==========================================================================================
				/* printf("Thread %d | i = %d | ps = %d | k = %d | P_c_i = %.3f | h_do_i_ps = %.1f | pred_UCHI_changed = %d | pred_CHI_changed_port = %d | act_UCHI_changed = %d | act_CHI_changed_port = %d | correct_CR_SS_maneuver = %d |  cb : %.1f, %.1f\n", thread_index, i, ps, k, P_c_i, h_do_i_ps,
				colregs_violation_evaluators[os_do_ps_pair_index].predicted_ownship_change_in_speed_or_course, colregs_violation_evaluators[os_do_ps_pair_index].predicted_ownship_change_in_course_to_port,
				colregs_violation_evaluators[os_do_ps_pair_index].actual_ownship_speed_or_course_change, colregs_violation_evaluators[os_do_ps_pair_index].actual_ownship_course_change_port,
				colregs_violation_evaluators[os_do_ps_pair_index].correct_CR_SS_maneuver, offset_sequence(0), RAD2DEG * offset_sequence(1)); */

				//==============================================================================================
			}

			Pr_WGW = obstacles[i].get_Pr_WGW();
			Pr_CCEM = obstacles[i].get_Pr_CCEM();
			h_colregs_i_ps = Pr_CCEM * pars->kappa_GW * colregs_violation_evaluators[os_do_ps_pair_index].evaluate_GW_violation(xs_cpa, xs_i_cpa, d_cpa) +
							 Pr_WGW * pars->kappa_SO * colregs_violation_evaluators[os_do_ps_pair_index].evaluate_SO_violation(d_0i_0) +
							 pars->kappa_RA * colregs_violation_evaluators[os_do_ps_pair_index].evaluate_readily_apparent_violation(offset_sequence(1), offset_sequence(0));

			//==================================================================================================

			/* if (offset_sequence(0) == 1.0 && offset_sequence(1) == 0.0 && offset_sequence(2) == 1.0 && offset_sequence(3) == 0.0)
			{
				printf("i = %d | ps = %d | h_do_i_ps : %.2f| h_colregs_i_ps = %.0f | colregs sit = %d | p_CHI_chng_port = %d | p_UCHI_chng = %d | act_UCHI_chngd = %d | act_UCHI_chng_port = %d | crct_HO_man = %d | cb : %.1f, %.1f, %.1f, %.1f\n", i, ps, h_do_i_ps, h_colregs_i_ps,
					   colregs_violation_evaluators[os_do_ps_pair_index].colregs_situation, colregs_violation_evaluators[os_do_ps_pair_index].predicted_ownship_change_in_course_to_port,
					   colregs_violation_evaluators[os_do_ps_pair_index].predicted_ownship_change_in_speed_or_course,
					   colregs_violation_evaluators[os_do_ps_pair_index].actual_ownship_speed_or_course_change, colregs_violation_evaluators[os_do_ps_pair_index].actual_ownship_course_change_port,
					   colregs_violation_evaluators[os_do_ps_pair_index].correct_HO_maneuver, offset_sequence(0), RAD2DEG * offset_sequence(1), offset_sequence(2), RAD2DEG * offset_sequence(3));

				// printf("Thread %d | i = %d | ps = %d | cb index %d | h_do_i_ps : %.4f| h_colregs_i_ps : %.4f | Pr_WGW = %.4f | Pr_CCEM = %.4f | cb : %.1f, %.1f \n", thread_index, i, ps, cb_index, h_do_i_ps, h_colregs_i_ps, Pr_WGW, Pr_CCEM, offset_sequence(0), RAD2DEG * offset_sequence(1));
			} */
			//==================================================================================================

			// The first element (grounding cost element) is dont care for threads that only
			// compute the partial dynamic obstacle cost and COLREGS violation cost
			return thrust::tuple<float, float, float>(-1.0f, h_do_i_ps, h_colregs_i_ps);
		}

		//=======================================================================================
		//	Private functions
		//=======================================================================================

	}
}