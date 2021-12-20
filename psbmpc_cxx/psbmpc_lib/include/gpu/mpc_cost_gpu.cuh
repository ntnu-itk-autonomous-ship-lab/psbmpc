/****************************************************************************************
*
*  File name : mpc_cost_gpu.cuh
*
*  Function  : Header file for the device compatible MPC Cost class (PSB/SB-MPC), which
*              contains implementations for the COLAV MPC cost functions.
*
*            ---------------------
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

#pragma once

#include "psbmpc_parameters.hpp"
#include "colregs_violation_evaluator.cuh"
#include "cb_cost_functor_structures.cuh"
#include "cuda_obstacle.cuh"
#include "tml/tml.cuh"
#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		template <typename Parameters>
		class MPC_Cost
		{
		private:
			Parameters pars;

			//==============================================
			// Pre-allocated temporaries
			//==============================================
			float cost_cd, cost_ch, delta_t, h_so_j, h_so_j_k;

			// Dynamic obstacle cost related (do cost)
			float cost_k, cost_do, cost_coll, l_i;

			TML::Vector2f v_0_p, v_i_p;
			bool mu_GW, mu_SO;

			// Grounding hazard related
			TML::Vector2f v_diff, n, L_0j, d2poly, d2line, p_os_k, projection;
			TML::Vector2d p_ray_end, v_prev, v, v_next;

			float l_sqrt, t_line, d_0j, phi_j;
			double val, epsilon;
			int n_samples, o_11, o_12, o_1, o_2, o_3, o_4, line_intersect_count;

			TML::Vector3f a, b;
			//==============================================
			//==============================================

			__host__ __device__ inline float Delta_u(const float u_1, const float u_2) const { return pars.K_du * fabs(u_1 - u_2); }

			__host__ __device__ inline float Delta_chi(const float chi_1, const float chi_2) const
			{
				if (chi_1 > 0)
					return pars.K_dchi_strb * powf(fabs(chi_1 - chi_2), 2);
				else
					return pars.K_dchi_port * powf(fabs(chi_1 - chi_2), 2);
			}

			__host__ __device__ inline float K_chi(const float chi) const
			{
				if (chi > 0)
					return pars.K_chi_strb * powf(chi, 2);
				else
					return pars.K_chi_port * powf(chi, 2);
			}

		public:
			__host__ __device__ int find_triplet_orientation(const TML::Vector2d &p, const TML::Vector2d &q, const TML::Vector2d &r);

			__host__ __device__ bool determine_if_on_segment(const TML::Vector2d &p, const TML::Vector2d &q, const TML::Vector2d &r) const;

			__host__ __device__ bool determine_if_lines_intersect(const TML::Vector2d &p_1, const TML::Vector2d &q_1, const TML::Vector2d &p_2, const TML::Vector2d &q_2);

			__host__ __device__ bool determine_if_inside_polygon(const TML::Vector2d &p, const Basic_Polygon &poly);

			__host__ __device__ TML::Vector2f distance_to_line_segment(const TML::Vector2f &p, const TML::Vector2f &q_1, const TML::Vector2f &q_2);

			__host__ __device__ TML::Vector2f distance_to_polygon(const TML::Vector2f &p, const Basic_Polygon &poly);

			__host__ __device__ MPC_Cost() {}

			__host__ __device__ MPC_Cost(const Parameters &pars) : pars(pars) {}

			__device__ inline float calculate_dynamic_obstacle_cost(
				const CB_Functor_Data *fdata,
				const Cuda_Obstacle *obstacles,
				const float P_c_i,
				const TML::PDVector4f &xs_p,
				const TML::PDVector4f &xs_i_p,
				const int i,
				const int k);

			__host__ __device__ inline float calculate_collision_cost(const TML::Vector2f &v_1, const TML::Vector2f &v_2) const { return pars.K_coll * (powf(v_1(0) - v_2(0), 2) + powf(v_1(1) - v_2(1), 2)); }

			__host__ __device__ float calculate_control_deviation_cost(const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, const float u_opt_last, const float chi_opt_last);

			__host__ __device__ float calculate_chattering_cost(const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times);

			__host__ __device__ float calculate_grounding_cost(
				const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory,
				const CB_Functor_Data *fdata,
				const Basic_Polygon &poly);
		};

		//=======================================================================================
		//  Name     : calculate_dynamic_obstacle_cost
		//  Function : Calculates maximum (wrt to time) hazard with dynamic obstacle i considering
		//			   a certain time instant for the ownship and obstacle i in pred. scen. ps.
		//  Author   : Trym Tengesdal
		//  Modified :
		//=======================================================================================
		template <typename Parameters>
		__device__ inline float MPC_Cost<Parameters>::calculate_dynamic_obstacle_cost(
			const CB_Functor_Data *fdata,	// In: Pointer to control behaviour functor data
			const Cuda_Obstacle *obstacles, // In: Pointer to Cuda_Obstacle array
			const float P_c_i,				// In: Predicted obstacle collision probabilities for obstacle in prediction scenario ps
			const TML::PDVector4f &xs_p,	// In: Predicted own-ship state at time step k
			const TML::PDVector4f &xs_i_p,	// In: Predicted obstacle state at time step k in prediction scenario ps
			const int i,					// In: Index of obstacle in consideration
			const int k						// In: Prediction time index
		)
		{
			cost_do = 0.0f;

			// l_i is the collision cost modifier depending on the obstacle track loss.
			cost_coll = 0.0f;
			l_i = 0.0f;

			v_0_p(0) = xs_p(3) * cos(xs_p(2));
			v_0_p(1) = xs_p(3) * sin(xs_p(2));
			v_i_p(0) = xs_i_p(2);
			v_i_p(1) = xs_i_p(3);

			cost_coll = calculate_collision_cost(v_0_p, v_i_p);

			// Track loss modifier to collision cost
			if (obstacles[i].get_duration_lost() > pars.p_step_do)
			{
				l_i = pars.dt * pars.p_step_do / obstacles[i].get_duration_lost();
			}
			else
			{
				l_i = 1;
			}

			// Should discount time when using a prediction scheme where the uncertainty for
			// each obstacle prediction scenario is bounded by r_ct
			//cost_do = l_i * cost_coll * P_c_i;
			cost_do = l_i * cost_coll * P_c_i * exp(-(float)k * pars.dt / pars.T_sgn);

			/* printf("k = %d | C = %.4f | P_c_i = %.6f | v_i_p = %.2f, %.2f | psi_0_p = %.2f | v_0_p = %.2f, %.2f | d_0i_p = %.2f | L_0i_p = %.2f, %.2f\n",
				k, cost_coll, P_c_i, v_i_p(0), v_i_p(1), psi_0_p, v_0_p(0), v_0_p(1), d_0i_p, L_0i_p(0), L_0i_p(1)); */
			return cost_do;
		}

		/****************************************************************************************
		*  Name     : calculate_control_deviation_cost
		*  Function : Determines penalty due to using offsets to guidance references ++
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ float MPC_Cost<Parameters>::calculate_control_deviation_cost(
			const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, // In: Control behaviour currently followed
			const float u_opt_last,										 // In: Previous optimal output surge modification
			const float chi_opt_last									 // In: Previous optimal output course modification
		)
		{
			cost_cd = 0.0f;
			for (int M = 0; M < pars.n_M; M++)
			{
				if (M == 0)
				{
					cost_cd += pars.K_u * (1.0f - offset_sequence[0]) + Delta_u(offset_sequence[0], u_opt_last) +
							   K_chi(offset_sequence[1]) + Delta_chi(offset_sequence[1], chi_opt_last);
				}
				else
				{
					cost_cd += pars.K_u * (1.0f - offset_sequence[2 * M]) + Delta_u(offset_sequence[2 * M], offset_sequence[2 * M - 2]) +
							   K_chi(offset_sequence[2 * M + 1]) + Delta_chi(offset_sequence[2 * M + 1], offset_sequence[2 * M - 1]);
				}
			}

			/* printf("K_u (1 - u_m_0) = %.4f | Delta_u(u_m_0, u_opt_last) = %.4f | K_chi(chi_0) = %.4f | Delta_chi(chi_0, chi_last) = %.4f\n",
				pars.K_u * (1 - offset_sequence[0]), Delta_u(offset_sequence[0], fdata->u_opt_last), K_chi(offset_sequence[1]), Delta_chi(offset_sequence[1], fdata->chi_opt_last)); */
			return cost_cd / (float)pars.n_M;
		}

		/****************************************************************************************
		*  Name     : calculate_chattering_cost
		*  Function : Determines penalty due to using wobbly (changing between positive and negative)
		* 			  course modifications
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ float MPC_Cost<Parameters>::calculate_chattering_cost(
			const TML::PDMatrix<float, 2 * MAX_N_M, 1> &offset_sequence, // In: Control behaviour currently followed
			const TML::PDMatrix<float, MAX_N_M, 1> &maneuver_times		 // In: Time of each maneuver in the offset sequence
		)
		{
			cost_ch = 0.0f;
			if (pars.n_M == 1)
			{
				return cost_ch;
			}

			delta_t = 0.0f;
			for (int M = 0; M < pars.n_M - 1; M++)
			{
				if ((offset_sequence(2 * M + 1) > 0 && offset_sequence(2 * M + 3) < 0) ||
					(offset_sequence(2 * M + 1) < 0 && offset_sequence(2 * M + 3) > 0))
				{
					delta_t = maneuver_times(M + 1) - maneuver_times(M);
					cost_ch += pars.K_sgn * exp(-delta_t / pars.T_sgn);
				}
			}
			return cost_ch / (float)(pars.n_M - 1);
		}

		template <typename Parameters>
		__host__ __device__ float MPC_Cost<Parameters>::calculate_grounding_cost(
			const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &trajectory, // In: Calling Own-ship trajectory
			const CB_Functor_Data *fdata,							  // In: Pointer to various device data needed for the GPU calculations
			const Basic_Polygon &poly								  // In: Static obstacle to compute grounding cost wrt
		)
		{
			h_so_j = 0.0f;
			h_so_j_k = 0.0f;

			n_samples = trajectory.get_cols();
			for (int k = 0; k < n_samples; k += pars.p_step_grounding)
			{
				p_os_k = trajectory.get_block<2, 1>(0, k, 2, 1);

				L_0j = distance_to_polygon(p_os_k, poly);
				d_0j = L_0j.norm();
				L_0j.normalize();

				phi_j = fmaxf(0.0f, L_0j.dot(fdata->wind_direction));

				if (d_0j >= pars.d_safe)
				{
					h_so_j_k = (pars.G_1 + pars.G_2 * phi_j * fdata->V_w * fdata->V_w) * expf(-(pars.G_3 * fabs(d_0j - pars.d_safe) + pars.G_4 * (float)k * pars.dt));
				}
				else
				{
					h_so_j_k = (pars.G_1 + pars.G_2 * phi_j * fdata->V_w * fdata->V_w) * expf(-pars.G_4 * (float)k * pars.dt);
				}

				if (h_so_j < h_so_j_k)
				{
					h_so_j = h_so_j_k;
				}
				/* if (k == 0)
					printf("t = %.1f | d_0j = %.6f | h_so_j_k = %.1f | h_so_j = %.1f \n",
						   k * pars.dt, d_0j, h_so_j_k, h_so_j); */
			}
			return h_so_j;
		}

		/****************************************************************************************
			Private functions
		****************************************************************************************/
		/****************************************************************************************
		*  Name     : find_triplet_orientation
		*  Function : Find orientation of ordered triplet (p, q, r)
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ int MPC_Cost<Parameters>::find_triplet_orientation(
			const TML::Vector2d &p,
			const TML::Vector2d &q,
			const TML::Vector2d &r)
		{
			epsilon = 1e-8; // abs(val) less than 1e-8 m^2 is considered zero for this check
			// Calculate z-component of cross product (q - p) x (r - q)
			val = (q(0) - p(0)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(0) - q(0));

			//printf("p = %.6f, %.6f | q = %.6f, %.6f | r = %.6f, %.6f | val = %.15f\n", p(0), p(1), q(0), q(1), r(0), r(1), val);
			if (val > -epsilon && val < epsilon)
			{
				return 0;
			} // colinear
			else if (val > epsilon)
			{
				return 1;
			} // clockwise
			else
			{
				return 2;
			} // counterclockwise
		}

		/****************************************************************************************
		*  Name     : determine_if_on_segment
		*  Function : Determine if the point q is on the segment pr
		*			  (really if q is inside the rectangle with diagonal pr...)
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ bool MPC_Cost<Parameters>::determine_if_on_segment(
			const TML::Vector2d &p,
			const TML::Vector2d &q,
			const TML::Vector2d &r) const
		{
			if (q(0) <= fmax(p(0), r(0)) && q(0) >= fmin(p(0), r(0)) &&
				q(1) <= fmax(p(1), r(1)) && q(1) >= fmin(p(1), r(1)))
			{
				return true;
			}
			return false;
		}

		/****************************************************************************************
		*  Name     : determine_if_lines_intersect
		*  Function : Determine if the line segments defined by p_1, q_1 and p_2, q_2 intersects
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ bool MPC_Cost<Parameters>::determine_if_lines_intersect(
			const TML::Vector2d &p_1,
			const TML::Vector2d &q_1,
			const TML::Vector2d &p_2,
			const TML::Vector2d &q_2)
		{
			// Find the four orientations needed for general and
			// special cases
			o_1 = find_triplet_orientation(p_1, q_1, p_2);
			o_2 = find_triplet_orientation(p_1, q_1, q_2);
			o_3 = find_triplet_orientation(p_2, q_2, p_1);
			o_4 = find_triplet_orientation(p_2, q_2, q_1);

			//printf("o_1 = %d | o_2 = %d | o_3 = %d | o_4 = %d\n", o_1, o_2, o_3, o_4);
			// General case
			if (o_1 != o_2 && o_3 != o_4)
			{
				return true;
			}

			// Special Cases
			// p_1, q_1 and p_2 are colinear and p_2 lies on segment p_1 -> q_1
			if (o_1 == 0 && determine_if_on_segment(p_1, p_2, q_1))
			{
				return true;
			}

			// p_1, q_1 and q_2 are colinear and q_2 lies on segment p_1 -> q_1
			if (o_2 == 0 && determine_if_on_segment(p_1, q_2, q_1))
			{
				return true;
			}

			// p_2, q_2 and p_1 are colinear and p_1 lies on segment p_2 -> q_2
			if (o_3 == 0 && determine_if_on_segment(p_2, p_1, q_2))
			{
				return true;
			}

			// p_2, q_2 and q_1 are colinear and q_1 lies on segment p_2 -> q_2
			if (o_4 == 0 && determine_if_on_segment(p_2, q_1, q_2))
			{
				return true;
			}

			return false; // Doesn't fall in any of the above cases
		}

		/****************************************************************************************
		*  Name     : determine_if_inside_polygon
		*  Function :
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ bool MPC_Cost<Parameters>::determine_if_inside_polygon(
			const TML::Vector2d &p,
			const Basic_Polygon &poly)
		{
			if (poly.vertices.get_cols() < 3 ||
				p(0) < poly.bbox(0, 0) || p(0) > poly.bbox(0, 1) || p(1) < poly.bbox(1, 0) || p(1) > poly.bbox(1, 1))
			{
				return false;
			}

			line_intersect_count = 0;

			// Pick a point outside the polygon bbox for the ray tracing
			p_ray_end = poly.bbox.get_col(1);
			p_ray_end(0) += 0.01;

			for (size_t l = 0; l < poly.vertices.get_cols(); l++)
			{
				if (l == 0)
				{
					v_prev = poly.vertices.get_col(poly.vertices.get_cols() - 1);
				}
				else
				{
					v_prev = poly.vertices.get_col(l - 1);
				}
				v = poly.vertices.get_col(l);
				if (l == poly.vertices.get_cols() - 1)
				{
					v_next = poly.vertices.get_col(0);
				}
				else
				{
					v_next = poly.vertices.get_col(l + 1);
				}

				if (determine_if_lines_intersect(p, p_ray_end, v, v_next))
				{
					//printf("index = %ld | v = %.6f, %.6f | v_next = %.6f, %.6f\n", l, v(0), v(1), v_next(0), v_next(1));
					// Special case when p is colinear with line segment from v -> v_next
					if (find_triplet_orientation(v, p, v_next) == 0)
					{
						return determine_if_on_segment(v, p, v_next);
					}
					line_intersect_count += 1;

					// Special case when the vertex v is colinear with p -> p_ray_end
					if (find_triplet_orientation(p, v, p_ray_end) == 0)
					{
						// Determine if:
						// 1) both polygon sides connected to v are below/above the ray: 2 intersections
						// 	  => add one to the line intersection count.
						// 2) if one side is below and the other above: 1 intersection
						o_11 = find_triplet_orientation(p, p_ray_end, v_prev);
						o_12 = find_triplet_orientation(p, p_ray_end, v_next);
						if (o_11 == o_12) // Case 1
						{
							line_intersect_count += 1;
						}
						// add one extra to account for the fact that the other side connecting the
						// vertex will also be checked, when (v_next = v).
						line_intersect_count += 1;
					}
				}
			}

			return line_intersect_count % 2 == 1;
		}

		/****************************************************************************************
		*  Name     : distance_to_line_segment
		*  Function : Calculate shortest distance vector from p to the line segment defined
		*			  by q_1 and q_2.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ TML::Vector2f MPC_Cost<Parameters>::distance_to_line_segment(
			const TML::Vector2f &p,
			const TML::Vector2f &q_1,
			const TML::Vector2f &q_2)
		{
			epsilon = 1e-6; // tolerance for the squared distance being zero, would be higher if doubles were considered
			a.set_block<2, 1>(0, 0, q_2 - q_1);
			a(2) = 0.0f;
			b.set_block<2, 1>(0, 0, p - q_1);
			b(2) = 0.0f;

			l_sqrt = a(0) * a(0) + a(1) * a(1);
			if (l_sqrt <= (float)epsilon)
			{
				return q_1 - p;
			}

			t_line = fmaxf(0.0f, fminf(1.0f, a.dot(b) / l_sqrt));
			projection = q_1 + t_line * (q_2 - q_1);

			return projection - p;
		}

		/****************************************************************************************
		*  Name     : distance_vector_to_polygon
		*  Function : Calculate distance vector from p to polygon
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		template <typename Parameters>
		__host__ __device__ TML::Vector2f MPC_Cost<Parameters>::distance_to_polygon(
			const TML::Vector2f &p,
			const Basic_Polygon &poly)
		{
			if (determine_if_inside_polygon(p, poly))
			{
				d2poly(0) = 0.0f;
				d2poly(1) = 0.0f;
				return d2poly;
			}
			d2poly(0) = 1e10f;
			d2poly(1) = 1e10f;
			for (size_t v = 0; v < poly.vertices.get_cols() - 1; v++)
			{
				d2line = distance_to_line_segment(p, poly.vertices.get_col(v), poly.vertices.get_col(v + 1));
				if (d2line.norm() < d2poly.norm())
				{
					d2poly = d2line;
				}
			}
			return d2poly;
		}
	}
}