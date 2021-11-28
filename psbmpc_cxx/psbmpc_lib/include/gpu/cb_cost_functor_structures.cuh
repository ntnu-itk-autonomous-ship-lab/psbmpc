/****************************************************************************************
*
*  File name : cb_cost_functor_structures.cuh
*
*  Function  : Header file for the data/parameter structures used in the control behaviour 
*			   cost functor. Used in the thrust framework for GPU calculations.
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

#pragma once

#include "grounding_hazard_manager.hpp"
#include "obstacle_manager.hpp"
#include "kinematic_ship_models_gpu.cuh"
#include "kinetic_ship_models_gpu.cuh"
#include "tml/tml.cuh"

namespace PSBMPC_LIB
{
	namespace GPU
	{
		/****************************************************************************************
		*  Name     : Basic_Polygon
		*  Function : Polygon struct made for use on the GPU for grounding cost calculation
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		struct Basic_Polygon
		{
			TML::PDMatrix<double, 2, MAX_N_VERTICES> vertices;

			TML::Matrix2d bbox; // Bounding box  for the polygon

			__host__ __device__ Basic_Polygon() {}

			// Only transfer outer ring of the polygon to the basic one
			__host__ Basic_Polygon& operator=(const polygon_2D &poly)
			{
				int n_vertices(0), v_count(0);
				for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
				{
					n_vertices += 1;
				}

				bbox(0, 0) = 1e6; bbox(1, 0) = 1e6; bbox(0, 1) = -1e6; bbox(1, 1) = -1e6;
				vertices.resize(2, n_vertices);
				for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); it != boost::end(boost::geometry::exterior_ring(poly)) - 1; it++)
				{
					vertices(0, v_count) = boost::geometry::get<0>(*it); 
					vertices(1, v_count) = boost::geometry::get<1>(*it);

					if (vertices(0, v_count) < bbox(0, 0)) { bbox(0, 0) = vertices(0, v_count); } // x_min
					if (vertices(1, v_count) < bbox(1, 0)) { bbox(1, 0) = vertices(1, v_count); } // y_min
					if (vertices(0, v_count) > bbox(0, 1)) { bbox(0, 1) = vertices(0, v_count); } // x_max
					if (vertices(1, v_count) > bbox(1, 1)) { bbox(1, 1) = vertices(1, v_count); } // y_max
					v_count += 1;
				}
				return *this;
			}
		};
	
		/****************************************************************************************
		*  Name     : CB_Functor_Pars
		*  Function : Struct containing a subset of the PSB-MPC parameters for use in the GPU.
		*			  Read-only data.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		struct CB_Functor_Pars
		{
			int n_M;
			int p_step, p_step_cpe, p_step_grounding;

			CPE_Method cpe_method;

			Prediction_Method prediction_method;

			Guidance_Method guidance_method;

			float T, dt;
			float d_safe, d_init;
			float K_coll;
			float kappa_SO, kappa_GW;
			float K_u, K_du;
			float K_chi_strb, K_dchi_strb;
			float K_chi_port, K_dchi_port; 
			float K_sgn, T_sgn;
			float G_1, G_2, G_3, G_4;

			__host__ __device__ CB_Functor_Pars() {}

			__host__ __device__ CB_Functor_Pars(const PSBMPC_Parameters &pars)
			{
				this->n_M = pars.n_M;

				this->p_step = pars.p_step; this->p_step_cpe = pars.p_step_cpe; this->p_step_grounding = pars.p_step_grounding; 

				this->cpe_method = pars.cpe_method;

				this->prediction_method = pars.prediction_method;

				this->guidance_method = pars.guidance_method;

				this->T = pars.T; this->dt = pars.dt; 

				this->d_safe = pars.d_safe; this->d_init = pars.d_init; 

				this->K_coll = pars.K_coll; 

				this->kappa_SO = pars.kappa_SO; this->kappa_GW = pars.kappa_GW;

				this->K_u = pars.K_u; this->K_du = pars.K_du;

				this->K_chi_strb = pars.K_chi_strb; this->K_dchi_strb = pars.K_dchi_strb;

				this->K_chi_port = pars.K_chi_port; this->K_dchi_port = pars.K_dchi_port;

				this->K_sgn = pars.K_sgn; this->T_sgn = pars.T_sgn;

				this->G_1 = pars.G_1; this->G_2 = pars.G_2; this->G_3 = pars.G_3; this->G_4 = pars.G_4;
			}
		};

		/****************************************************************************************
		*  Name     : CB_Functor_Data
		*  Function : Class containing read-only data/classes/information needed to evaluate 
		*			  the cost of one PSB-MPC control behaviour.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		class CB_Functor_Data
		{
		public:
			float ownship_length;

			TML::PDVector6f ownship_state;

			TML::PDMatrix<float, MAX_N_M, 1> maneuver_times;

			float u_d, chi_d;

			float u_opt_last;
			float chi_opt_last;

			int wp_c_0;

			TML::PDMatrix<float, 2, MAX_N_WPS> waypoints;

			float V_w;
			TML::Vector2f wind_direction;

			int n_do; 
			int n_so;

			// Number of prediction scenarios for each dynamic obstacle
			TML::PDMatrix<int, MAX_N_DO, 1> n_ps;

			//=======================================================================================
			//  Name     : CB_Functor_Data
			//  Function : Class constructor. Transfer miscellaneous relevant data from host to device.
			//  Author   : 
			//  Modified :
			//=======================================================================================
			__host__ CB_Functor_Data() {}

			__host__ CB_Functor_Data(
				const Eigen::MatrixXd &trajectory,
				const Eigen::VectorXd &maneuver_times,
				const double u_opt_last,
				const double chi_opt_last,
				const double u_d, 
				const double chi_d, 
				const int wp_c_0,
				const double ownship_length,
				const Eigen::Matrix<double, 2, -1> &waypoints, 
				const double V_w,
				const Eigen::Vector2d &wind_direction,
				const Static_Obstacles &polygons,
				const Dynamic_Obstacles &obstacles)
			{
				this->ownship_length = ownship_length;
				
				this->wp_c_0 = wp_c_0;

				TML::assign_eigen_object(this->ownship_state, trajectory.col(0));

				TML::assign_eigen_object(this->maneuver_times, maneuver_times);

				this->u_d = u_d;
				this->chi_d = chi_d;

				this->u_opt_last = u_opt_last;
				this->chi_opt_last = chi_opt_last;	

				TML::assign_eigen_object(this->waypoints, waypoints);	

				this->V_w = V_w;
				TML::assign_eigen_object(this->wind_direction, wind_direction);

				this->n_do = obstacles.size();
				this->n_so = polygons.size();

				this->n_ps.resize(n_do, 1);
				for (int i = 0; i < n_do; i++)
				{
					this->n_ps[i] = obstacles[i].get_trajectories().size();
				} 
			}
		};
	}
}