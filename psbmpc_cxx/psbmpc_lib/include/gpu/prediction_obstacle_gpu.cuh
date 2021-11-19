/****************************************************************************************
*
*  File name : prediction_obstacle_gpu.cuh
*
*  Function  : Header file for the predicion obstacle class. Lightweight class  
*			   specifically made for the GPU PSB-MPC predictions with active
*			   obstacle COLAV systems.
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

#pragma once

#include "psbmpc_defines.hpp"
#include "tracked_obstacle.hpp"
#include "tml/tml.cuh"

#include <thrust/device_vector.h>
#include <assert.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Prediction_Obstacle
		{
		private:

			int ID;

			// Obstacle dimension quantifiers, length (l) and width (w)
			float A, B, C, D, l, w;

			float x_offset, y_offset;

			// Intention of this obstacle when behaving intelligently
			Intention a_p;

			// Indicator of whether or not the prediction obstacle breaches COLREGS in
			// the joint prediction currently considered
			bool mu; 

			TML::PDMatrix4f A_CV;

			// State at the current time or predicted time
			TML::Vector4f xs_0;

			// Actual state trajectory from the joint prediction scheme
			// when the obstacle uses its own SB-MPC, used by the PSB-MPC,
			// and the predicted trajectory from the Obstacle_SBMPC
			// at the current predicted time t_k
			TML::PDMatrix<float, 4, MAX_N_SAMPLES> xs_p, xs_k_p;

			// In use when using the Obstacle_SBMPC
			TML::PDMatrix<float, 2, MAX_N_WPS> waypoints; 

			__host__ __device__ void assign_data(const Prediction_Obstacle &po);
			__host__ __device__ void assign_data(const Tracked_Obstacle &to);
			
		public:

			__host__ __device__ Prediction_Obstacle();

			__host__ __device__ Prediction_Obstacle(const TML::PDMatrix<float, 9, 1> &xs_aug, const double T, const double dt);

			__host__ __device__ Prediction_Obstacle(const Prediction_Obstacle &po);
			__host__ __device__ Prediction_Obstacle(const Tracked_Obstacle &to);

			__host__ __device__ ~Prediction_Obstacle();

			__host__ __device__ Prediction_Obstacle& operator=(const Prediction_Obstacle &rhs);
			__host__ __device__ Prediction_Obstacle& operator=(const Tracked_Obstacle &rhs);

			__host__ __device__ int get_ID() const { return ID; };

			__host__ __device__ float get_length() const { return l; };

			__host__ __device__ float get_width() const { return w; };

			__host__ __device__ inline Intention get_intention() const { return a_p; }

			__host__ __device__ inline bool get_COLREGS_breach_indicator() const { return mu; }

			__host__ __device__ inline TML::Vector4f get_initial_state() const { return xs_0; }; 

			__host__ __device__ inline TML::PDMatrix<float, 4, MAX_N_SAMPLES> get_trajectory() const { return xs_p; };
			__host__ __device__ inline TML::PDMatrix<float, 4, MAX_N_SAMPLES> get_predicted_trajectory() const { return xs_k_p; };

			__host__ __device__ inline TML::Vector4f get_trajectory_sample(const int k) const { assert(k < xs_p.get_cols() && k >= 0); return xs_p.get_col(k); }
			__host__ __device__ inline TML::Vector4f get_predicted_trajectory_sample(const int k_p) const { assert(k_p < xs_k_p.get_cols() && k_p >= 0); return xs_k_p.get_col(k_p); }

			__host__ __device__ inline TML::PDMatrix<float, 2, MAX_N_WPS> get_waypoints() const { return waypoints; }

			__host__ __device__ inline void set_intention(const Intention a) {assert(a >= KCC && a <= PM); a_p = a; }

			__host__ __device__ inline void set_trajectory_sample(const TML::Vector4f &xs_k, const int k) { assert(k < xs_p.get_cols() && k >= 0); xs_p.set_col(k, xs_k); }

			__host__ __device__ inline void set_waypoints(const TML::PDMatrix<float, 2, MAX_N_WPS> &waypoints) { this->waypoints = waypoints; }

			__host__ __device__ void predict_independent_trajectory(const double T, const double dt, const int k);

			__host__ __device__ void update(const TML::Vector4f &xs, const int k);
		};
	}
}