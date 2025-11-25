#pragma once

#include "psbmpc_defines.hpp"
#include "tracked_obstacle.hpp"
#include "tml/tml.cuh"

#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		class Cuda_Obstacle
		{
		private:
			int ID;

			// Obstacle dimension quantifiers, length (l) and width (w)
			float A, B, C, D, l, w;

			float x_offset, y_offset;

			// State and covariance at the current time or predicted time
			TML::Vector4f xs_0;
			TML::Matrix4f P_0;

			// Probability that the obstacle will give-way for the own-ship when specified by COLREGS
			float Pr_WGW;

			// Probability that the obstacle will perform a COLREGS compliant evasive maneuver when supposed to
			float Pr_CCEM;

			// If the tracker-based KF is on, the obstacle is tracked until it dies
			// while the duration lost may be reset if new measurements are aquired
			float duration_tracked, duration_lost;

			// Predicted covariance for each prediction scenario: n*n x n_samples, i.e. the covariance is flattened for each time step.
			// This is equal for all prediction scenarios including those with active COLAV (using MROU)
			TML::PDMatrix<float, 16, MAX_N_SAMPLES> P_p;

			// Predicted state for each prediction scenario: size n_ps x n x n_samples, where n = 4
			TML::PDMatrix<float, 4 * MAX_N_PS, MAX_N_SAMPLES> xs_p;

			__host__ void assign_data(const Cuda_Obstacle &co);

			__host__ void assign_data(const Tracked_Obstacle &to);

		public:
			// Cuda Obstacle`s are only copy constructed
			__host__ Cuda_Obstacle(){};

			__host__ Cuda_Obstacle(const Cuda_Obstacle &co);

			__host__ Cuda_Obstacle(const Tracked_Obstacle &to);

			__host__ Cuda_Obstacle &operator=(const Cuda_Obstacle &rhs);

			__host__ Cuda_Obstacle &operator=(const Tracked_Obstacle &rhs);

			__host__ __device__ inline int get_ID() const { return this->ID; };

			__host__ __device__ inline float get_length() const { return this->l; };

			__host__ __device__ inline float get_width() const { return this->w; };

			__host__ __device__ inline float get_Pr_WGW() const { return this->Pr_WGW; }

			__host__ __device__ inline float get_Pr_CCEM() const { return this->Pr_CCEM; }

			__host__ __device__ inline float get_duration_lost() const { return this->duration_lost; }

			__host__ __device__ inline float get_duration_tracked() const { return this->duration_tracked; }

			__host__ __device__ inline int get_n_trajectory_samples() const { return this->P_p.get_cols(); }

			__host__ __device__ inline int get_n_prediction_scenarios() const { return this->xs_p.get_rows() / 4; }

			__device__ inline TML::PDVector16f get_trajectory_covariance_sample(const int k) { return this->P_p.get_col(k); }

			__device__ inline TML::PDVector4f get_trajectory_sample(const int ps, const int k) { return this->xs_p.get_block<4, 1>(4 * ps, k, 4, 1); }
		};
	}
}