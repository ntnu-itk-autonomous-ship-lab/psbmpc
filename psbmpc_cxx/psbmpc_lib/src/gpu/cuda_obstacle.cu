/****************************************************************************************
*
*  File name : cuda_obstacle.cu
*
*  Function  : Cuda obstacle class functions. Derived class of base Obstacle class,
*		  	   used in the PSB-MPC GPU CB_Cost_Functor computation.
*
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

#include "gpu/cuda_obstacle.cuh"
#include "gpu/utilities_gpu.cuh"
#include <iostream>

namespace PSBMPC_LIB
{
	namespace GPU
	{
		/****************************************************************************************
		*  Name     : Cuda_Obstacle
		*  Function : Copy constructor. Overloaded for two derived obstacle types.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		__host__ Cuda_Obstacle::Cuda_Obstacle(
			const Cuda_Obstacle &co // In: Obstacle to copy
		)
		{
			assign_data(co);
		}

		__host__ Cuda_Obstacle::Cuda_Obstacle(
			const Tracked_Obstacle &to // In: Obstacle to copy
		)
		{
			assign_data(to);
		}

		/****************************************************************************************
		*  Name     : operator=
		*  Function : Overloaded for two derived obstacle types. NOTE: Should only be used
		*			  when the lhs is uninitialized.
		*  Author   : Trym Tengesdal
		*  Modified :
		*****************************************************************************************/
		__host__ Cuda_Obstacle &Cuda_Obstacle::operator=(
			const Cuda_Obstacle &rhs // In: Rhs to assign
		)
		{
			if (this == &rhs)
			{
				return *this;
			}

			assign_data(rhs);

			return *this;
		}

		__host__ Cuda_Obstacle &Cuda_Obstacle::operator=(
			const Tracked_Obstacle &rhs // In: Rhs Tracked_Obstacle whose data to assign
		)
		{
			assign_data(rhs);

			return *this;
		}

		/****************************************************************************************
		*  Private functions
		*****************************************************************************************/
		/****************************************************************************************
		*  Name     : assign_data
		*  Function :
		*  Author   :
		*  Modified :
		*****************************************************************************************/
		__host__ void Cuda_Obstacle::assign_data(
			const Cuda_Obstacle &co // In: Cuda_Obstacle whose data to assign to *this
		)
		{
			this->ID = co.ID;

			this->A = co.A;
			this->B = co.B;
			this->C = co.C;
			this->D = co.D;

			this->l = co.l;
			this->w = co.w;

			this->x_offset = co.x_offset;
			this->y_offset = co.y_offset;

			this->xs_0 = co.xs_0;
			this->P_0 = co.P_0;

			this->Pr_WGW = co.Pr_WGW;
			this->Pr_CCEM = co.Pr_CCEM;

			this->duration_tracked = co.duration_tracked;
			this->duration_lost = co.duration_lost;

			this->P_p = co.P_p;

			this->xs_p = co.xs_p;
		}

		__host__ void Cuda_Obstacle::assign_data(
			const Tracked_Obstacle &to // In: Tracked_Obstacle ptr whose data to assign to *this
		)
		{
			this->ID = to.ID;

			this->A = to.A;
			this->B = to.B;
			this->C = to.C;
			this->D = to.D;

			this->l = to.l;
			this->w = to.w;

			this->x_offset = to.x_offset;
			this->y_offset = to.y_offset;

			TML::assign_eigen_object(this->xs_0, to.xs_0);
			TML::assign_eigen_object(this->P_0, to.P_0);

			this->duration_tracked = to.duration_tracked;
			this->duration_lost = to.duration_lost;

			TML::assign_eigen_object(this->P_p, to.P_p);

			TML::PDMatrix<float, 4, MAX_N_SAMPLES> xs_p_ps;
			int n_ps_independent = to.xs_p.size();
			for (int ps = 0; ps < n_ps_independent; ps++)
			{
				TML::assign_eigen_object(xs_p_ps, to.xs_p[ps]);

				xs_p.set_block(4 * ps, 0, xs_p_ps.get_rows(), xs_p_ps.get_cols(), xs_p_ps);
			}
		}

	}
}