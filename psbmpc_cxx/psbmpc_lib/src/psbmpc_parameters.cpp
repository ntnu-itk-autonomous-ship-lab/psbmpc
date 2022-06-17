/****************************************************************************************
 *
 *  File name : psbmpc_parameters.cpp
 *
 *  Function  : Class function file for the PSB-MPC parameter class.
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
#include "psbmpc_parameters.hpp"
#include "Eigen/Dense"
#include <vector>

namespace PSBMPC_LIB
{

	/****************************************************************************************
		Public functions
	****************************************************************************************/
	/****************************************************************************************
	 *  Name     : get_<type>par
	 *  Function : Returns parameter with index <index>, "overloaded" for different data types
	 *  Author   : Trym Tengesdal
	 *  Modified :
	 *****************************************************************************************/
	int PSBMPC_Parameters::get_ipar(
		const int index // In: Index of parameter to return (Must be of int type)
	) const
	{
		switch (index)
		{
		case i_ipar_n_M:
			return n_M;
		case i_ipar_n_do_ps:
			return n_do_ps;
		case i_ipar_p_step_opt:
			return p_step_opt;
		case i_ipar_p_step_do:
			return p_step_do;
		case i_ipar_p_step_grounding:
			return p_step_grounding;
		default:
			// Throw
			return 0;
		}
	}

	double PSBMPC_Parameters::get_dpar(
		const int index // In: Index of parameter to return (Must be of double type)
	) const
	{
		switch (index)
		{
		case i_dpar_T:
			return T;
		case i_dpar_dt:
			return dt;
		case i_dpar_t_ts:
			return t_ts;
		case i_dpar_d_safe:
			return d_safe;
		case i_dpar_d_do_relevant:
			return d_do_relevant;
		case i_dpar_d_so_relevant:
			return d_so_relevant;
		case i_dpar_K_coll:
			return K_coll;
		case i_dpar_T_coll:
			return T_coll;
		case i_dpar_kappa_SO:
			return kappa_SO;
		case i_dpar_kappa_GW:
			return kappa_GW;
		case i_dpar_kappa_RA:
			return kappa_RA;
		case i_dpar_K_u:
			return K_u;
		case i_dpar_K_du:
			return K_du;
		case i_dpar_K_chi:
			return K_chi;
		case i_dpar_K_dchi:
			return K_dchi;
		case i_dpar_K_e:
			return K_e;
		case i_dpar_G_1:
			return G_1;
		case i_dpar_G_2:
			return G_2;
		case i_dpar_G_3:
			return G_3;
		case i_dpar_G_4:
			return G_4;
		case i_dpar_epsilon_rdp:
			return epsilon_rdp;
		default:
			// Throw
			return 0.0;
		}
	}

	std::vector<Eigen::VectorXd> PSBMPC_Parameters::get_opar(
		const int index // In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
	{
		switch (index)
		{
		case i_opar_u_offsets:
			return u_offsets;
		case i_opar_chi_offsets:
			return chi_offsets;
		default:
		{
			// Throw
			std::vector<Eigen::VectorXd> bs;
			return bs;
		}
		}
	}

	/****************************************************************************************
	 *  Name     : set_par
	 *  Function : Sets parameter with index <index> to value <value>, given that it is inside
	 *			  valid limits. Overloaded for different data types
	 *  Author   : Trym Tengesdal
	 *  Modified :
	 *****************************************************************************************/
	void PSBMPC_Parameters::set_par(
		const int index, // In: Index of parameter to set
		const int value	 // In: Value to set for parameter
	)
	{
		if (value >= ipar_low[index] && value <= ipar_high[index])
		{
			switch (index)
			{
			case i_ipar_n_M:
				n_M = value;
				break; // Should here resize offset matrices to make this change legal
			case i_ipar_n_do_ps:
				n_do_ps = value;
				break;
			case i_ipar_p_step_opt:
				p_step_opt = value;
				break;
			case i_ipar_p_step_do:
				p_step_do = value;
				break;
			case i_ipar_p_step_grounding:
				p_step_grounding = value;
				break;
			default:
				// Throw
				break;
			}
		}
		else
		{
			// Throw invalid par value
		}
	}

	void PSBMPC_Parameters::set_par(
		const int index,   // In: Index of parameter to set
		const double value // In: Value to set for parameter
	)
	{
		if (value >= dpar_low[index] && value <= dpar_high[index])
		{
			switch (index)
			{
			case i_dpar_T:
				T = value;
				break;
			case i_dpar_dt:
				dt = value;
				break;
			case i_dpar_t_ts:
				t_ts = value;
				break;
			case i_dpar_d_safe:
				// Limits on d_do_relevant depend on d_safe
				d_safe = value;
				dpar_low[i_dpar_d_do_relevant] = d_safe;
				break;
			case i_dpar_d_do_relevant:
				d_do_relevant = value;
				break;
			case i_dpar_d_so_relevant:
				d_so_relevant = value;
				break;
			case i_dpar_K_coll:
				K_coll = value;
				break;
			case i_dpar_T_coll:
				T_coll = value;
				break;
			case i_dpar_kappa_SO:
				kappa_SO = value;
				break;
			case i_dpar_kappa_GW:
				kappa_GW = value;
				break;
			case i_dpar_kappa_RA:
				kappa_RA = value;
				break;
			case i_dpar_K_u:
				K_u = value;
				break;
			case i_dpar_K_du:
				K_du = value;
				break;
			case i_dpar_K_chi:
				K_chi = value;
				break;
			case i_dpar_K_dchi:
				K_dchi = value;
				break;
			case i_dpar_K_e:
				K_e = value;
				break;
			case i_dpar_G_1:
				G_1 = value;
				break;
			case i_dpar_G_2:
				G_2 = value;
				break;
			case i_dpar_G_3:
				G_3 = value;
				break;
			case i_dpar_G_4:
				G_4 = value;
				break;
			case i_dpar_epsilon_rdp:
				epsilon_rdp = value;
				break;
			default: // Throw invalid index
				break;
			}
		}
		else
		{
			// Throw invalid par value
		}
	}

	void PSBMPC_Parameters::set_par(
		const int index,						  // In: Index of parameter to set
		const std::vector<Eigen::VectorXd> &value // In: Value to set for parameter
	)
	{
		if ((int)value.size() == n_M)
		{
			switch (index)
			{
			case i_opar_u_offsets:
				for (int j = 0; j < n_M; j++)
				{
					if (value[j].size() > 0)
					{
						u_offsets[j] = value[j];
					}
				}
				break;
			case i_opar_chi_offsets:
				for (int j = 0; j < n_M; j++)
				{
					if (value[j].size() > 0)
					{
						chi_offsets[j] = value[j];
					}
				}
				break;
			default:
				// Throw invalid index
				break;
			}
		}
		else
		{
			// Throw invalid value
		}
	}

	/****************************************************************************************
		Private functions
	****************************************************************************************/
	/****************************************************************************************
	 *  Name     : initialize_par_limits
	 *  Function : Sets initial low and high limits on tuning parameters
	 *  Author   :
	 *  Modified :
	 *****************************************************************************************/
	void PSBMPC_Parameters::initialize_par_limits()
	{
		ipar_low.resize(N_IPAR);
		ipar_high.resize(N_IPAR);
		for (int i = 0; i < N_IPAR; i++)
		{
			ipar_low[i] = 0.0;
			ipar_high[i] = 1e12;
		}
		ipar_low[i_ipar_n_M] = 1;
		ipar_high[i_ipar_n_M] = 10;
		ipar_low[i_ipar_n_do_ps] = 1;
		ipar_high[i_ipar_n_do_ps] = MAX_N_PS;

		// std::cout << "i_par_low = " << ipar_low.transpose() << std::endl;
		// std::cout << "i_par_high = " << ipar_high.transpose() << std::endl;

		dpar_low.resize(N_DPAR);
		dpar_high.resize(N_DPAR);
		for (int i = 0; i < N_DPAR; i++)
		{
			dpar_low[i] = 0.0;
			dpar_high[i] = 1e12;
		}
		dpar_low[i_dpar_T] = 60.0;
		dpar_low[i_dpar_dt] = 0.001;

		dpar_low[i_dpar_d_safe] = 20.0;
		dpar_low[i_dpar_d_do_relevant] = d_safe;

		dpar_high[i_dpar_K_chi] = 3.0;
		dpar_high[i_dpar_K_dchi] = 3.0;

		// std::cout << "d_par_low = " << dpar_low.transpose() << std::endl;
		// std::cout << "d_par_high = " << dpar_high.transpose() << std::endl;
	}

	/****************************************************************************************
	 *  Name     : initialize_pars
	 *  Function : Sets initial values for PSBMPC tuning parameters, two overloads.
	 *  Author   :
	 *  Modified :
	 *****************************************************************************************/
	void PSBMPC_Parameters::initialize_pars()
	{
		n_cbs = 1;
		n_M = 1;
		n_do_ps = 5;

		chi_offsets.resize(n_M);
		u_offsets.resize(n_M);
		for (int M = 0; M < n_M; M++)
		{
			if (M == 0)
			{
				u_offsets[M].resize(3);

				// u_offsets[M] << 1.0;
				//  u_offsets[M] << 1.0, 0.5;
				u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(13);
				// chi_offsets[M] << 0.0;
				// chi_offsets[M] << -30.0, 0.0, 30.0;
				// chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -60.0, -45.0, -30.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 30.0, 45.0, 60.0;
				// chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				// chi_offsets[M] << -60.0, -50.0, -40.0, -30.0, -20.0, -10.0, -5.0, 0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
				chi_offsets[M] *= DEG2RAD;
			}
			else if (M == 1)
			{
				u_offsets[M].resize(2);
				// u_offsets[M] << 1.0;
				u_offsets[M] << 1.0, 0.5;
				// u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(13);
				// chi_offsets[M] << 0.0;
				// chi_offsets[M] << -30.0, 0.0, 30.0;
				// chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
				// chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				// chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
				// chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				chi_offsets[M] << -60.0, -45.0, -30.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 30.0, 45.0, 60.0;
				// chi_offsets[M] << -60.0, -50.0, -40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
				chi_offsets[M] *= DEG2RAD;
			}
			else
			{
				u_offsets[M].resize(1);
				u_offsets[M] << 1.0;
				// u_offsets[M] << 1.0, 0.5;
				// u_offsets[M] << 1.0, 0.5, 0.0;

				chi_offsets[M].resize(7);
				// chi_offsets[M] << 0.0;
				// chi_offsets[M] << -30.0, 0.0, 30.0;
				// chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
				// chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
				chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
				// chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
				// chi_offsets[M] << -60.0, -45.0, -30.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 30.0, 45.0, 60.0;
				// chi_offsets[M] << -60.0, -50.0, -40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
				chi_offsets[M] *= DEG2RAD;
			}
			n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
		}

		cpe_method = CE;
		prediction_method = ERK1;
		guidance_method = LOS;

		T = 220.0; // 220.0
		dt = 5.0;

		p_step_opt = 1;
		p_step_do = 1;
		p_step_grounding = 2;
		if (prediction_method == ERK1)
		{
			dt = 1.0;
			p_step_opt = 10;
		}
		t_ts = 25;

		d_so_relevant = 150;
		d_do_relevant = 500;
		d_safe = 10.0;
		K_coll = 30.0;
		T_coll = 110;
		kappa_SO = 20.0;
		kappa_GW = 25.0;
		kappa_RA = 5.0;
		K_u = 3.3;
		K_du = 0.6;
		K_chi = 1.8;
		K_dchi = 1.3;
		K_e = 0.007;

		G_1 = 100.0;
		G_2 = 5.0;
		G_3 = 1.4;
		G_4 = 0.5;

		epsilon_rdp = 2.5;
	}

	void PSBMPC_Parameters::initialize_pars(
		const std::vector<std::vector<double>> &u_offsets,	 // In: Vector of vectors of surge modifications
		const std::vector<std::vector<double>> &chi_offsets, // In: Vector of vectors of course modifications
		const CPE_Method cpe_method,						 // In: Collision probability estimator type to use
		const Prediction_Method prediction_method,			 // In: Prediction type to use
		const Guidance_Method guidance_method,				 // In: Guidance type to use
		const std::vector<int> &ipars,						 // In: Vector of integer parameters
		const std::vector<double> &dpars					 // In: Vector of double parameters
	)
	{
		assert(ipars.size() == N_IPAR && dpars.size() == N_DPAR);
		n_M = ipars[i_ipar_n_M];
		n_do_ps = ipars[i_ipar_n_do_ps];
		assert((int)u_offsets.size() > 0 && (int)chi_offsets.size() > 0 && n_M > 0 && n_do_ps > 0);

		p_step_opt = ipars[i_ipar_p_step_opt];
		p_step_do = ipars[i_ipar_p_step_do];
		p_step_grounding = ipars[i_ipar_p_step_grounding];

		this->n_cbs = 1;
		this->u_offsets.resize(n_M);
		this->chi_offsets.resize(n_M);
		for (int M = 0; M < n_M; M++)
		{
			assert((int)u_offsets[M].size() > 0 && (int)chi_offsets[M].size() > 0);
			this->u_offsets[M].resize(u_offsets[M].size());
			for (size_t uo = 0; uo < u_offsets[M].size(); uo++)
			{
				this->u_offsets[M](uo) = u_offsets[M][uo];
			}
			this->chi_offsets[M].resize(chi_offsets[M].size());
			for (size_t co = 0; co < chi_offsets[M].size(); co++)
			{
				// Input pars for chi_offsets are in degrees, so must convert to radians
				this->chi_offsets[M](co) = chi_offsets[M][co] * DEG2RAD;
			}
			this->n_cbs *= this->u_offsets[M].size() * this->chi_offsets[M].size();
		}

		this->cpe_method = cpe_method;
		this->prediction_method = prediction_method;
		this->guidance_method = guidance_method;

		T = dpars[i_dpar_T];
		dt = dpars[i_dpar_dt];
		t_ts = dpars[i_dpar_t_ts];

		d_safe = dpars[i_dpar_d_safe];
		d_do_relevant = dpars[i_dpar_d_do_relevant];
		d_so_relevant = dpars[i_dpar_d_so_relevant];

		K_coll = dpars[i_dpar_K_coll];
		T_coll = dpars[i_dpar_T_coll];

		kappa_SO = dpars[i_dpar_kappa_SO];
		kappa_GW = dpars[i_dpar_kappa_GW];
		kappa_RA = dpars[i_dpar_kappa_RA];

		K_u = dpars[i_dpar_K_u];
		K_du = dpars[i_dpar_K_du];
		K_chi = dpars[i_dpar_K_chi];
		K_dchi = dpars[i_dpar_K_dchi];
		K_e = dpars[i_dpar_K_e];

		G_1 = dpars[i_dpar_G_1];
		G_2 = dpars[i_dpar_G_2];
		G_3 = dpars[i_dpar_G_3];
		G_4 = dpars[i_dpar_G_4];

		epsilon_rdp = dpars[i_dpar_epsilon_rdp];
	}

}