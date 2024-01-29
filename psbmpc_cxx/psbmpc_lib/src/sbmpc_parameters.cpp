/****************************************************************************************
*
*  File name : sbmpc_parameters.cpp
*
*  Function  : Class function file for the SB-MPC parameter class
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

#include "psbmpc_defines.hpp"
#include "sbmpc_parameters.hpp"
#include "Eigen/Dense"
#include <vector>
#include <iostream>

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
	int SBMPC_Parameters::get_ipar(
		const int index // In: Index of parameter to return (Must be of int type)
	) const
	{
		switch (index)
		{
		case i_ipar_n_M_SBMPC:
			return n_M;
		case i_ipar_n_do_ps_SBMPC:
			return n_do_ps;
		case i_ipar_p_step_opt_SBMPC:
			return p_step_opt;
		case i_ipar_p_step_grounding_SBMPC:
			return p_step_grounding;
		default:
			std::cout << "Index " << index << " is not a valid int parameter index." << std::endl;
			return 0;
		}
	}

	double SBMPC_Parameters::get_dpar(
		const int index // In: Index of parameter to return (Must be of double type)
	) const
	{
		switch (index)
		{
		case i_dpar_T_SBMPC:
			return T;
		case i_dpar_dt_SBMPC:
			return dt;
		case i_dpar_t_ts_SBMPC:
			return t_ts;
		case i_dpar_d_safe_SBMPC:
			return d_safe;
		case i_dpar_d_close_SBMPC:
			return d_close;
		case i_dpar_d_do_relevant_SBMPC:
			return d_do_relevant;
		case i_dpar_d_so_relevant_SBMPC:
			return d_so_relevant;
		case i_dpar_K_coll_SBMPC:
			return K_coll;
		case i_dpar_phi_AH_SBMPC:
			return phi_AH * RAD2DEG;
		case i_dpar_phi_OT_SBMPC:
			return phi_OT * RAD2DEG;
		case i_dpar_phi_HO_SBMPC:
			return phi_HO * RAD2DEG;
		case i_dpar_phi_CR_SBMPC:
			return phi_CR * RAD2DEG;
		case i_dpar_kappa_SBMPC:
			return kappa;
		case i_dpar_kappa_TC_SBMPC:
			return kappa_TC;
		case i_dpar_K_u_SBMPC:
			return K_u;
		case i_dpar_K_du_SBMPC:
			return K_du;
		case i_dpar_K_chi_strb_SBMPC:
			return K_chi_strb;
		case i_dpar_K_dchi_strb_SBMPC:
			return K_dchi_strb;
		case i_dpar_K_chi_port_SBMPC:
			return K_chi_port;
		case i_dpar_K_dchi_port_SBMPC:
			return K_dchi_port;
		case i_dpar_K_sgn_SBMPC:
			return K_sgn;
		case i_dpar_T_sgn_SBMPC:
			return T_sgn;
		case i_dpar_q_SBMPC:
			return q;
		case i_dpar_p_SBMPC:
			return p;
		case i_dpar_G_1_SBMPC:
			return G_1;
		case i_dpar_G_2_SBMPC:
			return G_2;
		case i_dpar_G_3_SBMPC:
			return G_3;
		case i_dpar_G_4_SBMPC:
			return G_4;
		case i_dpar_epsilon_rdp_SBMPC:
			return epsilon_rdp;
		default:
			std::cout << "Index " << index << " is not a valid double parameter index." << std::endl;
			return 0.0;
		}
	}

	std::vector<Eigen::VectorXd> SBMPC_Parameters::get_opar(
		const int index // In: Index of parameter to return (Must be of std::vector<Eigen::VectorXd> type)
	) const
	{
		std::vector<Eigen::VectorXd> chi_offsets_deg;
		switch (index)
		{
		case i_opar_u_offsets_SBMPC:
			return u_offsets;
		case i_opar_chi_offsets_SBMPC:
			chi_offsets_deg.resize(chi_offsets.size());
			for (int j = 0; j < n_M; j++)
			{
				if (chi_offsets[j].size() > 0)
				{
					chi_offsets_deg[j] = chi_offsets[j] * RAD2DEG;
				}
			}
			return chi_offsets_deg;
		default:
		{
			std::vector<Eigen::VectorXd> bs;
			std::cout << "Index " << index << " is not a valid vector parameter index." << std::endl;
			return bs;
		}
		}
	}

	/****************************************************************************************
	 *  Name     : set_par
	 *  Function : Sets parameter with index <index> to value <value>, given that it is inside
	 *			  valid limits. Overloaded for different data types
	 *  Author   :
	 *  Modified :
	 *****************************************************************************************/
	void SBMPC_Parameters::set_par(
		const int index, // In: Index of parameter to set
		const int value	 // In: Value to set for parameter
	)
	{
		switch (index)
		{
		case i_ipar_n_M_SBMPC:
			n_M = value;
			break; // Should here resize offset matrices to make this change legal
		case i_ipar_n_do_ps_SBMPC:
			n_do_ps = value;
			break;
		case i_ipar_p_step_opt_SBMPC:
			p_step_opt = value;
			break;
		case i_ipar_p_step_grounding_SBMPC:
			p_step_grounding = value;
			break;
		default:
			std::cout << "Index " << index << " is not a valid int parameter index." << std::endl;
			break;
		}
	}

	void SBMPC_Parameters::set_par(
		const int index,   // In: Index of parameter to set
		const double value // In: Value to set for parameter
	)
	{
		switch (index)
		{
		case i_dpar_T_SBMPC:
			T = value;
			break;
		case i_dpar_dt_SBMPC:
			dt = value;
			break;
		case i_dpar_t_ts_SBMPC:
			t_ts = value;
			break;
		case i_dpar_d_safe_SBMPC:
			d_safe = value;
			break;
		case i_dpar_d_close_SBMPC:
			d_close = value;
			break;
		case i_dpar_d_do_relevant_SBMPC:
			d_do_relevant = value;
			break;
		case i_dpar_d_so_relevant_SBMPC:
			d_so_relevant = value;
			break;
		case i_dpar_K_coll_SBMPC:
			K_coll = value;
			break;
		case i_dpar_phi_AH_SBMPC:
			phi_AH = value * DEG2RAD;
			break;
		case i_dpar_phi_OT_SBMPC:
			phi_OT = value * DEG2RAD;
			break;
		case i_dpar_phi_HO_SBMPC:
			phi_HO = value * DEG2RAD;
			break;
		case i_dpar_phi_CR_SBMPC:
			phi_CR = value * DEG2RAD;
			break;
		case i_dpar_kappa_SBMPC:
			kappa = value;
			break;
		case i_dpar_kappa_TC_SBMPC:
			kappa_TC = value;
			break;
		case i_dpar_K_u_SBMPC:
			K_u = value;
			break;
		case i_dpar_K_du_SBMPC:
			K_du = value;
			break;
		case i_dpar_K_chi_strb_SBMPC:
			K_chi_strb = value;
			break;
		case i_dpar_K_dchi_strb_SBMPC:
			K_dchi_strb = value;
			break;
		case i_dpar_K_chi_port_SBMPC:
			K_chi_port = value;
			break;
		case i_dpar_K_dchi_port_SBMPC:
			K_dchi_port = value;
			break;
		case i_dpar_K_sgn_SBMPC:
			K_sgn = value;
			break;
		case i_dpar_T_sgn_SBMPC:
			T_sgn = value;
			break;
		case i_dpar_q_SBMPC:
			q = value;
			break;
		case i_dpar_p_SBMPC:
			p = value;
			break;
		case i_dpar_G_1_SBMPC:
			G_1 = value;
			break;
		case i_dpar_G_2_SBMPC:
			G_2 = value;
			break;
		case i_dpar_G_3_SBMPC:
			G_3 = value;
			break;
		case i_dpar_G_4_SBMPC:
			G_4 = value;
			break;
		case i_dpar_epsilon_rdp_SBMPC:		
			epsilon_rdp = value;
			break;
		default:
			std::cout << "Index " << index << " is not a valid double parameter index." << std::endl;
			break;
		}
	}

	void SBMPC_Parameters::set_par(
		const int index,						  // In: Index of parameter to set
		const std::vector<Eigen::VectorXd> &value // In: Value to set for parameter
	)
	{
		n_M = (int)value.size(); // Overrides the n_M value (changes if n_M is not set in accordance with value arg)
		chi_offsets.resize(n_M);
		u_offsets.resize(n_M);

		switch (index)
		{
		case i_opar_u_offsets_SBMPC:
			for (int j = 0; j < n_M; j++)
			{
				if (value[j].size() > 0)
				{
					u_offsets[j] = value[j];
				}
			}
			break;
		case i_opar_chi_offsets_SBMPC:
			for (int j = 0; j < n_M; j++)
			{
				if (value[j].size() > 0)
				{
					chi_offsets[j] = value[j]*DEG2RAD;
				}
			}
			break;
		default:
			std::vector<Eigen::VectorXd> bs;
			std::cout << "Index " << index << " is not a valid vector parameter index." << std::endl;
			break;
		}
	}

	/****************************************************************************************
		Private functions
	****************************************************************************************/
	/****************************************************************************************
	*  Name     : initialize_pars
	*  Function : Sets initial values for PSBMPC tuning parameters, two overloads.
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void SBMPC_Parameters::initialize_pars(
		const bool is_obstacle_sbmpc // In: Boolean flag to determine MPC tuning
	)
	{
		if (!is_obstacle_sbmpc) // Tuning for SB-MPC
		{
			n_cbs = 1;
			n_M = 1;

			chi_offsets.resize(n_M);
			u_offsets.resize(n_M);
			for (int M = 0; M < n_M; M++)
			{
				if (M == 0)
				{
					u_offsets[M].resize(3);
					//u_offsets[M] << 1.0;
					u_offsets[M] << 1.0, 0.5, 0.0;

					chi_offsets[M].resize(13);
					//chi_offsets[M] << 30.0;
					//chi_offsets[M] << -30.0, 0.0, 30.0;
					//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
					chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
					chi_offsets[M] *= DEG2RAD;
				}
				else
				{
					u_offsets[M].resize(2);
					//u_offsets[M] << 1.0;
					u_offsets[M] << 1.0, 0.5;
					//u_offsets[M] << 1.0, 0.5, 0.0;

					chi_offsets[M].resize(7);
					//chi_offsets[M] << 0.0;
					//chi_offsets[M] << -30.0, 0.0, 30.0;
					//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
					//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
					chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
					//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
					chi_offsets[M] *= DEG2RAD;
				}
				n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
			}

			prediction_method = ERK1;
			guidance_method = LOS;

			T = 110.0;
			dt = 5.0;

			p_step_opt = 1;
			p_step_grounding = 2;
			if (prediction_method == ERK1)
			{
				dt = 0.5;
				p_step_opt = 10;
			}
			t_ts = 35;

			d_do_relevant = 1500;
			d_close = 1000;
			d_safe = 50;
			K_coll = 0.2;
			phi_AH = 68.5 * DEG2RAD;
			phi_OT = 68.5 * DEG2RAD;
			phi_HO = 22.5 * DEG2RAD;
			phi_CR = 68.5 * DEG2RAD;
			kappa = 10.0;
			kappa_TC = 20.0;
			K_u = 15;
			K_du = 6;
			K_chi_strb = 1.3;
			K_chi_port = 1.6;
			K_dchi_strb = 0.9;
			K_dchi_port = 1.2;
			K_sgn = 8;
			T_sgn = 4 * t_ts;

			G_1 = 100.0;
			G_2 = 5.0;
			G_3 = 0.25;
			G_4 = 0.01;

			epsilon_rdp = 2.0;
			q = 4.0;
			p = 1.0;
		}
		else // Tuning for Obstacle SB-MPC
		{
			n_cbs = 1;
			n_M = 1;

			chi_offsets.resize(n_M);
			u_offsets.resize(n_M);
			for (int M = 0; M < n_M; M++)
			{
				if (M == 0)
				{
					u_offsets[M].resize(3);
					//u_offsets[M] << 1.0;
					u_offsets[M] << 1.0, 0.5, 0.0;

					chi_offsets[M].resize(13);
					//chi_offsets[M] << 30.0;
					//chi_offsets[M] << -30.0, 0.0, 30.0;
					//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
					chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
					chi_offsets[M] *= DEG2RAD;
				}
				else
				{
					u_offsets[M].resize(2);
					//u_offsets[M] << 1.0;
					u_offsets[M] << 1.0, 0.5;
					//u_offsets[M] << 1.0, 0.5, 0.0;

					chi_offsets[M].resize(7);
					//chi_offsets[M] << 0.0;
					//chi_offsets[M] << -30.0, 0.0, 30.0;
					//chi_offsets[M] << -90.0, -45.0, 0.0, 45.0, 90.0;
					//chi_offsets[M] << -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0;
					chi_offsets[M] << -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0;
					//chi_offsets[M] << -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0;
					chi_offsets[M] *= DEG2RAD;
				}
				n_cbs *= u_offsets[M].size() * chi_offsets[M].size();
			}

			prediction_method = ERK1;
			guidance_method = LOS;

			T = 120.0;
			dt = 5.0;

			p_step_opt = 1;
			p_step_grounding = 2;
			if (prediction_method == ERK1)
			{
				dt = 0.5;
				p_step_opt = 10;
			}
			t_ts = 50;

			d_do_relevant = 1500;
			d_close = 1000;
			d_safe = 50;
			K_coll = 2.5;
			phi_AH = 68.5 * DEG2RAD;
			phi_OT = 68.5 * DEG2RAD;
			phi_HO = 22.5 * DEG2RAD;
			phi_CR = 68.5 * DEG2RAD;
			kappa = 8.0;
			kappa_TC = 0.0;
			K_u = 4;
			K_du = 2.5;
			K_chi_strb = 1.3;
			K_chi_port = 1.6;
			K_dchi_strb = 0.9;
			K_dchi_port = 1.2;
			K_sgn = 8;
			T_sgn = 4 * t_ts;

			G_1 = 100.0;
			G_2 = 5.0;
			G_3 = 0.25;
			G_4 = 0.01;

			epsilon_rdp = 2.0;
			q = 4.0;
			p = 1.0;
		}
	}

	void SBMPC_Parameters::initialize_pars(
		const std::vector<std::vector<double>> &u_offsets,	 // In: Vector of vectors of surge modifications
		const std::vector<std::vector<double>> &chi_offsets, // In: Vector of vectors of course modifications
		const Prediction_Method prediction_method,			 // In: Prediction type to use
		const Guidance_Method guidance_method,				 // In: Guidance type to use
		const std::vector<int> &ipars,						 // In: Vector of integer parameters
		const std::vector<double> &dpars					 // In: Vector of double parameters
	)
	{
		n_M = ipars[i_ipar_n_M_SBMPC];
		n_do_ps = ipars[i_ipar_n_do_ps_SBMPC];
		assert((int)u_offsets.size() > 0 && (int)chi_offsets.size() > 0 && n_M > 0 && n_do_ps > 0);

		p_step_opt = ipars[i_ipar_p_step_opt_SBMPC];
		p_step_grounding = ipars[i_ipar_p_step_grounding_SBMPC];

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

		this->prediction_method = prediction_method;
		this->guidance_method = guidance_method;

		T = dpars[i_dpar_T_SBMPC];
		dt = dpars[i_dpar_dt_SBMPC];
		t_ts = dpars[i_dpar_t_ts_SBMPC];

		d_safe = dpars[i_dpar_d_safe_SBMPC];
		d_close = dpars[i_dpar_d_close_SBMPC];
		d_do_relevant = dpars[i_dpar_d_do_relevant_SBMPC];
		d_so_relevant = dpars[i_dpar_d_so_relevant_SBMPC];

		K_coll = dpars[i_dpar_K_coll_SBMPC];

		// Input pars for phi_AH - CR are in degrees, so must convert to radians
		phi_AH = dpars[i_dpar_phi_AH_SBMPC] * DEG2RAD;
		phi_OT = dpars[i_dpar_phi_OT_SBMPC] * DEG2RAD;
		phi_HO = dpars[i_dpar_phi_HO_SBMPC] * DEG2RAD;
		phi_CR = dpars[i_dpar_phi_CR_SBMPC] * DEG2RAD;

		kappa = dpars[i_dpar_kappa_SBMPC];
		kappa_TC = dpars[i_dpar_kappa_TC_SBMPC];

		K_u = dpars[i_dpar_K_u_SBMPC];
		K_du = dpars[i_dpar_K_du_SBMPC];

		K_chi_strb = dpars[i_dpar_K_chi_strb_SBMPC];
		K_dchi_strb = dpars[i_dpar_K_dchi_strb_SBMPC];
		K_chi_port = dpars[i_dpar_K_chi_port_SBMPC];
		K_dchi_port = dpars[i_dpar_K_dchi_port_SBMPC];

		K_sgn = dpars[i_dpar_K_sgn_SBMPC];
		T_sgn = dpars[i_dpar_T_sgn_SBMPC];

		q = dpars[i_dpar_q_SBMPC];
		p = dpars[i_dpar_p_SBMPC];

		G_1 = dpars[i_dpar_G_1_SBMPC];
		G_2 = dpars[i_dpar_G_2_SBMPC];
		G_3 = dpars[i_dpar_G_3_SBMPC];
		G_4 = dpars[i_dpar_G_4_SBMPC];

		epsilon_rdp = dpars[i_dpar_epsilon_rdp_SBMPC];
	}
}