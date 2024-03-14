/****************************************************************************************
*
*  File name : sbmpc_parameters.hpp
*
*  Function  : Header file for the SB-MPC parameter class.
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

#pragma once

#include "sbmpc_index.hpp"
#include "psbmpc_parameters.hpp"

namespace PSBMPC_LIB
{
	class SBMPC;
	class Obstacle_Predictor;
	class Obstacle_Manager;
	class Grounding_Hazard_Manager;
	namespace CPU
	{
		template <typename Parameters>
		class MPC_Cost;
	}

	class SBMPC_Parameters
	{
	private:
		friend class SBMPC;
		friend class Obstacle_Predictor;
		friend class Obstacle_Manager;
		friend class Grounding_Hazard_Manager;
		friend class CPU::MPC_Cost<SBMPC_Parameters>;

		// Number of control behaviours and sequential maneuvers for the ownship, respectively
		int n_cbs, n_M, n_do_ps;

		int p_step_opt, p_step_grounding;

		// Finite sets of offsets considered to the own-ship surge and course references,
		// for each maneuver in the horizon
		std::vector<Eigen::VectorXd> u_offsets;
		std::vector<Eigen::VectorXd> chi_offsets;

		Prediction_Method prediction_method;

		Guidance_Method guidance_method;

		double T, dt;
		double t_ts;
		double d_safe, d_close, d_do_relevant, d_so_relevant;
		double K_coll;
		double phi_AH, phi_OT, phi_HO, phi_CR;
		double kappa, kappa_TC;
		double K_u, K_du;
		double K_chi_strb, K_dchi_strb;
		double K_chi_port, K_dchi_port;
		double q, p;
		double G_1, G_2, G_3, G_4;
		double epsilon_rdp;

		void initialize_pars(const bool is_obstacle_sbmpc);
		
		void initialize_pars(
			const std::vector<std::vector<double>> &u_offsets,
			const std::vector<std::vector<double>> &chi_offsets,
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars,
			const std::vector<double> &dpars);

	public:
		SBMPC_Parameters() {
			initialize_pars(false);
		}

		SBMPC_Parameters(const bool is_obstacle_sbmpc) { initialize_pars(is_obstacle_sbmpc); }
		SBMPC_Parameters(
			const std::vector<std::vector<double>> &u_offsets,
			const std::vector<std::vector<double>> &chi_offsets,
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars,
			const std::vector<double> &dpars)
		{
			initialize_pars(u_offsets, chi_offsets, prediction_method, guidance_method, ipars, dpars);
		}

		inline void set_prediction_method(const Prediction_Method prediction_method)
		{
			if (prediction_method >= Linear && prediction_method <= ERK4)
				this->prediction_method = prediction_method;
		};

		inline void set_guidance_method(const Guidance_Method guidance_method)
		{
			if (guidance_method >= LOS && guidance_method <= CH)
				this->guidance_method = guidance_method;
		};

		void set_par(const int index, const int value);

		void set_par(const int index, const double value);

		void set_par(const int index, const std::vector<Eigen::VectorXd> &value);

		int get_ipar(const int index) const;

		double get_dpar(const int index) const;

		std::vector<Eigen::VectorXd> get_opar(const int index) const;

		inline Prediction_Method get_prediction_method() const { return prediction_method; };

		inline Guidance_Method get_guidance_method() const { return guidance_method; };
	};
}