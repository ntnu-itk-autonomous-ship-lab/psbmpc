/****************************************************************************************
*
*  File name : psbmpc_parameters.hpp
*
*  Function  : Header file for the PSB-MPC parameter struct.
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

#include "psbmpc_index.hpp"
#include "Eigen/Dense"
#include <vector>

namespace PSBMPC_LIB
{

	// See "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions" and/or
	// "Collision Probability Estimation for Maritime Collision Avoidance Using the Cross-Entropy Method" for more information on CPE
	enum CPE_Method
	{
		CE,		// Consider positional uncertainty only
		MCSKF4D // Consider uncertainty in both position and velocity along piece-wise linear segments
	};

	enum Prediction_Method
	{
		Linear, // Linear prediction
		ERK1,	// Explicit Runge Kutta 1 = Eulers method
		ERK4	// Explicit Runge Kutta of fourth order, not implemented yet nor needed.
	};

	enum Guidance_Method
	{
		LOS, // Line-of-sight
		WPP, // Waypoint-Pursuit
		CH	 // Course Hold
	};

	template <class T>
	struct CVE_Pars
	{
		T d_init_colregs_situation;		   // Threshold for initializing the COLREGS violation evaluator
		T head_on_width;				   // Width of zone where vessels are considered head-on wrt own-ship
		T overtaking_angle;				   // Bearing angle wrt own-ship/obstacle where vessels are considered to overtake
		T max_acceptable_SO_speed_change;  // Max allowable stand-on speed change before considered violating COLREGS
		T max_acceptable_SO_course_change; // Max allowable stand-on course change before considered violating COLREGS
		T critical_distance_to_ignore_SO;  // Distance where the own-ship SO role should be aborted to make a safety maneuver
		T GW_safety_margin;				   // Minimum distance to keep to other vessel for a COLREGS compliant maneuver to be correct
	};

	enum COLREGS_Situation
	{
		HO,
		OT_ing,
		OT_en,
		CR_PS,
		CR_SS
	};

	enum class SpeedChange
	{
		Lower,
		None,
		Higher
	};

	enum class CourseChange
	{
		Portwards,
		None,
		Starboardwards
	};

	class Grounding_Hazard_Manager;
	class Obstacle_Manager;

	namespace CPU
	{
		class PSBMPC;
		template <typename Parameters>
		class MPC_Cost;
	}
	namespace GPU
	{
		class PSBMPC;
		template <typename Parameters>
		class MPC_Cost;
		class CB_Functor_Pars;
	}

	class PSBMPC_Parameters
	{
	private:
		friend class CPU::PSBMPC;
		friend class GPU::PSBMPC;
		friend class CPU::MPC_Cost<PSBMPC_Parameters>;
		friend class GPU::MPC_Cost<PSBMPC_Parameters>;
		friend class Obstacle_Predictor;
		friend class Obstacle_Manager;
		friend class Grounding_Hazard_Manager;
		friend class GPU::CB_Functor_Pars;

		// Number of control behaviours, sequential maneuvers and maximum allowable
		// prediction scenarios for an obstacle, respectively
		int n_cbs, n_M, n_do_ps;

		// Step between samples in prediction, collision probability estimation and
		// grounding cost evaluation, respectively
		int p_step_opt, p_step_do, p_step_grounding;

		// Finite sets of offsets considered to the own-ship surge and course references,
		// for each maneuver in the horizon
		std::vector<Eigen::VectorXd> u_offsets;
		std::vector<Eigen::VectorXd> chi_offsets;

		Eigen::VectorXd dpar_low, dpar_high;
		Eigen::VectorXd ipar_low, ipar_high;

		CPE_Method cpe_method;

		Prediction_Method prediction_method;

		Guidance_Method guidance_method;

		double T, dt;
		double t_ts;
		double d_safe, d_do_relevant, d_so_relevant;
		double K_coll;
		double kappa_SO, kappa_GW;
		double K_u, K_du;
		double K_chi_strb, K_dchi_strb;
		double K_chi_port, K_dchi_port;
		double K_sgn, T_sgn;
		double G_1, G_2, G_3, G_4;
		double epsilon_rdp;

		void initialize_par_limits();

		void initialize_pars();
		void initialize_pars(
			const std::vector<std::vector<double>> &u_offsets,
			const std::vector<std::vector<double>> &chi_offsets,
			const CPE_Method cpe_method,
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars,
			const std::vector<double> &dpars);

	public:
		PSBMPC_Parameters()
		{
			initialize_pars();
			initialize_par_limits();
		}
		PSBMPC_Parameters(
			const std::vector<std::vector<double>> &u_offsets,
			const std::vector<std::vector<double>> &chi_offsets,
			const CPE_Method cpe_method,
			const Prediction_Method prediction_method,
			const Guidance_Method guidance_method,
			const std::vector<int> &ipars,
			const std::vector<double> &dpars)
		{
			initialize_pars(u_offsets, chi_offsets, cpe_method, prediction_method, guidance_method, ipars, dpars);

			initialize_par_limits();
		}

		void set_par(const int index, const int value);

		void set_par(const int index, const double value);

		void set_par(const int index, const std::vector<Eigen::VectorXd> &value);

		inline void set_cpe_method(const CPE_Method cpe_method)
		{
			if (cpe_method >= CE && cpe_method <= MCSKF4D)
				this->cpe_method = cpe_method;
		};

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

		int get_ipar(const int index) const;

		double get_dpar(const int index) const;

		std::vector<Eigen::VectorXd> get_opar(const int index) const;

		inline CPE_Method get_cpe_method() const { return cpe_method; };

		inline Prediction_Method get_prediction_method() const { return prediction_method; };

		inline Guidance_Method get_guidance_method() const { return guidance_method; };
	};
}
