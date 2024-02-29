/****************************************************************************************
*
*  File name : sbmpc.cpp
*
*  Function  : Class functions for the original Scenario-based Model Predictive Control
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

#include "cpu/utilities_cpu.hpp"
#include "sbmpc.hpp"

#include <iostream>
namespace PSBMPC_LIB
{
	/****************************************************************************************
	*  Name     : SBMPC
	*  Function : Class constructor, initializes parameters and variables
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	SBMPC::SBMPC()
		: pars(false)
	{
		offset_sequence_counter.resize(2 * pars.n_M);
		offset_sequence.resize(2 * pars.n_M);
		maneuver_times.resize(pars.n_M);

		mpc_cost = CPU::MPC_Cost<SBMPC_Parameters>(pars);

		chi_opt_last = 0.0;
		u_opt_last = 1.0;
	}

	SBMPC::SBMPC(
		const CPU::Ownship &ownship, // In: Own-ship with specific parameter set
		const SBMPC_Parameters &pars // In: Parameter object to initialize the SB-MPC
		)
		: ownship(ownship), pars(pars), mpc_cost(pars)
	{
		offset_sequence_counter.resize(2 * pars.n_M);
		offset_sequence.resize(2 * pars.n_M);
		maneuver_times.resize(pars.n_M);

		chi_opt_last = 0.0;
		u_opt_last = 1.0;
	}

	// Pybind11 compatibility overload
	SBMPC::SBMPC(
		const std::shared_ptr<CPU::Kinematic_Ship> &ownship_ptr, // In/out from/to Python: Own-ship with specific parameter set
		const SBMPC_Parameters &pars                             // In: Parameter object to initialize the SB-MPC
		)
		: u_opt_last(1.0), chi_opt_last(0.0), pars(pars), mpc_cost(pars)
	{
		this->ownship_ptr = ownship_ptr;
		offset_sequence_counter.resize(2 * pars.n_M);
		offset_sequence.resize(2 * pars.n_M);
		maneuver_times.resize(pars.n_M);
	}

	/****************************************************************************************
	*  Name     : calculate_optimal_offsets
	*  Function :
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	void SBMPC::calculate_optimal_offsets(
		double &u_opt,										  // In/out: Optimal surge offset
		double &chi_opt,									  // In/out: Optimal course offset
		Eigen::MatrixXd &predicted_trajectory,				  // In/out: Predicted optimal ownship trajectory
		const double u_d,									  // In: Surge reference
		const double chi_d,									  // In: Course reference
		const Eigen::Matrix<double, 2, -1> &waypoints,		  // In: Next waypoints
		const Eigen::VectorXd &ownship_state,				  // In: Current ship state
		const Eigen::Matrix<double, 4, -1> &static_obstacles, // In: Static obstacle information parameterized as no-go lines
		const Dynamic_Obstacles &obstacles,					  // In: Dynamic obstacle information
		const bool disable									  // In: Disable the COLAV functionality or not
	)
	{
		int n_samples = std::round(pars.T / pars.dt);

		trajectory.resize(ownship_state.size(), n_samples);
		trajectory.col(0) = ownship_state;

		ownship.determine_active_waypoint_segment(waypoints, ownship_state);

		update_transitional_variables(ownship_state, obstacles);

		int n_do = obstacles.size();
		int n_so = static_obstacles.cols();

		Eigen::VectorXd opt_offset_sequence(2 * pars.n_M);

		// Predict nominal trajectory first, assign as optimal if no need for
		// COLAV, or use in the prediction initialization
		for (int M = 0; M < pars.n_M; M++)
		{
			offset_sequence(2 * M) = 1.0;
			offset_sequence(2 * M + 1) = 0.0;
		}
		maneuver_times.setZero();
		ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

		bool colav_active = determine_colav_active(obstacles, n_so, disable);
		if (!colav_active)
		{
			u_opt = 1.0;
			u_opt_last = u_opt;
			chi_opt = 0.0;
			chi_opt_last = chi_opt;

			assign_optimal_trajectory(predicted_trajectory);

			return;
		}

		setup_prediction(obstacles);

		double cost;
		Eigen::VectorXd cost_i(n_do);
		min_cost = 1e12;
		reset_control_behaviour();
		for (int cb = 0; cb < pars.n_cbs; cb++)
		{
			cost = 0.0;

			ownship.predict_trajectory(
				trajectory,
				offset_sequence,
				maneuver_times,
				u_d, chi_d,
				waypoints,
				pars.prediction_method,
				pars.guidance_method,
				pars.T,
				pars.dt);

			for (int i = 0; i < n_do; i++)
			{
				cost_i(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, offset_sequence, maneuver_times, obstacles, tv, i, ownship.get_length());
			}

			cost += cost_i.maxCoeff();

			//cost += mpc_cost.calculate_grounding_cost(trajectory, static_obstacles, ownship.get_length());

			cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);

			if (cost < min_cost)
			{
				min_cost = cost;
				opt_offset_sequence = offset_sequence;

				assign_optimal_trajectory(predicted_trajectory);
			}
			increment_control_behaviour();
		}

		u_opt = opt_offset_sequence(0);
		u_opt_last = u_opt;
		chi_opt = opt_offset_sequence(1);
		chi_opt_last = chi_opt;

		if (u_opt == 0)
		{
			chi_opt = 0;
			chi_opt_last = chi_opt;
		}

		/* std::cout << "Optimal offset sequence : ";
	for (int M = 0; M < pars.n_M; M++)
	{
		std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		if (M < pars.n_M - 1) std::cout << ", ";
	}
	std::cout << std::endl;

	std::cout << "Cost at optimum : " << min_cost << std::endl; */
	}

	void SBMPC::calculate_optimal_offsets(
		double &u_opt,								   // In/out: Optimal surge offset
		double &chi_opt,							   // In/out: Optimal course offset
		Eigen::MatrixXd &predicted_trajectory,		   // In/out: Predicted optimal ownship trajectory
		const double u_d,							   // In: Surge reference
		const double chi_d,							   // In: Course reference
		const Eigen::Matrix<double, 2, -1> &waypoints, // In: Next waypoints
		const Eigen::VectorXd &ownship_state,		   // In: Current ship state
		const double V_w,							   // In: Estimated wind speed
		const Eigen::Vector2d &wind_direction,		   // In: Unit vector in NE describing the estimated wind direction
		const Static_Obstacles &polygons,			   // In: Static obstacles parameterized as polygons
		const Dynamic_Obstacles &obstacles,			   // In: Dynamic obstacle information
		const bool disable							   // In: Disable the COLAV functionality or not
	)
	{
		int n_samples = std::round(pars.T / pars.dt);

		trajectory.resize(ownship_state.size(), n_samples);
		trajectory.col(0) = ownship_state;

		ownship.determine_active_waypoint_segment(waypoints, ownship_state);

		update_transitional_variables(ownship_state, obstacles);

		int n_do = obstacles.size();
		int n_so = polygons.size();

		Eigen::VectorXd opt_offset_sequence(2 * pars.n_M);

		// Predict nominal trajectory first, assign as optimal if no need for
		// COLAV, or use in the prediction initialization
		for (int M = 0; M < pars.n_M; M++)
		{
			offset_sequence(2 * M) = 1.0;
			offset_sequence(2 * M + 1) = 0.0;
		}
		maneuver_times.setZero();
		ownship.predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

		bool colav_active = determine_colav_active(obstacles, n_so, disable);
		if (!colav_active)
		{
			u_opt = 1.0;
			u_opt_last = u_opt;
			chi_opt = 0.0;
			chi_opt_last = chi_opt;

			assign_optimal_trajectory(predicted_trajectory);

			return;
		}

		setup_prediction(obstacles);

		double cost;
		Eigen::VectorXd cost_i(n_do);
		min_cost = 1e12;
		reset_control_behaviour();
		for (int cb = 0; cb < pars.n_cbs; cb++)
		{
			cost = 0.0;

			ownship.predict_trajectory(
				trajectory,
				offset_sequence,
				maneuver_times,
				u_d, chi_d,
				waypoints,
				pars.prediction_method,
				pars.guidance_method,
				pars.T,
				pars.dt);

			for (int i = 0; i < n_do; i++)
			{
				cost_i(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, offset_sequence, maneuver_times, obstacles, tv, i, ownship.get_length());
			}

			cost += cost_i.maxCoeff();

			cost += mpc_cost.calculate_grounding_cost(trajectory, polygons, V_w, wind_direction);

			cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);

			if (cost < min_cost)
			{
				min_cost = cost;
				opt_offset_sequence = offset_sequence;

				assign_optimal_trajectory(predicted_trajectory);
			}
			increment_control_behaviour();
		}

		u_opt = opt_offset_sequence(0);
		u_opt_last = u_opt;
		chi_opt = opt_offset_sequence(1);
		chi_opt_last = chi_opt;

		if (u_opt == 0)
		{
			chi_opt = 0;
			chi_opt_last = chi_opt;
		}

		std::cout << "Optimal offset sequence : ";
		for (int M = 0; M < pars.n_M; M++)
		{
			std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
			if (M < pars.n_M - 1)
				std::cout << ", ";
		}
		std::cout << std::endl;

		//
		std::cout << "Cost at optimum : " << min_cost << std::endl;
	}

	// Pybind11 compatibility overload
	optimal_offsets_results_SBMPC_py SBMPC::calculate_optimal_offsets_py(
		const double u_d,							     // In: Surge reference
		const double chi_d,							     // In: Course reference
		const Eigen::Matrix<double, 2, -1> &waypoints,   // In: Next waypoints
		const Eigen::VectorXd &ownship_state,		     // In: Current ship state
		const double V_w,							     // In: Estimated wind speed
		const Eigen::Vector2d &wind_direction,		     // In: Unit vector in NE describing the estimated wind direction
		const std::vector<Eigen::MatrixXd> &polygons_py, // In: Static obstacles parameterized as polygons
		const Dynamic_Obstacles &obstacles,			     // In: Dynamic obstacle information
		const bool new_static_obstacle_data,             // In: Flag to indicate if new static obstacle data is given
		const bool disable							     // In: Disable the COLAV functionality or not
	)
	{
		// u_opt, chi_opt and predicted_trajectory moved from method inputs, now used in the output instead of void
		double u_opt;                          // Out: Optimal surge offset
		double chi_opt;                        // Out: Optimal course offset
		Eigen::MatrixXd predicted_trajectory;  // Out: Predicted optimal ownship trajectory

		// Making the Python polygons compatible with the C++ codebase's Static_Obstacles
		static Static_Obstacles polygons;
		if (new_static_obstacle_data)
		{
			polygons = process_list_of_np_polygons(polygons_py);
		}

		optimal_offsets_results_SBMPC_py result_py;
		int n_samples = std::round(pars.T / pars.dt);

		trajectory.resize(ownship_state.size(), n_samples);
		trajectory.col(0) = ownship_state;

		ownship_ptr->determine_active_waypoint_segment(waypoints, ownship_state);

		update_transitional_variables_ptr_ver(ownship_state, obstacles);

		int n_do = obstacles.size();
		int n_so = polygons.size();

		Eigen::VectorXd opt_offset_sequence(2 * pars.n_M);

		// Predict nominal trajectory first, assign as optimal if no need for
		// COLAV, or use in the prediction initialization
		for (int M = 0; M < pars.n_M; M++)
		{
			offset_sequence(2 * M) = 1.0;
			offset_sequence(2 * M + 1) = 0.0;
		}
		maneuver_times.setZero();
		ownship_ptr->predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, pars.prediction_method, pars.guidance_method, pars.T, pars.dt);

		bool colav_active = determine_colav_active(obstacles, n_so, disable);
		if (!colav_active)
		{
			u_opt = 1.0;
			u_opt_last = u_opt;
			chi_opt = 0.0;
			chi_opt_last = chi_opt;

			assign_optimal_trajectory(predicted_trajectory);

			// Pybind11 compatability return
			result_py.u_opt_py = u_opt;
			result_py.chi_opt_py = chi_opt;
			result_py.predicted_trajectory_py = predicted_trajectory;

			return result_py;
		}

		setup_prediction_ptr_ver(obstacles);

		double cost;
		Eigen::VectorXd cost_i(n_do);
		min_cost = 1e12;
		reset_control_behaviour();
		for (int cb = 0; cb < pars.n_cbs; cb++)
		{
			cost = 0.0;

			ownship_ptr->predict_trajectory(
				trajectory,
				offset_sequence,
				maneuver_times,
				u_d, chi_d,
				waypoints,
				pars.prediction_method,
				pars.guidance_method,
				pars.T,
				pars.dt);

			for (int i = 0; i < n_do; i++)
			{
				cost_i(i) = mpc_cost.calculate_dynamic_obstacle_cost(trajectory, offset_sequence, maneuver_times, obstacles, tv, i, ownship_ptr->get_length());
			}

			cost += cost_i.maxCoeff();

			cost += mpc_cost.calculate_grounding_cost(trajectory, polygons, V_w, wind_direction);

			cost += mpc_cost.calculate_control_deviation_cost(offset_sequence, u_opt_last, chi_opt_last);

			if (cost < min_cost)
			{
				min_cost = cost;
				opt_offset_sequence = offset_sequence;

				assign_optimal_trajectory(predicted_trajectory);
			}
			increment_control_behaviour();
		}

		u_opt = opt_offset_sequence(0);
		u_opt_last = u_opt;
		chi_opt = opt_offset_sequence(1);
		chi_opt_last = chi_opt;

		if (u_opt == 0)
		{
			chi_opt = 0;
			chi_opt_last = chi_opt;
		}

		//std::cout << "Optimal offset sequence : ";
		//for (int M = 0; M < pars.n_M; M++)
		//{
		//	std::cout << opt_offset_sequence(2 * M) << ", " << opt_offset_sequence(2 * M + 1) * RAD2DEG;
		//	if (M < pars.n_M - 1)
		//		std::cout << ", ";
		/}
		//std::cout << std::endl;

		//std::cout << "Cost at optimum : " << min_cost << std::endl;

		// Pybind11 compatability return
		result_py.u_opt_py = u_opt;
		result_py.chi_opt_py = chi_opt;
		result_py.predicted_trajectory_py = predicted_trajectory;

		return result_py;
	}

	/****************************************************************************************
		Private functions
	****************************************************************************************/
	/****************************************************************************************
	*  Name     : reset_control_behavior
	*  Function : Sets the offset sequence back to the initial starting point, i.e. the
	*			  leftmost branch of the control behavior tree
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	void SBMPC::reset_control_behaviour()
	{
		offset_sequence_counter.setZero();
		for (int M = 0; M < pars.n_M; M++)
		{
			offset_sequence(2 * M) = pars.u_offsets[M](0);
			offset_sequence(2 * M + 1) = pars.chi_offsets[M](0);
		}
	}

	/****************************************************************************************
	*  Name     : increment_control_behavior
	*  Function : Increments the control behavior counter and changes the offset sequence
	*			  accordingly. Backpropagation is used for the incrementation
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	void SBMPC::increment_control_behaviour()
	{
		for (int M = pars.n_M - 1; M > -1; M--)
		{
			// Only increment counter for "leaf node offsets" on each iteration, which are the
			// course offsets in the last maneuver
			if (M == pars.n_M - 1)
			{
				offset_sequence_counter(2 * M + 1) += 1;
			}

			// If one reaches the end of maneuver M's course offsets, reset corresponding
			// counter and increment surge offset counter above
			if (offset_sequence_counter(2 * M + 1) == pars.chi_offsets[M].size())
			{
				offset_sequence_counter(2 * M + 1) = 0;
				offset_sequence_counter(2 * M) += 1;
			}
			offset_sequence(2 * M + 1) = pars.chi_offsets[M](offset_sequence_counter(2 * M + 1));

			// If one reaches the end of maneuver M's surge offsets, reset corresponding
			// counter and increment course offset counter above (if any)
			if (offset_sequence_counter(2 * M) == pars.u_offsets[M].size())
			{
				offset_sequence_counter(2 * M) = 0;
				if (M > 0)
				{
					offset_sequence_counter(2 * M - 1) += 1;
				}
			}
			offset_sequence(2 * M) = pars.u_offsets[M](offset_sequence_counter(2 * M));
		}
	}

	/****************************************************************************************
	*  Name     : update_transitional_variables
	*  Function : Updates the transitional cost indicators O, Q, X, S
	*			  at the current time t0 wrt all obstacles.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	void SBMPC::update_transitional_variables(
		const Eigen::VectorXd &ownship_state, // In: Current time own-ship state
		const Dynamic_Obstacles &obstacles	  // In: Dynamic obstacle information
	)
	{
		bool is_close;

		// A : Own-ship, B : Obstacle i
		Eigen::Vector2d v_A, v_B, L_AB;
		double psi_A, psi_B, d_AB;
		if (ownship_state.size() == 4)
		{
			v_A(0) = ownship_state(3) * cos(ownship_state(2));
			v_A(1) = ownship_state(3) * sin(ownship_state(2));
			psi_A = ownship_state(2);
		}
		else
		{
			v_A(0) = ownship_state(3);
			v_A(1) = ownship_state(4);
			psi_A = ownship_state(2);
			v_A = CPU::rotate_vector_2D(v_A, psi_A);
		}

		int n_do = obstacles.size();

		tv.AH_0.resize(n_do);
		tv.S_TC_0.resize(n_do);
		tv.S_i_TC_0.resize(n_do);
		tv.O_TC_0.resize(n_do);
		tv.Q_TC_0.resize(n_do);
		tv.IP_0.resize(n_do);
		tv.H_TC_0.resize(n_do);
		tv.X_TC_0.resize(n_do);

		for (int i = 0; i < n_do; i++)
		{
			v_B(0) = obstacles[i].kf.get_state()(2);
			v_B(1) = obstacles[i].kf.get_state()(3);
			psi_B = atan2(v_B(1), v_B(0));

			L_AB(0) = obstacles[i].kf.get_state()(0) - ownship_state(0);
			L_AB(1) = obstacles[i].kf.get_state()(1) - ownship_state(1);
			d_AB = L_AB.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_AB = d_AB - 0.5 * (ownship.get_length() + obstacles[i].get_length());

			L_AB = L_AB.normalized();

			/*********************************************************************
			* Transitional variable update
			*********************************************************************/
			is_close = d_AB <= pars.d_close;

			tv.AH_0[i] = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

			//std::cout << "Obst i = " << i << " ahead at t0 ? " << tv.AH_0[i] << std::endl;

			// Obstacle on starboard side
			tv.S_TC_0[i] = CPU::angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << tv.S_TC_0[i] << std::endl;

			// Ownship on starboard side of obstacle
			tv.S_i_TC_0[i] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			//std::cout << "Own-ship on starboard side of obst i = " << i << " at t0 ? " << tv.S_i_TC_0[i] << std::endl;

			// Ownship overtaking the obstacle
			tv.O_TC_0[i] = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() &&
						   v_B.norm() < v_A.norm() &&
						   v_B.norm() > 0.25 &&
						   is_close &&
						   tv.AH_0[i];

			//std::cout << "Own-ship overtaking obst i = " << i << " at t0 ? " << tv.O_TC_0[i] << std::endl;

			// Obstacle overtaking the ownship
			tv.Q_TC_0[i] = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() &&
						   v_A.norm() < v_B.norm() &&
						   v_A.norm() > 0.25 &&
						   is_close &&
						   !tv.AH_0[i];

			//std::cout << "Obst i = " << i << " overtaking the ownship at t0 ? " << tv.Q_TC_0[i] << std::endl;

			// Determine if the obstacle is passed by
			tv.IP_0[i] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm() && // Ownship's perspective
						   !tv.Q_TC_0[i]) ||
						  (v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() && // Obstacle's perspective
						   !tv.O_TC_0[i])) &&
						 d_AB > pars.d_safe;

			//std::cout << "Obst i = " << i << " passed by at t0 ? " << tv.IP_0[i] << std::endl;

			// This is not mentioned in article, but also implemented here..
			tv.H_TC_0[i] = v_A.dot(v_B) < -cos(pars.phi_HO) * v_A.norm() * v_B.norm() &&
						   v_A.norm() > 0.25 &&
						   v_B.norm() > 0.25 &&
						   tv.AH_0[i];

			//std::cout << "Head-on at t0 wrt obst i = " << i << " ? " << tv.H_TC_0[i] << std::endl;

			// Crossing situation, a bit redundant with the !is_passed condition also,
			// but better safe than sorry (could be replaced with B_is_ahead also)
			tv.X_TC_0[i] = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm() &&
						   !tv.H_TC_0[i] &&
						   !tv.IP_0[i] &&
						   v_A.norm() > 0.25 &&
						   v_B.norm() > 0.25;

			//std::cout << "Crossing at t0 wrt obst i = " << i << " ? " << tv.X_TC_0[i] << std::endl;
		}
	}

	// Pybind11 compatibility overload
	void SBMPC::update_transitional_variables_ptr_ver(
		const Eigen::VectorXd &ownship_state, // In: Current time own-ship state
		const Dynamic_Obstacles &obstacles	  // In: Dynamic obstacle information
	)
	{
		bool is_close;

		// A : Own-ship, B : Obstacle i
		Eigen::Vector2d v_A, v_B, L_AB;
		double psi_A, psi_B, d_AB;
		if (ownship_state.size() == 4)
		{
			v_A(0) = ownship_state(3) * cos(ownship_state(2));
			v_A(1) = ownship_state(3) * sin(ownship_state(2));
			psi_A = ownship_state(2);
		}
		else
		{
			v_A(0) = ownship_state(3);
			v_A(1) = ownship_state(4);
			psi_A = ownship_state(2);
			v_A = CPU::rotate_vector_2D(v_A, psi_A);
		}

		int n_do = obstacles.size();

		tv.AH_0.resize(n_do);
		tv.S_TC_0.resize(n_do);
		tv.S_i_TC_0.resize(n_do);
		tv.O_TC_0.resize(n_do);
		tv.Q_TC_0.resize(n_do);
		tv.IP_0.resize(n_do);
		tv.H_TC_0.resize(n_do);
		tv.X_TC_0.resize(n_do);

		for (int i = 0; i < n_do; i++)
		{
			v_B(0) = obstacles[i].kf.get_state()(2);
			v_B(1) = obstacles[i].kf.get_state()(3);
			psi_B = atan2(v_B(1), v_B(0));

			L_AB(0) = obstacles[i].kf.get_state()(0) - ownship_state(0);
			L_AB(1) = obstacles[i].kf.get_state()(1) - ownship_state(1);
			d_AB = L_AB.norm();

			// Decrease the distance between the vessels by their respective max dimension
			d_AB = d_AB - 0.5 * (ownship_ptr->get_length() + obstacles[i].get_length());

			L_AB = L_AB.normalized();

			/*********************************************************************
			* Transitional variable update
			*********************************************************************/
			is_close = d_AB <= pars.d_close;

			tv.AH_0[i] = v_A.dot(L_AB) > cos(pars.phi_AH) * v_A.norm();

			//std::cout << "Obst i = " << i << " ahead at t0 ? " << tv.AH_0[i] << std::endl;

			// Obstacle on starboard side
			tv.S_TC_0[i] = CPU::angle_difference_pmpi(atan2(L_AB(1), L_AB(0)), psi_A) > 0;

			//std::cout << "Obst i = " << i << " on starboard side at t0 ? " << tv.S_TC_0[i] << std::endl;

			// Ownship on starboard side of obstacle
			tv.S_i_TC_0[i] = atan2(-L_AB(1), -L_AB(0)) > psi_B;

			//std::cout << "Own-ship on starboard side of obst i = " << i << " at t0 ? " << tv.S_i_TC_0[i] << std::endl;

			// Ownship overtaking the obstacle
			tv.O_TC_0[i] = v_B.dot(v_A) > cos(pars.phi_OT) * v_B.norm() * v_A.norm() &&
						   v_B.norm() < v_A.norm() &&
						   v_B.norm() > 0.25 &&
						   is_close &&
						   tv.AH_0[i];

			//std::cout << "Own-ship overtaking obst i = " << i << " at t0 ? " << tv.O_TC_0[i] << std::endl;

			// Obstacle overtaking the ownship
			tv.Q_TC_0[i] = v_A.dot(v_B) > cos(pars.phi_OT) * v_A.norm() * v_B.norm() &&
						   v_A.norm() < v_B.norm() &&
						   v_A.norm() > 0.25 &&
						   is_close &&
						   !tv.AH_0[i];

			//std::cout << "Obst i = " << i << " overtaking the ownship at t0 ? " << tv.Q_TC_0[i] << std::endl;

			// Determine if the obstacle is passed by
			tv.IP_0[i] = ((v_A.dot(L_AB) < cos(112.5 * DEG2RAD) * v_A.norm() && // Ownship's perspective
						   !tv.Q_TC_0[i]) ||
						  (v_B.dot(-L_AB) < cos(112.5 * DEG2RAD) * v_B.norm() && // Obstacle's perspective
						   !tv.O_TC_0[i])) &&
						 d_AB > pars.d_safe;

			//std::cout << "Obst i = " << i << " passed by at t0 ? " << tv.IP_0[i] << std::endl;

			// This is not mentioned in article, but also implemented here..
			tv.H_TC_0[i] = v_A.dot(v_B) < -cos(pars.phi_HO) * v_A.norm() * v_B.norm() &&
						   v_A.norm() > 0.25 &&
						   v_B.norm() > 0.25 &&
						   tv.AH_0[i];

			//std::cout << "Head-on at t0 wrt obst i = " << i << " ? " << tv.H_TC_0[i] << std::endl;

			// Crossing situation, a bit redundant with the !is_passed condition also,
			// but better safe than sorry (could be replaced with B_is_ahead also)
			tv.X_TC_0[i] = v_A.dot(v_B) < cos(pars.phi_CR) * v_A.norm() * v_B.norm() &&
						   !tv.H_TC_0[i] &&
						   !tv.IP_0[i] &&
						   v_A.norm() > 0.25 &&
						   v_B.norm() > 0.25;

			//std::cout << "Crossing at t0 wrt obst i = " << i << " ? " << tv.X_TC_0[i] << std::endl;
		}
	}

	/****************************************************************************************
	*  Name     : setup_prediction
	*  Function : Sets up the own-ship maneuvering times.
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	void SBMPC::setup_prediction(
		const Dynamic_Obstacles &obstacles // In: Dynamic obstacle information
	)
	{
		int n_do = obstacles.size();

		//***********************************************************************************
		// Own-ship prediction initialization
		//***********************************************************************************
		Eigen::VectorXd t_cpa(n_do), d_cpa(n_do);
		Eigen::Vector2d p_cpa, v_0;

		Eigen::Vector4d xs_0, xs_i_0;
		if (trajectory.rows() == 4)
		{
			v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
			v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
		}
		else
		{
			v_0(0) = trajectory(3, 0);
			v_0(1) = trajectory(4, 0);
			v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
		}
		xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
		xs_0(2) = v_0(0);
		xs_0(3) = v_0(1);

		maneuver_times.resize(pars.n_M);
		// First avoidance maneuver is always at t0
		maneuver_times.setZero();

		double t_cpa_min(0.0), d_safe_i(0.0);
		std::vector<bool> maneuvered_by(n_do);
		int index_closest(-1);
		for (int M = 1; M < pars.n_M; M++)
		{
			// If a predicted collision occurs with the closest obstacle, avoidance maneuver
			// M is taken right after the obstacle possibly maneuvers (modelled to be at t_0 + M * t_ts
			// given that t_cpa > t_ts. If t_cpa < t_ts or n_do = 0, the subsequent maneuver is taken
			// at t_{M-1} + t_ts + 1 anyways (simplification)
			maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);

			// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
			t_cpa_min = 1e10;
			index_closest = -1;
			for (int i = 0; i < n_do; i++)
			{
				xs_i_0 = obstacles[i].kf.get_state();
				CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

				d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[i].get_length());
				// For the current avoidance maneuver, determine which obstacle that should be
				// considered, i.e. the closest obstacle that is not already passed (which means
				// that the previous avoidance maneuver happened before CPA with this obstacle)
				if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min && t_cpa(i) <= pars.T)
				{
					t_cpa_min = t_cpa(i);
					index_closest = i;
				}
			}

			if (index_closest != -1)
			{
				d_safe_i = pars.d_safe + 0.5 * (ownship.get_length() + obstacles[index_closest].get_width());
				// If no predicted collision,  avoidance maneuver M with the closest
				// obstacle (that is not passed) is taken at t_cpa_min
				if (d_cpa(index_closest) > d_safe_i)
				{
					//std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
					maneuvered_by[index_closest] = true;
					maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
				}
			}
		}

		//std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
	}

	// Pybind11 compatibility overload
	void SBMPC::setup_prediction_ptr_ver(
		const Dynamic_Obstacles &obstacles // In: Dynamic obstacle information
	)
	{
		int n_do = obstacles.size();

		//***********************************************************************************
		// Own-ship prediction initialization
		//***********************************************************************************
		Eigen::VectorXd t_cpa(n_do), d_cpa(n_do);
		Eigen::Vector2d p_cpa, v_0;

		Eigen::Vector4d xs_0, xs_i_0;
		if (trajectory.rows() == 4)
		{
			v_0(0) = trajectory(3, 0) * cos(trajectory(2, 0));
			v_0(1) = trajectory(3, 0) * sin(trajectory(2, 0));
		}
		else
		{
			v_0(0) = trajectory(3, 0);
			v_0(1) = trajectory(4, 0);
			v_0 = CPU::rotate_vector_2D(v_0, trajectory(2, 0));
		}
		xs_0.block<2, 1>(0, 0) = trajectory.block<2, 1>(0, 0);
		xs_0(2) = v_0(0);
		xs_0(3) = v_0(1);

		maneuver_times.resize(pars.n_M);
		// First avoidance maneuver is always at t0
		maneuver_times.setZero();

		double t_cpa_min(0.0), d_safe_i(0.0);
		std::vector<bool> maneuvered_by(n_do);
		int index_closest(-1);
		for (int M = 1; M < pars.n_M; M++)
		{
			// If a predicted collision occurs with the closest obstacle, avoidance maneuver
			// M is taken right after the obstacle possibly maneuvers (modelled to be at t_0 + M * t_ts
			// given that t_cpa > t_ts. If t_cpa < t_ts or n_do = 0, the subsequent maneuver is taken
			// at t_{M-1} + t_ts + 1 anyways (simplification)
			maneuver_times(M) = maneuver_times(M - 1) + std::round((pars.t_ts + 1) / pars.dt);

			// Otherwise, find the closest obstacle (wrt t_cpa) that is a possible hazard
			t_cpa_min = 1e10;
			index_closest = -1;
			for (int i = 0; i < n_do; i++)
			{
				xs_i_0 = obstacles[i].kf.get_state();
				CPU::calculate_cpa(p_cpa, t_cpa(i), d_cpa(i), xs_0, xs_i_0);

				d_safe_i = pars.d_safe + 0.5 * (ownship_ptr->get_length() + obstacles[i].get_length());
				// For the current avoidance maneuver, determine which obstacle that should be
				// considered, i.e. the closest obstacle that is not already passed (which means
				// that the previous avoidance maneuver happened before CPA with this obstacle)
				if (!maneuvered_by[i] && maneuver_times(M - 1) * pars.dt < t_cpa(i) && t_cpa(i) <= t_cpa_min && t_cpa(i) <= pars.T)
				{
					t_cpa_min = t_cpa(i);
					index_closest = i;
				}
			}

			if (index_closest != -1)
			{
				d_safe_i = pars.d_safe + 0.5 * (ownship_ptr->get_length() + obstacles[index_closest].get_width());
				// If no predicted collision,  avoidance maneuver M with the closest
				// obstacle (that is not passed) is taken at t_cpa_min
				if (d_cpa(index_closest) > d_safe_i)
				{
					//std::cout << "OS maneuver M = " << M << " at t = " << t_cpa(index_closest) << " wrt obstacle " << index_closest << std::endl;
					maneuvered_by[index_closest] = true;
					maneuver_times(M) = std::round(t_cpa(index_closest) / pars.dt);
				}
			}
		}

		//std::cout << "Ownship maneuver times = " << maneuver_times.transpose() << std::endl;
	}

	/****************************************************************************************
	*  Name     : determine_colav_active
	*  Function : Uses the freshly updated obstacles vector and the number of static
	*			  obstacles to determine whether it is necessary to run the PSBMPC
	*  Author   : Trym Tengesdal
	*  Modified :
	*****************************************************************************************/
	bool SBMPC::determine_colav_active(
		const Dynamic_Obstacles &obstacles, // In: Dynamic obstacle information
		const int n_so,						// In: Number of static obstacles
		const bool disable					// In: Disable the COLAV functionality or not
	)
	{
		if (disable)
		{
			return false;
		}
		Eigen::VectorXd xs = trajectory.col(0);
		bool colav_active = false;
		Eigen::Vector2d d_0i;
		for (size_t i = 0; i < obstacles.size(); i++)
		{
			d_0i(0) = obstacles[i].kf.get_state()(0) - xs(0);
			d_0i(1) = obstacles[i].kf.get_state()(1) - xs(1);
			if (d_0i.norm() < pars.d_do_relevant)
				colav_active = true;

			// If all obstacles are passed, even though inside colav range,
			// then no need for colav
			if (tv.IP_0[i])
			{
				colav_active = false;
			}
			else
			{
				colav_active = true;
			}
		}
		colav_active = colav_active || n_so > 0;

		return colav_active;
	}

	/****************************************************************************************
	*  Name     : assign_optimal_trajectory
	*  Function : Set the optimal trajectory to the current predicted trajectory
	*  Author   :
	*  Modified :
	*****************************************************************************************/
	void SBMPC::assign_optimal_trajectory(
		Eigen::MatrixXd &optimal_trajectory // In/out: Optimal PSB-MPC trajectory
	)
	{
		int n_samples = std::round(pars.T / pars.dt);
		// Set current optimal x-y position trajectory, downsample if linear prediction was not used
		if (false) //(pars.prediction_method > Linear)
		{
			int count = 0;
			optimal_trajectory.resize(trajectory.rows(), n_samples / pars.p_step_opt);
			for (int k = 0; k < n_samples; k += pars.p_step_opt)
			{
				optimal_trajectory.col(count) = trajectory.col(k);
				if (count < std::round(n_samples / pars.p_step_opt) - 1)
					count++;
			}
		}
		else
		{
			optimal_trajectory.resize(trajectory.rows(), n_samples);
			optimal_trajectory = trajectory.block(0, 0, trajectory.rows(), n_samples);
		}
	}

	// Pybind11/colav simulator compatability method
	Static_Obstacles SBMPC::process_list_of_np_polygons(const std::vector<Eigen::MatrixXd>& polygons_py) 
	{
		polygon_2D boost_polygon;
		Static_Obstacles boost_static_obstacles;

		for (const Eigen::MatrixXd& polygon_py : polygons_py)
		{
			for (int i = 0; i < polygon_py.rows() - 1; ++i)
			{
				for (int j = 0; j < polygon_py.cols(); ++j)
				{
					// std::cout << "x: " << polygon_py(i, j) << "y: " << polygon_py(i + 1, j) << std::endl;
					point_2D boost_point;
					boost::geometry::assign_values(boost_point, polygon_py(i, j), polygon_py(i + 1, j));
					boost::geometry::append(boost_polygon, boost_point);
				}
				++i;
			}
			boost_static_obstacles.push_back(boost_polygon);
		}
		// std::cout << "Number of static obstacles: " << boost_static_obstacles.size() << std::endl;
		return boost_static_obstacles;
	}
}