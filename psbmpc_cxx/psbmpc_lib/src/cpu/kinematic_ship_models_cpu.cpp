#include "cpu/utilities_cpu.hpp"
#include "cpu/kinematic_ship_models_cpu.hpp"
#include <vector>
#include <iostream>

namespace PSBMPC_LIB
{
	namespace CPU
	{
		Kinematic_Ship::Kinematic_Ship()
		{
			l = 5.0; // milliAmpere dims
			w = 3.0;

			T_U = 1.44;
			T_chi = 0.92; // Ad hoc identified time constants for milliAmpere

			/* l = 10.0;
			w = 4.0;

			T_U = 10.0;
			T_chi = 8.0; 		// Ad hoc time constants for a 10m long ship */

			// Guidance parameters
			e_int = 0.0;
			e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance
			R_a = 5.0;					   // WP acceptance radius (5.0 for milliampere, 20.0 for "normal ship")
			LOS_LD = 66.0;				   // LOS lookahead distance (66.0 for milliampere, 200.0 for "normal ship")
			LOS_K_i = 0.0;				   // LOS integral gain (0.0)

			wp_c_0 = 0;
			wp_c_p = 0;

			// Path prediction shape
			path_prediction_shape = SMOOTH;
		}

		Kinematic_Ship::Kinematic_Ship(
			const double l,		 // In: Ship length
			const double w,		 // In: Ship width
			const double T_U,	 // In: Ship first order speed time constant
			const double T_chi,	 // In: Ship first order course time constant
			const double R_a,	 // In: Ship radius of acceptance parameter in WP following
			const double LOS_LD, // In: Ship lookahead distance parameter in LOS WP following
			const double LOS_K_i // In: Ship integral gain parameter in LOS WP following
			) : l(l), w(w), T_U(T_U), T_chi(T_chi), R_a(R_a), LOS_LD(LOS_LD), LOS_K_i(LOS_K_i)
		{

			/* l = 10.0;
			w = 4.0;*/

			// Guidance parameters
			e_int = 0;
			e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance

			wp_c_0 = 0;
			wp_c_p = 0;

			// Path prediction shape
			path_prediction_shape = SMOOTH;
		}

		Kinematic_Ship::Kinematic_Ship(
			const double l,		                              // In: Ship length
			const double w,		                              // In: Ship width
			const double T_U,	                              // In: Ship first order speed time constant
			const double T_chi,	                              // In: Ship first order course time constant
			const double R_a,	                              // In: Ship radius of acceptance parameter in WP following
			const double LOS_LD,                              // In: Ship lookahead distance parameter in LOS WP following
			const double LOS_K_i,                             // In: Ship integral gain parameter in LOS WP following
			const Path_Prediction_Shape path_prediction_shape // In: Shape of the ship's predicted path
			) : l(l), w(w), T_U(T_U), T_chi(T_chi), R_a(R_a), 
				LOS_LD(LOS_LD), LOS_K_i(LOS_K_i), path_prediction_shape(path_prediction_shape)
		{

			/* l = 10.0;
			w = 4.0;*/

			// Guidance parameters
			e_int = 0;
			e_int_max = 20 * M_PI / 180.0; // Maximum integral correction in LOS guidance

			wp_c_0 = 0;
			wp_c_p = 0;
		}

		// Pybind11 compatability overload
		Kinematic_Ship::Kinematic_Ship(const Kinematic_Ship &other)
		{
			this->l = other.l;
			this->w = other.w;

			this->T_U = other.T_U;
			this->T_chi = other.T_chi;

			this->e_int = other.e_int;
			this->e_int_max = other.e_int_max;
			this->R_a = other.R_a;
			this->LOS_LD = other.LOS_LD;
			this->LOS_K_i = other.LOS_K_i;

			this->wp_c_0 = other.wp_c_0;
			this->wp_c_p = other.wp_c_p;

			this->path_prediction_shape = other.path_prediction_shape;
		}

		void Kinematic_Ship::determine_active_waypoint_segment(
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow
			const Eigen::Vector4d &xs					   // In: Ownship state
		)
		{
			int n_wps = waypoints.cols();
			Eigen::Vector2d d_0_wp, L_wp_segment, L_0wp;
			bool segment_passed = false;

			if (n_wps <= 2)
			{
				wp_c_0 = 0;
				wp_c_p = 0;
				return;
			}

			for (int i = wp_c_0; i < n_wps - 1; i++)
			{
				d_0_wp(0) = waypoints(0, i + 1) - xs(0);
				d_0_wp(1) = waypoints(1, i + 1) - xs(1);

				L_wp_segment(0) = waypoints(0, i + 1) - waypoints(0, i);
				L_wp_segment(1) = waypoints(1, i + 1) - waypoints(1, i);
				L_wp_segment = L_wp_segment.normalized();

				segment_passed = L_wp_segment.dot(d_0_wp.normalized()) < cos(90 * DEG2RAD);

				if (d_0_wp.norm() <= R_a || segment_passed)
				{
					wp_c_0++;
					std::cout << "Segment " << i << " passed" << std::endl;
				}
				else
				{
					break;
				}
			}
			wp_c_p = wp_c_0;
		}

		void Kinematic_Ship::update_guidance_references(
			double &u_d,								   // In/out: Surge reference
			double &chi_d,								   // In/out: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt,							   // In: Time step
			const Guidance_Method guidance_method		   // In: Type of guidance used
		)
		{
			// Nominally no surge modification
			u_d = 1.0 * u_d;

			int n_wps = waypoints.cols();
			double alpha(0.0), e(0.0);
			Eigen::Vector2d d_next_wp, L_wp_segment;
			bool segment_passed(false), inside_wp_R_a(false);

			if (guidance_method == LOS || guidance_method == WPP)
			{
				// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
				if (wp_c_p == n_wps - 1)
				{
					d_next_wp(0) = waypoints(0, wp_c_p) - xs(0);
					d_next_wp(1) = waypoints(1, wp_c_p) - xs(1);
					L_wp_segment(0) = waypoints(0, wp_c_p) - waypoints(0, wp_c_p - 1);
					L_wp_segment(1) = waypoints(1, wp_c_p) - waypoints(1, wp_c_p - 1);
				}
				else
				{
					d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
					d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);
					L_wp_segment(0) = waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p);
					L_wp_segment(1) = waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p);
				}
				L_wp_segment = L_wp_segment.normalized();

				segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

				inside_wp_R_a = d_next_wp.norm() <= R_a;

				if (inside_wp_R_a || segment_passed)
				{
					e_int = 0;
					if (wp_c_p < n_wps - 1)
					{
						wp_c_p++;
					}
				}
			}

			// After last waypoint is reached the own-ship stops
			if (wp_c_p == n_wps - 1 && inside_wp_R_a)
			{
				u_d = 0.0;
			}

			switch (guidance_method)
			{
			case LOS:
				alpha = atan2(L_wp_segment(1), L_wp_segment(0));
				e = -(xs(0) - waypoints(0, wp_c_p)) * sin(alpha) + (xs(1) - waypoints(1, wp_c_p)) * cos(alpha);
				e_int += e * dt;
				if (e_int >= e_int_max)
					e_int -= e * dt;
				chi_d = CPU::wrap_angle_to_pmpi(alpha + atan2(-(e + LOS_K_i * e_int), LOS_LD));
				break;
			case WPP:
				chi_d = CPU::wrap_angle_to_pmpi(atan2(d_next_wp(1), d_next_wp(0)));
				break;
			case CH:
				chi_d = CPU::wrap_angle_to_pmpi(xs(2));
				break;
			default:
				// Throw
				break;
			}
		}

		void Kinematic_Ship::update_guidance_references(
			double &u_d,								   // In/out: Surge reference
			double &chi_d,								   // In/out: Course reference
			double &cross_track_error,					   // In/out: Cross track error
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt,							   // In: Time step
			const Guidance_Method guidance_method		   // In: Type of guidance used
		)
		{
			// Nominally no surge modification
			u_d = 1.0 * u_d;

			int n_wps = waypoints.cols();
			double alpha(0.0), e(0.0);
			Eigen::Vector2d d_next_wp, L_wp_segment;
			bool segment_passed(false), inside_wp_R_a(false);

			if (guidance_method == LOS || guidance_method == WPP)
			{
				// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
				if (wp_c_p == n_wps - 1)
				{
					d_next_wp(0) = waypoints(0, wp_c_p) - xs(0);
					d_next_wp(1) = waypoints(1, wp_c_p) - xs(1);
					L_wp_segment(0) = waypoints(0, wp_c_p) - waypoints(0, wp_c_p - 1);
					L_wp_segment(1) = waypoints(1, wp_c_p) - waypoints(1, wp_c_p - 1);
				}
				else
				{
					d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
					d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);
					L_wp_segment(0) = waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p);
					L_wp_segment(1) = waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p);
				}
				L_wp_segment = L_wp_segment.normalized();

				segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

				inside_wp_R_a = d_next_wp.norm() <= R_a;

				if (inside_wp_R_a || segment_passed)
				{
					e_int = 0;
					if (wp_c_p < n_wps - 1)
					{
						wp_c_p++;
					}
				}
			}

			// After last waypoint is reached the own-ship stops
			if (wp_c_p == n_wps - 1 && inside_wp_R_a)
			{
				u_d = 0.0;
			}

			switch (guidance_method)
			{
			case LOS:
				alpha = atan2(L_wp_segment(1), L_wp_segment(0));
				e = -(xs(0) - waypoints(0, wp_c_p)) * sin(alpha) + (xs(1) - waypoints(1, wp_c_p)) * cos(alpha);
				e_int += e * dt;
				if (e_int >= e_int_max)
					e_int -= e * dt;
				chi_d = CPU::wrap_angle_to_pmpi(alpha + atan2(-(e + LOS_K_i * e_int), LOS_LD));
				break;
			case WPP:
				chi_d = CPU::wrap_angle_to_pmpi(atan2(d_next_wp(1), d_next_wp(0)));
				break;
			case CH:
				chi_d = CPU::wrap_angle_to_pmpi(xs(2));
				break;
			default:
				// Throw
				break;
			}
			cross_track_error = e;
		}

		void Kinematic_Ship::update_guidance_references(
			double &u_d,								   // In/out: Surge reference
			double &chi_d,								   // In/out: Course reference
			const double e_m,							   // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt								   // In: Time step
		)
		{
			// Nominally no surge modification
			u_d = 1.0 * u_d;

			int n_wps = waypoints.cols();
			double alpha(0.0), e(0.0);
			Eigen::Vector2d d_next_wp, L_wp_segment;
			bool segment_passed(false), inside_wp_R_a(false);

			// Determine if a switch must be made to the next waypoint segment, for LOS and WPP
			if (wp_c_p == n_wps - 1)
			{
				d_next_wp(0) = waypoints(0, wp_c_p) - xs(0);
				d_next_wp(1) = waypoints(1, wp_c_p) - xs(1);
				L_wp_segment(0) = waypoints(0, wp_c_p) - waypoints(0, wp_c_p - 1);
				L_wp_segment(1) = waypoints(1, wp_c_p) - waypoints(1, wp_c_p - 1);
			}
			else
			{
				d_next_wp(0) = waypoints(0, wp_c_p + 1) - xs(0);
				d_next_wp(1) = waypoints(1, wp_c_p + 1) - xs(1);
				L_wp_segment(0) = waypoints(0, wp_c_p + 1) - waypoints(0, wp_c_p);
				L_wp_segment(1) = waypoints(1, wp_c_p + 1) - waypoints(1, wp_c_p);
			}
			L_wp_segment = L_wp_segment.normalized();

			segment_passed = L_wp_segment.dot(d_next_wp.normalized()) < cos(90 * DEG2RAD);

			inside_wp_R_a = d_next_wp.norm() <= R_a;

			if (inside_wp_R_a || segment_passed)
			{
				e_int = 0;
				if (wp_c_p < n_wps - 1)
				{
					wp_c_p++;
				}
			}

			// After last waypoint is reached the own-ship stops
			if (wp_c_p == n_wps - 1 && inside_wp_R_a)
			{
				u_d = 0.0;
			}

			alpha = atan2(L_wp_segment(1), L_wp_segment(0));
			e = -(xs(0) - waypoints(0, wp_c_p)) * sin(alpha) + (xs(1) - waypoints(1, wp_c_p)) * cos(alpha);
			e += e_m; // Add artificial cross track error to cause different path alignment, for obstacle prediction
			e_int += e * dt;
			if (e_int >= e_int_max)
				e_int -= e * dt;
			chi_d = CPU::wrap_angle_to_pmpi(alpha + atan2(-(e + LOS_K_i * e_int), LOS_LD));
		}

		// Pybind11 compatibility overloads
		Eigen::Vector2d Kinematic_Ship::update_guidance_references_py(
			double &u_d,		                           // In/out from/to Python: Surge reference
			double &chi_d,                                 // In/out from/to Python: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt,							   // In: Time step
			const Guidance_Method guidance_method		   // In: Type of guidance used
		)
		{
			// Coupled to initial overload number 1
			update_guidance_references(u_d, chi_d, waypoints, xs, dt, guidance_method);
			Eigen::Vector2d output(u_d, chi_d);
			return output;
		}

		Eigen::Vector3d Kinematic_Ship::update_guidance_references_py(
			double &u_d,		                           // In/out from/to Python: Surge reference
			double &chi_d,                                 // In/out from/to Python: Course reference
			double cross_track_error,                      // In/out: Cross track error
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt,							   // In: Time step
			const Guidance_Method guidance_method		   // In: Type of guidance used
		)
		{
			// Coupled to initial overload number 2
			update_guidance_references(u_d, chi_d, cross_track_error, waypoints, xs, dt, guidance_method);	
			Eigen::Vector3d output(u_d, chi_d, cross_track_error);
			return output;
		}

		Eigen::Vector2d Kinematic_Ship::update_guidance_references_py(
			double &u_d,			                       // In/out from/to Python: Surge reference
			double &chi_d,			                       // In/out from/to Python: Course reference
			const double e_m,							   // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Waypoints to follow.
			const Eigen::Vector4d &xs,					   // In: Ownship state
			const double dt                                // In: Time step
		)								   
		{
			// Coupled to initial overload number 3
			update_guidance_references(u_d, chi_d, e_m, waypoints, xs, dt);
			Eigen::Vector2d output(u_d, chi_d);
			return output;
		}

		Eigen::Vector4d Kinematic_Ship::predict(
			const Eigen::Vector4d &xs_old,			  // In: State [x, y, chi, U] to predict forward
			const double U_d,						  // In: Speed over ground (SOG) reference
			const double chi_d,						  // In: Course (COG) reference
			const double dt,						  // In: Time step
			const Prediction_Method prediction_method // In: Method used for prediction
		)
		{
			Eigen::Vector4d xs_new;
			double chi_diff = angle_difference_pmpi(chi_d, xs_old(2));

			switch (prediction_method)
			{
			case Linear:
				// Straight line trajectory with the current heading and surge speed
				xs_new(0) = xs_old(0) + dt * xs_old(3) * cos(xs_old(2));
				xs_new(1) = xs_old(1) + dt * xs_old(3) * sin(xs_old(2));
				xs_new.block<2, 1>(2, 0) = xs_old.block<2, 1>(2, 0);
				break;
			case ERK1:
				// First set xs_new to the continuous time derivative of the model
				xs_new(0) = xs_old(3) * cos(xs_old(2));
				xs_new(1) = xs_old(3) * sin(xs_old(2));
				xs_new(2) = (1 / T_chi) * chi_diff;
				xs_new(3) = (1 / T_U) * (U_d - xs_old(3));

				// Then use forward euler to obtain new states
				xs_new = xs_old + dt * xs_new;
				break;
			default:
				// Throw
				xs_new.setZero();
			}
			xs_new(2) = wrap_angle_to_pmpi(xs_new(2));
			return xs_new;
		}

		void Kinematic_Ship::predict_trajectory(
			Eigen::MatrixXd &trajectory,				   // In/out: Obstacle ship trajectory
			const Eigen::VectorXd &offset_sequence,		   // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		   // In: Time indices for each collision avoidance maneuver
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		   // In: Type of guidance to be used
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)
		{
			int n_samples = std::round(T / dt);

			assert(trajectory.rows() == 4);
			trajectory.conservativeResize(4, n_samples);

			wp_c_p = wp_c_0;

			int man_count = 0;
			double u_m = 1, u_d_p = u_d;
			double chi_m = 0, chi_d_p = chi_d;
			Eigen::Vector4d xs = trajectory.col(0);

			for (int k = 0; k < n_samples; k++)
			{
				if (k == maneuver_times[man_count])
				{
					u_m = offset_sequence[2 * man_count];
					if (u_m < 0.1)
					{
						chi_m = 0.0;
					}
					else
					{
						chi_m += offset_sequence[2 * man_count + 1];
					}
					if (man_count < (int)maneuver_times.size() - 1)
						man_count += 1;
				}

				update_guidance_references(u_d_p, chi_d_p, waypoints, xs, dt, guidance_method);

				xs = predict(xs, u_m * u_d_p, CPU::wrap_angle_to_pmpi(chi_d_p + chi_m), dt, prediction_method);

				if (k < n_samples - 1)
					trajectory.col(k + 1) = xs;
			}
		}

		void Kinematic_Ship::predict_trajectory(
			Eigen::MatrixXd &trajectory,				   // In/out: Obstacle ship trajectory
			double &max_cross_track_error,				   // In: Maximum absolute predicted cross track error
			const Eigen::VectorXd &offset_sequence,		   // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		   // In: Time indices for each collision avoidance maneuver
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		   // In: Type of guidance to be used (equal to LOS here)
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)
		{
			int n_samples = std::round(T / dt);

			assert(trajectory.rows() == 4);
			trajectory.conservativeResize(4, n_samples);

			wp_c_p = wp_c_0;

			int man_count = 0;
			double u_m = 1, u_d_p = u_d;
			double chi_m = 0, chi_d_p = chi_d, e_k(0.0);
			Eigen::Vector4d xs = trajectory.col(0);
			max_cross_track_error = 0.0;

			for (int k = 0; k < n_samples; k++)
			{
				if (k == maneuver_times[man_count])
				{
					u_m = offset_sequence[2 * man_count];
					if (u_m < 0.1)
					{
						chi_m = 0.0;
					}
					else
					{
						chi_m += offset_sequence[2 * man_count + 1];
					}
					if (man_count < (int)maneuver_times.size() - 1)
						man_count += 1;
				}

				update_guidance_references(u_d_p, chi_d_p, e_k, waypoints, xs, dt, guidance_method);

				xs = predict(xs, u_m * u_d_p, CPU::wrap_angle_to_pmpi(chi_d_p + chi_m), dt, prediction_method);

				if (k < n_samples - 1)
					trajectory.col(k + 1) = xs;

				if (max_cross_track_error < abs(e_k))
				{
					max_cross_track_error = abs(e_k);
				}
			}
		}

		void Kinematic_Ship::predict_trajectory(
			Eigen::MatrixXd &trajectory,				       // In/out: Obstacle ship trajectory
			double &max_cross_track_error,				       // In: Maximum absolute predicted cross track error
			const Eigen::VectorXd &offset_sequence,		       // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		       // In: Time indices for each collision avoidance maneuver
			const double u_d,							       // In: Surge reference
			const double chi_d,							       // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints,     // In: Obstacle waypoints
			const Prediction_Method prediction_method,	       // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		       // In: Type of guidance to be used (equal to LOS here)
			const Path_Prediction_Shape path_prediction_shape, // In: Type of prediction geometry to be used
			const double T,								       // In: Prediction horizon
			const double dt								       // In: Prediction time step
		)
		{
			int n_samples = std::round(T / dt);

			assert(trajectory.rows() == 4);
			trajectory.conservativeResize(4, n_samples);

			wp_c_p = wp_c_0;

			int man_count = 0;
			double u_m = 1, u_d_p = u_d;
			double chi_m = 0, chi_d_p = chi_d, e_k(0.0);
			Eigen::Vector4d xs = trajectory.col(0);
			max_cross_track_error = 0.0;

			for (int k = 0; k < n_samples; k++)
			{
				if (k == maneuver_times[man_count])
				{
					u_m = offset_sequence[2 * man_count];
					if (u_m < 0.1)
					{
						chi_m = 0.0;
					}
					else
					{
						chi_m += offset_sequence[2 * man_count + 1];
					}
					if (man_count < (int)maneuver_times.size() - 1)
						man_count += 1;
				}

				update_guidance_references(u_d_p, chi_d_p, e_k, waypoints, xs, dt, guidance_method);

				switch (path_prediction_shape)
				{
				case LINEAR: // "Piecewise-linear" path
					xs = predict(xs, u_m * u_d_p, CPU::wrap_angle_to_pmpi(chi_m), dt, prediction_method);
					break;
				case SMOOTH: // The geometry which is used in the standard implementation of the PSB-MPC. 
					xs = predict(xs, u_m * u_d_p, CPU::wrap_angle_to_pmpi(chi_d_p + chi_m), dt, prediction_method);
					break;
				default: // The Smooth prediction_geometry is set as the default behaviour
					xs = predict(xs, u_m * u_d_p, CPU::wrap_angle_to_pmpi(chi_d_p + chi_m), dt, prediction_method);
					break;
				}
				
				if (k < n_samples - 1)
					trajectory.col(k + 1) = xs;

				if (max_cross_track_error < abs(e_k))
				{
					max_cross_track_error = abs(e_k);
				}
			}
		}

		void Kinematic_Ship::predict_trajectory(
			Eigen::MatrixXd &trajectory,				   // In/out: Obstacle ship trajectory
			const double e_m,							   // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)
		{
			int n_samples = std::round(T / dt);

			assert(trajectory.rows() == 4);
			trajectory.conservativeResize(4, n_samples);

			wp_c_p = wp_c_0;

			double u_d_p = u_d;
			double chi_d_p = chi_d;
			Eigen::Vector4d xs = trajectory.col(0);

			for (int k = 0; k < n_samples; k++)
			{
				update_guidance_references(u_d_p, chi_d_p, e_m, waypoints, xs, dt);

				xs = predict(xs, u_d_p, chi_d_p, dt, prediction_method);

				if (k < n_samples - 1)
					trajectory.col(k + 1) = xs;
			}
		}

		void Kinematic_Ship::predict_trajectory(
			Eigen::MatrixXd &trajectory,				       // In/out: Obstacle ship trajectory
			const double e_m,							       // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const double chi_m,                                // In: Modifier to the course to cause a different path alignment
			const double u_d,							       // In: Surge reference
			const double chi_d,							       // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints,     // In: Obstacle waypoints
			const Prediction_Method prediction_method,	       // In: Type of prediction method to be used, typically an explicit method
			const Path_Prediction_Shape path_prediction_shape, // In: Type of prediction geometry to be used
			const double T,								       // In: Prediction horizon
			const double dt								       // In: Prediction time step
		)
		{
			int n_samples = std::round(T / dt);

			assert(trajectory.rows() == 4);
			trajectory.conservativeResize(4, n_samples);

			wp_c_p = wp_c_0;

			double u_d_p = u_d;
			double chi_d_p = chi_d;
			double chi_m_p = chi_m;
			Eigen::Vector4d xs = trajectory.col(0);

			for (int k = 0; k < n_samples; k++)
			{	
				switch (path_prediction_shape)
				{
					case LINEAR:
						update_guidance_references(u_d_p, chi_m_p, 0, waypoints, xs, dt);
						xs = predict(xs, u_d_p, chi_m + chi_d_p, dt, prediction_method);
						break;
					case SMOOTH:
						update_guidance_references(u_d_p, chi_d_p, e_m, waypoints, xs, dt);
						xs = predict(xs, u_d_p, chi_d_p, dt, prediction_method);
						break;
					default:
						update_guidance_references(u_d_p, chi_d_p, e_m, waypoints, xs, dt);
						xs = predict(xs, u_d_p, chi_d_p, dt, prediction_method);
						break;
				}

				if (k < n_samples - 1)
					trajectory.col(k + 1) = xs;
			}
		}

		// Pybind11 compatibility overloads
		Eigen::MatrixXd Kinematic_Ship::predict_trajectory_py(
			Eigen::MatrixXd &trajectory,                   // In/out from/to Python: Obstacle ship trajectory
			const Eigen::VectorXd &offset_sequence,		   // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		   // In: Time indices for each collision avoidance maneuver
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		   // In: Type of guidance to be used
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)				    
		{
			// Coupled to initial overload number 1
			predict_trajectory(trajectory, offset_sequence, maneuver_times, u_d, chi_d, waypoints, prediction_method, guidance_method, T, dt);
			return trajectory;
		}

		Eigen::MatrixXd Kinematic_Ship::predict_trajectory_py(
			Eigen::MatrixXd &trajectory,		           // In/out from/to Python: Obstacle ship trajectory
			double &max_cross_track_error,				   // In: Maximum absolute predicted cross track error
			const Eigen::VectorXd &offset_sequence,		   // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		   // In: Time indices for each collision avoidance maneuver
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		   // In: Type of guidance to be used (equal to LOS here)
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)
		{
			// Coupled to initial overload number 2
			predict_trajectory(trajectory, max_cross_track_error, offset_sequence, maneuver_times, u_d, chi_d, waypoints, prediction_method, guidance_method, T, dt);
			return trajectory;
		}

		Eigen::MatrixXd Kinematic_Ship::predict_trajectory_py(
			Eigen::MatrixXd &trajectory,		               // In/out from/to Python: Obstacle ship trajectory
			double &max_cross_track_error,				       // In: Maximum absolute predicted cross track error
			const Eigen::VectorXd &offset_sequence,		       // In: Sequence of offsets in the candidate control behavior
			const Eigen::VectorXd &maneuver_times,		       // In: Time indices for each collision avoidance maneuver
			const double u_d,							       // In: Surge reference
			const double chi_d,							       // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints,     // In: Obstacle waypoints
			const Prediction_Method prediction_method,	       // In: Type of prediction method to be used, typically an explicit method
			const Guidance_Method guidance_method,		       // In: Type of guidance to be used (equal to LOS here)
			const Path_Prediction_Shape path_prediction_shape, // In: Type of prediction geometry to be used
			const double T,								       // In: Prediction horizon
			const double dt								       // In: Prediction time step
		)
		{
			// Coupled to initial overload number 3
			predict_trajectory(trajectory, max_cross_track_error, offset_sequence, maneuver_times, u_d, chi_d, waypoints, prediction_method, guidance_method, path_prediction_shape, T, dt);
			return trajectory;
		}

		Eigen::MatrixXd Kinematic_Ship::predict_trajectory_py(
			Eigen::MatrixXd &trajectory,        		   // In/out from/to Python: Obstacle ship trajectory
			const double e_m,							   // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const double u_d,							   // In: Surge reference
			const double chi_d,							   // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints, // In: Obstacle waypoints
			const Prediction_Method prediction_method,	   // In: Type of prediction method to be used, typically an explicit method
			const double T,								   // In: Prediction horizon
			const double dt								   // In: Prediction time step
		)
		{
			// Coupled to initial overload number 4
			predict_trajectory(trajectory, e_m, u_d, chi_d, waypoints, prediction_method, T, dt);
			return trajectory;
		}

		Eigen::MatrixXd Kinematic_Ship::predict_trajectory_py(
			Eigen::MatrixXd &trajectory,				       // In/out from/to Python: Obstacle ship trajectory
			const double e_m,							       // In: Modifier to the LOS-guidance cross track error to cause a different path alignment
			const double chi_m,                                // In: Modifier to the course to cause a different path alignment
			const double u_d,							       // In: Surge reference
			const double chi_d,							       // In: Course reference
			const Eigen::Matrix<double, 2, -1> &waypoints,     // In: Obstacle waypoints
			const Prediction_Method prediction_method,	       // In: Type of prediction method to be used, typically an explicit method
			const Path_Prediction_Shape path_prediction_shape, // In: Type of prediction geometry to be used
			const double T,								       // In: Prediction horizon
			const double dt								       // In: Prediction time step
		)
		{
			// Coupled to initial overload number 5
			predict_trajectory(trajectory, e_m, chi_m, u_d, chi_d, waypoints, prediction_method, path_prediction_shape, T, dt);
			return trajectory;
		}

		/****************************************************************************************
				Private functions
		*****************************************************************************************/
	}
}