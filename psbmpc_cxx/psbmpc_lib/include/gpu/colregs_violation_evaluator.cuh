/****************************************************************************************
*
*  File name : colregs_violation_evaluator.cuh
*
*  Function  : Header file for the COLREGS Violation Evaluator class, which keeps
*              track of and calculates the COLREGS violation cost.
*
*            ---------------------
*
*  Version 1.0
*
*  Copyright (C) 2020 Sverre Velten Rothmund, Trym Tengesdal NTNU Trondheim.
*  All rights reserved.
*
*  Author    : Sverre Velten Rothmund
*
*  Modified  :
*
*****************************************************************************************/
#pragma once

#include "gpu/utilities_gpu.cuh"
#include "tml/tml.cuh"

#include <cmath>
#include <thrust/device_vector.h>

// TODO LEGG TIL DOCUMENTATION TEMPLATES ALA DET NEDENFOR FOR CPU CLASSEN OGSÅ!
namespace PSBMPC_LIB
{
    //NOTE! Ownship has states [X, Y, CHI, U], while obstacle ship has [X, Y, VX, VY]......
    //This greatly increases the chance of bugs and makes this code difficult to check.
    namespace GPU
    {
        //Evaluates colregs violation between a pair of ships
        class COLREGS_Violation_Evaluator
        {
        public:

            /****************************************************************************************
            *  Name     : COLREGS_Violation_Evaluator
            *  Function : Class constructor
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __host__ COLREGS_Violation_Evaluator() = default;

            __host__ COLREGS_Violation_Evaluator(
                const double max_distance_at_cpa,
                const double d_close,
                const double head_on_width,
                const double overtaking_angle,
                const double max_acceptable_SO_speed_change,
                const double max_acceptable_SO_course_change,
                const double critical_distance_to_ignore_SO) 
                : 
                max_distance_at_cpa(max_distance_at_cpa), d_close(d_close), head_on_width(head_on_width), overtaking_angle(overtaking_angle), 
                max_acceptable_SO_speed_change(max_acceptable_SO_speed_change), max_acceptable_SO_course_change(max_acceptable_SO_course_change), 
                critical_distance_to_ignore_SO(critical_distance_to_ignore_SO)
            {}

            __host__ COLREGS_Violation_Evaluator(const COLREGS_Violation_Evaluator &other) = default;

            /****************************************************************************************
            *  Name     : operator=
            *  Function : Assignment operator
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __host__ COLREGS_Violation_Evaluator &operator=(const COLREGS_Violation_Evaluator &rhs) = default;


            /****************************************************************************************
            *  Name     : update
            *  Function : TODO
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ void update(const TML::PDVector4f &ownship_state, const TML::PDVector4f &obstacle_state_vx_vy)
            {
                const auto obstacle_state = vx_vy_to_heading_speed_state(obstacle_state_vx_vy);
                if (evaluate_situation_started(ownship_state, obstacle_state))
                {
                    initialized = true;
                    colregs_situation = evaluate_colregs_situation(ownship_state, obstacle_state);
                    initial_ownship_state = ownship_state;
                    initial_obstacle_state = obstacle_state;
                }
            }

            /****************************************************************************************
            *  Name     : evaluate_SO_violation
            *  Function : TODO
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_SO_violation(
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &ownship_trajectory, 
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &obstacle_trajectory)
            {
                if (!initialized)
                    return false;

                const bool distance_larger_than_critical = std::abs((ownship_trajectory.col(0).head(2) - obstacle_trajectory.col(0).head(2)).norm()) > critical_distance_to_ignore_SO;
                const auto course_change = evaluate_course_change(ownship_trajectory);
                const auto speed_change = evaluate_speed_change(ownship_trajectory);
                const bool stands_on_correct =  course_change == CourseChange::None &&  speed_change == SpeedChange::None;
                const bool has_SO_role = colregs_situation == OT_en || colregs_situation == CR_PS;
                const bool is_risk_of_collision = evaluate_risk_of_collision(ownship_trajectory, obstacle_trajectory);
                const bool so_violation = is_risk_of_collision && distance_larger_than_critical && has_SO_role && !stands_on_correct;
                return so_violation;
            }

            /****************************************************************************************
            *  Name     : evaluate_GW_violation
            *  Function : TODO
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_GW_violation(
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &ownship_trajectory, 
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &obstacle_trajectory)
            {
                if (!initialized)
                    return false;

                bool correct_HO_maneuver = evaluate_crossing_port_to_port(ownship_trajectory, obstacle_trajectory);
                bool correct_CR_SS_maneuver = evaluate_crossing_aft(ownship_trajectory, obstacle_trajectory);
                bool correct_CR_PS_maneuver = evaluate_course_change(ownship_trajectory) != CourseChange::Portwards;
                bool is_risk_of_collision = evaluate_risk_of_collision(ownship_trajectory, obstacle_trajectory);
                bool gw_violation =  is_risk_of_collision                                        &&
                        ((colregs_situation == HO && !correct_HO_maneuver)          ||
                        (colregs_situation == CR_SS && !correct_CR_SS_maneuver)     ||
                        (colregs_situation == CR_PS && !correct_CR_PS_maneuver));
                return gw_violation;
            }

        private:

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

            bool initialized = false;
            double max_distance_at_cpa = 100.0;
            double d_close = 800.0;
            double head_on_width = 10.0 * DEG2RAD;
            double overtaking_angle = (90.0 + 22.5) * DEG2RAD;
            double max_acceptable_SO_speed_change = 2.0;
            double max_acceptable_SO_course_change = 2.5 * DEG2RAD;
            double critical_distance_to_ignore_SO = 0.0;

            COLREGS_Situation colregs_situation;
            TML::Vector4f initial_ownship_state;
            TML::Vector4f initial_obstacle_state;

            /****************************************************************************************
            *  Name     : evaluate_situation_started
            *  Function : TODO
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_situation_started(
                const TML::PDVector4f &ownship_state, 
                const TML::PDVector4f &obstacle_state
                )
            {
                return evaluateCPA(ownship_state, obstacle_state).time_untill_CPA < d_close // FIX UNITS TO EITHER M OR M/S
            }

            /****************************************************************************************
            *  Name     : evaluate_colregs_situation
            *  Function : TODO
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ COLREGS_Situation evaluate_colregs_situation(
                const TML::PDVector4f &ownship_state, 
                const TML::PDVector4f &obstacle_state
                )
            {
                const double heading_diff = wrapPI(obstacle_state(COG) - ownship_state(COG));
                if (heading_diff < -M_PI + head_on_width / 2 || heading_diff > M_PI - head_on_width / 2)
                    return HO;

                const double bearing_to_obstacle_relative_to_ownship = relativeBearing(ownship_state, obstacle_state(PX), obstacle_state(PY));
                if (bearing_to_obstacle_relative_to_ownship > overtaking_angle || bearing_to_obstacle_relative_to_ownship < -overtaking_angle)
                    return OT_en;

                const double bearing_to_ownship_relative_to_obstacle = relativeBearing(obstacle_state, ownship_state(PX), ownship_state(PY));
                if (bearing_to_ownship_relative_to_obstacle > overtaking_angle || bearing_to_ownship_relative_to_obstacle < -overtaking_angle)
                    return OT_ing;

                if (bearing_to_obstacle_relative_to_ownship < 0)
                    return CR_PS;

                return CR_SS;
            }

            /****************************************************************************************
            *  Name     : evaluate_risk_of_collision
            *  Function : 
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_risk_of_collision(
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &ownship_trajectory, 
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &obstacle_trajectory
                )
            {
                auto CPA = evaluateCPA(ownship_trajectory, obstacle_trajectory);
                return CPA.closest_distance < max_distance_at_cpa;
            }
            

            /****************************************************************************************
            *  Name     : evaluate_course_change
            *  Function : 
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ CourseChange evaluate_course_change(
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &ownship_trajectory
                )
            {
                for (int i = 0; i < ownship_trajectory.get_cols(); ++i)
                {
                    if (ownship_trajectory(COG, i) - initial_ownship_state(COG) < -max_acceptable_SO_course_change){
                        return CourseChange::Portwards;
                    }
                    if (ownship_trajectory(COG, i) - initial_ownship_state(COG) > max_acceptable_SO_course_change){
                        return CourseChange::Starboardwards;
                    }
                }
                return CourseChange::None;
            }

            /****************************************************************************************
            *  Name     : evaluate_speed_change
            *  Function : 
            *  Author   : 
            *  Modified :
            *****************************************************************************************/
            __device__ SpeedChange evaluate_speed_change(
                const TML::PDMatrix<float, 4, MAX_N_SAMPLES> &ownship_trajectory
                )
            {
                //Seems like the trajectories are slowing down towards the end as they are approaching the final wp, so im only considering the first half of the traj as there shouldnt be any changes in speed or course after that
                for (int i = 0; i < ownship_trajectory.get_cols()/2; ++i)
                {
                    if (ownship_trajectory(SOG, i) - initial_ownship_state(SOG) > max_acceptable_SO_speed_change)
                        return SpeedChange::Higher;
                    if (ownship_trajectory(SOG, i) - initial_ownship_state(SOG) < -max_acceptable_SO_speed_change)
                        return SpeedChange::Lower;
                }
                return SpeedChange::None;
            }
        };
    }

}