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

#include "psbmpc_parameters.hpp"
#include "gpu/utilities_gpu.cuh"
#include "tml/tml.cuh"

#include <cmath>
#include <thrust/device_vector.h>

namespace PSBMPC_LIB
{
    //NOTE! Ownship has states [X, Y, CHI, U], while obstacle ship has [X, Y, VX, VY]
    namespace GPU
    {
        class COLREGS_Violation_Evaluator
        {
        public:
            /****************************************************************************************
            *  Name     : COLREGS_Violation_Evaluator
            *  Function : Class constructor
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __host__ COLREGS_Violation_Evaluator()
            {
                pars.max_distance_at_cpa = 50.0;
                pars.d_init_colregs_situation = 500.0;
                pars.head_on_width = 30.0 * DEG2RAD;
                pars.overtaking_angle = (90.0 + 22.5) * DEG2RAD;
                pars.max_acceptable_SO_speed_change = 2.0;
                pars.max_acceptable_SO_course_change = 2.5 * DEG2RAD;
                pars.critical_distance_to_ignore_SO = 30.0;
                pars.GW_safety_margin = 10.0;
            }

            __host__ COLREGS_Violation_Evaluator(const CVE_Pars<float> &pars) : pars(pars) {}

            /****************************************************************************************
            *  Name     : update
            *  Function : Determines if the COLREGS situation has started, sets initial states if
            *             that is the case
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ void update(const TML::PDVector4f &ownship_state, const TML::PDVector4f &obstacle_state_vx_vy)
            {
                obstacle_state = vx_vy_to_heading_speed_state(obstacle_state_vx_vy);
                if (!initialized && evaluate_situation_started(ownship_state, obstacle_state))
                {
                    initialized = true;
                    colregs_situation = evaluate_colregs_situation(ownship_state, obstacle_state);
                    initial_ownship_state = ownship_state;
                    initial_obstacle_state = obstacle_state;
                }
                else if (initialized)
                {
                    evaluate_actual_maneuver_changes(ownship_state(COG), ownship_state(SOG));
                
                    if(!has_passed){
                        //If the intersection point is in the past, then the ships have passed
                        const TML::Vector2f intersection_point = intersectionpoint(ownship_state, vx_vy_to_heading_speed_state(obstacle_state_vx_vy));
                        has_passed = evaluate_arrival_time(ownshipCPA, intersection_point(0), intersection_point(1)) < 0;
                    }
                    
                }
            }

            /****************************************************************************************
            *  Name     : evaluate_actual_maneuver_changes
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ void evaluate_actual_maneuver_changes(
                const float chi_0, // In: Own-ship course at the current time t_0
                const float U_0    // In: Own-ship speed at the current time t_0
            )
            {
                if (!actual_ownship_speed_or_course_change)
                    actual_ownship_speed_or_course_change = fabs(wrap_angle_to_pmpi(chi_0 - initial_ownship_state(COG))) > pars.max_acceptable_SO_course_change || fabs(U_0 - initial_ownship_state(SOG)) > pars.max_acceptable_SO_speed_change;

                if (!actual_ownship_course_change_port)
                    actual_ownship_course_change_port = wrap_angle_to_pmpi(chi_0 - initial_ownship_state(COG)) < -pars.max_acceptable_SO_course_change;
            }

            /****************************************************************************************
            *  Name     : evaluate_predicted_maneuver_changes
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ void evaluate_predicted_maneuver_changes(
                const float chi, // In: Own-ship course at the current predicted time k
                const float U    // In: Own-ship speed at the current predicted time k
            )
            {
                if (!predicted_ownship_change_in_speed_or_course)
                    predicted_ownship_change_in_speed_or_course = fabs(wrap_angle_to_pmpi(chi - initial_ownship_state(COG))) > pars.max_acceptable_SO_course_change || fabs(U - initial_ownship_state(SOG)) > pars.max_acceptable_SO_speed_change;

                if (!predicted_ownship_change_in_course_to_port)
                    predicted_ownship_change_in_course_to_port = wrap_angle_to_pmpi(chi - initial_ownship_state(COG)) < -pars.max_acceptable_SO_course_change;
            }

            /****************************************************************************************
            *  Name     : evaluate_SO_violation
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_SO_violation(
                const float d_0i_0, // In: Distance at the current time t_0 between ownship and obstacle i
                const float d_cpa   // In: Distance at predicted CPA between ownship and obstacle i
            )
            {
                if (!initialized || has_passed)
                    return false;

                return d_cpa < pars.max_distance_at_cpa &&
                       d_0i_0 > pars.critical_distance_to_ignore_SO &&
                       (colregs_situation == OT_en || colregs_situation == CR_PS) &&
                       (actual_ownship_speed_or_course_change || predicted_ownship_change_in_speed_or_course);
            }

            /****************************************************************************************
            *  Name     : evaluate_GW_violation
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_GW_violation(
                const TML::PDVector4f &ownship_CPA_state,        // In: Ownship state at CPA
                const TML::PDVector4f &obstacle_CPA_state_vx_vy, // In: Obstacle i state at CPA
                const float d_cpa                                // In: Distance at actual predicted CPA between ownship and obstacle i
            )
            {
                if (!initialized || has_passed)
                    return false;

                correct_HO_maneuver = evaluate_crossing_port_to_port(ownship_CPA_state, vx_vy_to_heading_speed_state(obstacle_CPA_state_vx_vy)) && d_cpa >= pars.GW_safety_margin;
                correct_CR_SS_maneuver = evaluate_crossing_aft(ownship_CPA_state, vx_vy_to_heading_speed_state(obstacle_CPA_state_vx_vy)) && d_cpa >= pars.GW_safety_margin;
                correct_OT_ing_maneuver = d_cpa >= pars.GW_safety_margin;
                correct_CR_PS_maneuver = !(actual_ownship_course_change_port || predicted_ownship_change_in_course_to_port);
                return d_cpa < pars.max_distance_at_cpa &&
                       ((colregs_situation == HO && !correct_HO_maneuver) ||
                        (colregs_situation == CR_SS && !correct_CR_SS_maneuver) ||
                        (colregs_situation == CR_PS && !correct_CR_PS_maneuver) ||
                        (colregs_situation == OT_ing && correct_OT_ing_maneuver));
            }

            __device__ void reset()
            {
                predicted_ownship_change_in_course_to_port = false;
                predicted_ownship_change_in_speed_or_course = false;
            }

        private:
            CVE_Pars<float> pars;
            bool initialized = false;
            bool predicted_ownship_change_in_course_to_port = false;
            bool predicted_ownship_change_in_speed_or_course = false;
            bool actual_ownship_speed_or_course_change = false;
            bool actual_ownship_course_change_port = false;
            bool has_passed = false; //If the ships have passed each other, then dont give any penalty

            COLREGS_Situation colregs_situation;
            TML::PDVector4f initial_ownship_state;
            TML::PDVector4f initial_obstacle_state;

            //=================================
            // Temporaries
            bool correct_HO_maneuver;
            bool correct_CR_SS_maneuver;
            bool correct_CR_PS_maneuver;
            bool correct_OT_ing_maneuver;
            float heading_diff;
            float bearing_to_obstacle_relative_to_ownship;
            float bearing_to_ownship_relative_to_obstacle;
            float bearing_to_intersection_point;
            float ownship_crossing_arrival_time;
            float obstacle_crossing_arrival_time;

            TML::PDVector4f obstacle_state;
            TML::Vector2f intersection_point;
            //=================================

            /****************************************************************************************
            *  Name     : evaluate_situation_started
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __host__ __device__ bool evaluate_situation_started(
                const TML::PDVector4f &ownship_state,
                const TML::PDVector4f &obstacle_state)
            {
                return evaluateDistance(ownship_state, obstacle_state) < pars.d_init_colregs_situation;
            }

            /****************************************************************************************
            *  Name     : evaluate_colregs_situation
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __host__ __device__ COLREGS_Situation evaluate_colregs_situation(
                const TML::PDVector4f &ownship_state,
                const TML::PDVector4f &obstacle_state)
            {
                heading_diff = wrap_angle_to_pmpi(obstacle_state(COG) - ownship_state(COG));
                if (heading_diff < -M_PI + pars.head_on_width / 2 || heading_diff > M_PI - pars.head_on_width / 2)
                    return HO;

                bearing_to_obstacle_relative_to_ownship = relativeBearing(ownship_state, obstacle_state(PX), obstacle_state(PY));
                if (bearing_to_obstacle_relative_to_ownship > pars.overtaking_angle || bearing_to_obstacle_relative_to_ownship < -pars.overtaking_angle)
                    return OT_en;

                bearing_to_ownship_relative_to_obstacle = relativeBearing(obstacle_state, ownship_state(PX), ownship_state(PY));
                if (bearing_to_ownship_relative_to_obstacle > pars.overtaking_angle || bearing_to_ownship_relative_to_obstacle < -pars.overtaking_angle)
                    return OT_ing;

                if (bearing_to_obstacle_relative_to_ownship < 0)
                    return CR_PS;

                return CR_SS;
            }

            /****************************************************************************************
            *  Name     : evaluate_crossing_port_to_port
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __host__ __device__ bool evaluate_crossing_port_to_port(
                const TML::PDVector4f &ownshipCPA,
                const TML::PDVector4f &obstacleCPA)
            {
                return relativeBearing(ownshipCPA, obstacleCPA(PX), obstacleCPA(PY)) < 0;
            }

            /****************************************************************************************
            *  Name     : evaluate_crossing_aft
            *  Function :
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __host__ __device__ bool evaluate_crossing_aft(
                const TML::PDVector4f &ownshipCPA,
                const TML::PDVector4f &obstacleCPA)
            {
                intersection_point = intersectionpoint(ownshipCPA, obstacleCPA);
                ownship_crossing_arrival_time = evaluate_arrival_time(ownshipCPA, intersection_point(0), intersection_point(1));
                obstacle_crossing_arrival_time = evaluate_arrival_time(obstacleCPA, intersection_point(0), intersection_point(1));
                return ownship_crossing_arrival_time > obstacle_crossing_arrival_time;
            }
        };
    }
}
