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
            __host__ COLREGS_Violation_Evaluator() = default;

            __host__ COLREGS_Violation_Evaluator(const CVE_Pars<float> &pars) : pars(pars) {}

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
            *  Function : Determines if the COLREGS situation has started, sets initial states if
            *             that is the case
            *  Author   :
            *  Modified :
            *****************************************************************************************/
           __host__ void update(const Eigen::Vector4d &ownship_state_eigen, const Eigen::Vector4d &obstacle_state_vx_vy_eigen)
            {
                TML::PDVector4f ownship_state, obstacle_state_vx_vy;
                TML::assign_eigen_object(obstacle_state_vx_vy, obstacle_state_vx_vy_eigen);
                TML::assign_eigen_object(ownship_state, ownship_state_eigen);

                obstacle_state = vx_vy_to_heading_speed_state(obstacle_state_vx_vy);
                if (evaluate_situation_started(ownship_state, obstacle_state))
                {
                    initialized = true;
                    colregs_situation = evaluate_colregs_situation(ownship_state, obstacle_state);
                    initial_ownship_state = ownship_state;
                    initial_obstacle_state = obstacle_state;
                }
            }

            __host__ __device__ void update(const TML::PDVector4f &ownship_state, const TML::PDVector4f &obstacle_state_vx_vy)
            {
                obstacle_state = vx_vy_to_heading_speed_state(obstacle_state_vx_vy);
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
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_SO_violation(
                const TML::PDVector4f &ownship_current_state,
                const TML::PDVector4f &obstacle_current_state_vx_vy,
                bool has_been_change_in_speed_or_course,
                float dCPA
                )
            {
                if (!initialized)
                    return false;

                distance_larger_than_critical = evaluateDistance(ownship_current_state,obstacle_current_state_vx_vy) > pars.critical_distance_to_ignore_SO;
                stands_on_correct = !has_been_change_in_speed_or_course;
                has_SO_role = colregs_situation == OT_en || colregs_situation == CR_PS;
                is_risk_of_collision = dCPA < pars.max_distance_at_cpa;
 
                return is_risk_of_collision && distance_larger_than_critical && has_SO_role && !stands_on_correct;
            }

            /****************************************************************************************
            *  Name     : evaluate_GW_violation
            *  Function : TODO
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_GW_violation(
                const TML::PDVector4f &ownship_CPA_state,
                const TML::PDVector4f &obstacle_CPA_state_vx_vy, 
                bool has_been_change_in_course_to_port, 
                float dCPA
                )
            {
                if (!initialized)
                    return false;

                correct_HO_maneuver = evaluate_crossing_port_to_port(ownship_CPA_state, obstacle_CPA_state_vx_vy);
                correct_CR_SS_maneuver = evaluate_crossing_aft(ownship_CPA_state, obstacle_CPA_state_vx_vy);
                correct_CR_PS_maneuver = !has_been_change_in_course_to_port;
                return  dCPA < pars.max_distance_at_cpa                             &&
                        ((colregs_situation == HO && !correct_HO_maneuver)          ||
                        (colregs_situation == CR_SS && !correct_CR_SS_maneuver)     ||
                        (colregs_situation == CR_PS && !correct_CR_PS_maneuver));
            }

        private:

            bool initialized = false;
            
            CVE_Pars<float> pars;

            COLREGS_Situation colregs_situation;
            TML::PDVector4f initial_ownship_state;
            TML::PDVector4f initial_obstacle_state;


            //=================================
            // Temporaries
            bool distance_larger_than_critical;
            bool stands_on_correct;
            bool has_SO_role;
            bool is_risk_of_collision;

            bool correct_HO_maneuver;
            bool correct_CR_SS_maneuver;
            bool correct_CR_PS_maneuver;

            float heading_diff;
            float bearing_to_obstacle_relative_to_ownship;
            float bearing_to_ownship_relative_to_obstacle;
            float bearing_to_intersection_point;

            TML::PDVector4f obstacle_state;
            TML::Vector2f intersection_point;
            //=================================

            /****************************************************************************************
            *  Name     : evaluate_situation_started
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ bool evaluate_situation_started(
                const TML::PDVector4f &ownship_state,
                const TML::PDVector4f &obstacle_state
                )
            {
                return evaluateDistance(ownship_state, obstacle_state) < pars.d_close;
            }

            /****************************************************************************************
            *  Name     : evaluate_colregs_situation
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            __device__ COLREGS_Situation evaluate_colregs_situation(
                const TML::PDVector4f &ownship_state,
                const TML::PDVector4f &obstacle_state
                )
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
            bool evaluate_crossing_port_to_port(
                const TML::PDVector4f &ownshipCPA, 
                const TML::PDVector4f &obstacleCPA
                )
            {
                return relativeBearing(ownshipCPA, obstacleCPA(PX), obstacleCPA(PY)) < 0;
            }

            /****************************************************************************************
            *  Name     : evaluate_crossing_aft
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            bool evaluate_crossing_aft(
                const TML::PDVector4f &ownshipCPA, 
                const TML::PDVector4f &obstacleCPA
                )
            {
                //We are crossing behind if the intersection point is ahead of ownship at CPA
                intersection_point = intersectionpoint(ownshipCPA, obstacleCPA);
                bearing_to_intersection_point = relativeBearing(ownshipCPA, intersection_point(0), intersection_point(1));
                return fabs(bearing_to_intersection_point) > 90 * DEG2RAD;
            }
        };
    }
}
