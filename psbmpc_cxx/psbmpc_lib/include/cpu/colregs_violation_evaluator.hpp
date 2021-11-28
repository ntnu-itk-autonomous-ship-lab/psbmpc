/****************************************************************************************
*
*  File name : colregs_violation_evaluator.hpp
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
#include "cpu/geometry.hpp"

#include <Eigen/Dense>
#include <optional>
#include <cmath>

namespace PSBMPC_LIB
{
    namespace CPU
    {

        //NOTE! Ownship has states [X, Y, CHI, u], while obstacle ship has [X, Y, VX, VY]......
        //This greatly increases the chance of bugs and makes this code difficult to check.
        class COLREGS_Violation_Evaluator
        {
        private:
            
            CVE_Pars pars;

            std::optional<COLREGS_Situation> colregs_situation;
            std::optional<Eigen::Vector4d> initial_ownship_state;
            std::optional<Eigen::Vector4d> initial_obstacle_state;

            /****************************************************************************************
            *  Name     : evaluate_situation_started
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            bool evaluate_situation_started(const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state) const
            {
                return evaluateDistance(ownship_state, obstacle_state) < pars.d_close;
            }

            /****************************************************************************************
            *  Name     : evaluate_colregs_situation
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            COLREGS_Situation evaluate_colregs_situation(const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state) const
            {
                const double heading_diff = wrapPI(obstacle_state(COG) - ownship_state(COG));
                if (heading_diff < -M_PI + pars.head_on_width / 2 || heading_diff > M_PI - pars.head_on_width / 2)
                    return HO;

                const double bearing_to_obstacle_relative_to_ownship = relativeBearing(ownship_state, obstacle_state(PX), obstacle_state(PY));
                if (bearing_to_obstacle_relative_to_ownship > pars.overtaking_angle || bearing_to_obstacle_relative_to_ownship < -pars.overtaking_angle)
                    return OT_en;

                const double bearing_to_ownship_relative_to_obstacle = relativeBearing(obstacle_state, ownship_state(PX), ownship_state(PY));
                if (bearing_to_ownship_relative_to_obstacle > pars.overtaking_angle || bearing_to_ownship_relative_to_obstacle < -pars.overtaking_angle)
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
            bool evaluate_risk_of_collision(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
            {
                auto CPA = evaluateCPA(ownship_trajectory, obstacle_trajectory);
                return CPA.closest_distance < pars.max_distance_at_cpa;
            }

            /****************************************************************************************
            *  Name     : evaluate_course_change
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            CourseChange evaluate_course_change(const Eigen::MatrixXd &ownship_trajectory) const
            {
                for (int i = 0; i < ownship_trajectory.cols(); ++i)
                {
                    if (ownship_trajectory(COG, i) - initial_ownship_state.value()(COG) < -pars.max_acceptable_SO_course_change){
                        return CourseChange::Portwards;
                    }
                    if (ownship_trajectory(COG, i) - initial_ownship_state.value()(COG) > pars.max_acceptable_SO_course_change){
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
            SpeedChange evaluate_speed_change(const Eigen::MatrixXd &ownship_trajectory) const
            {
                //Seems like the trajectories are slowing down towards the end as they are approaching the final wp, so im only considering the first half of the traj as there shouldnt be any changes in speed or course after that
                for (int i = 0; i < ownship_trajectory.cols()/2; ++i)
                {
                    if (ownship_trajectory(SOG, i) - initial_ownship_state.value()(SOG) > pars.max_acceptable_SO_speed_change)
                        return SpeedChange::Higher;
                    if (ownship_trajectory(SOG, i) - initial_ownship_state.value()(SOG) < -pars.max_acceptable_SO_speed_change)
                        return SpeedChange::Lower;
                }
                return SpeedChange::None;
            }

        public:

            /****************************************************************************************
            *  Name     : COLREGS_Violation_Evaluator
            *  Function : Class constructor
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            COLREGS_Violation_Evaluator() = default;

            COLREGS_Violation_Evaluator(const CVE_Pars &pars) : pars(pars) {}

            COLREGS_Violation_Evaluator(const COLREGS_Violation_Evaluator &other) = default;


            /****************************************************************************************
            *  Name     : operator=
            *  Function : Assignment operator
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            COLREGS_Violation_Evaluator &operator=(const COLREGS_Violation_Evaluator &rhs) = default;

            /****************************************************************************************
            *  Name     : update
            *  Function : Determines if the COLREGS situation has started, sets initial states if
            *             that is the case
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            void update(const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state_vx_vy)
            {
                const auto obstacle_state = vx_vy_to_heading_speed_state(obstacle_state_vx_vy);
                if (evaluate_situation_started(ownship_state, obstacle_state))
                {
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
            bool evaluate_SO_violation(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
            {
                if (!initial_ownship_state.has_value())
                    return false;

                const bool distance_larger_than_critical = std::abs((ownship_trajectory.col(0).head(2) - obstacle_trajectory.col(0).head(2)).norm()) > pars.critical_distance_to_ignore_SO;
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
            *  Function : 
            *  Author   :
            *  Modified :
            *****************************************************************************************/
            bool evaluate_GW_violation(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
            {
                if (!initial_ownship_state.has_value() || !initial_obstacle_state.has_value())
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
        };
    }
}