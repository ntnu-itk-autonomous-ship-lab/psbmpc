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
*  Copyright (C) 2020 Trym Tengesdal, NTNU Trondheim.
*  All rights reserved.
*
*  Author    : Trym Tengesdal
*
*  Modified  :
*
*****************************************************************************************/
#pragma once

#include "geometry.hpp"

#include <Eigen/Dense>
#include <optional>

//Evaluates colregs violation between a pair of ships
class COLREGS_Violation_Evaluator
{
public:
    COLREGS_Violation_Evaluator() = default;

    COLREGS_Violation_Evaluator(const COLREGS_Violation_Evaluator &other) = default;

    COLREGS_Violation_Evaluator &operator=(const COLREGS_Violation_Evaluator &rhs) = default;

    void update(const Eigen::VectorXd &ownship_state, const Eigen::Vector4d &obstacle_state)
    {
        if (evaluate_situation_started(ownship_state, obstacle_state))
        {
            colregs_situation = evaluate_colregs_situation(ownship_state, obstacle_state);
            initial_ownship_state = ownship_state;
            initial_obstacle_state = obstacle_state;
        }
    }

    bool evaluate_SO_violation(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
    {
        if (!initial_ownship_state.has_value())
            return false;

        bool distance_larger_than_critical = std::abs((initial_ownship_state->head(2) - initial_ownship_state->head(2)).norm()) > critical_distance_to_ignor_SO;
        bool stands_on_correct = evaluate_course_change(ownship_trajectory) == CourseChange::None && evaluate_speed_change(ownship_trajectory) == SpeedChange::None;
        bool has_SO_role = colregs_situation == OT_en || colregs_situation == CR_PS;
        bool is_risk_of_collision = evaluate_risk_of_collision(ownship_trajectory, obstacle_trajectory);
        return is_risk_of_collision && distance_larger_than_critical && has_SO_role && !stands_on_correct;
    }

    bool evaluate_GW_violation(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
    {
        if (!initial_ownship_state.has_value() || !initial_obstacle_state.has_value())
            return false;

        bool correct_HO_maneuver = evaluate_crossing_port_to_port(ownship_trajectory, obstacle_trajectory);
        bool correct_CR_SS_maneuver = evaluate_crossing_aft(ownship_trajectory, obstacle_trajectory);
        bool correct_CR_PS_maneuver = evaluate_course_change(ownship_trajectory) != CourseChange::Portwards;
        bool is_risk_of_collision = evaluate_risk_of_collision(ownship_trajectory, obstacle_trajectory);
        return  is_risk_of_collision                                        && 
                ((colregs_situation == HO && !correct_HO_maneuver)          || 
                (colregs_situation == CR_SS && !correct_CR_SS_maneuver)     || 
                (colregs_situation == CR_PS && !correct_CR_PS_maneuver));
    }

private:
    // These should be able to be set externally?
    static constexpr double colregs_distance = 100;
    static constexpr double colregs_start_time = 100;
    static constexpr double head_on_width = 10 * DEG2RAD;
    static constexpr double overtaking_angle = (90 + 22.5) * DEG2RAD;
    static constexpr double max_acceptable_SO_speed_change = 1;
    static constexpr double max_acceptable_SO_course_change = 10 * DEG2RAD;
    static constexpr double critical_distance_to_ignor_SO = 5;

    enum COLREGS_Situation
    {
        HO,
        OT_ing,
        OT_en,
        CR_PS,
        CR_SS
    };

    std::optional<COLREGS_Situation> colregs_situation;
    std::optional<Eigen::VectorXd> initial_ownship_state;
    std::optional<Eigen::Vector4d> initial_obstacle_state;

    bool evaluate_situation_started(const Eigen::VectorXd &ownship_state, const Eigen::Vector4d &obstacle_state) const
    {
        return evaluateCPA(ownship_state, obstacle_state).time_untill_CPA < colregs_start_time;
    }

    COLREGS_Situation evaluate_colregs_situation(const Eigen::VectorXd &ownship_state, const Eigen::Vector4d &obstacle_state) const
    {
        const double heading_diff = wrapPI(obstacle_state(CHI) - ownship_state(CHI));
        if (heading_diff < -M_PI + head_on_width / 2 || heading_diff > M_PI - head_on_width / 2)
            return HO;

        const double bearing_to_obstacle_relative_to_ownship = relativeBearing(ownship_state, obstacle_state(PX), obstacle_state(PY));
        if (bearing_to_obstacle_relative_to_ownship > overtaking_angle || bearing_to_obstacle_relative_to_ownship < -overtaking_angle)
            return OT_en;

        const double bearing_to_ownship_relative_to_obstacle = relativeBearing(obstacle_state, ownship_state(PX), ownship_state(PY));
        if (bearing_to_ownship_relative_to_obstacle > overtaking_angle || bearing_to_ownship_relative_to_obstacle < -overtaking_angle)
            return OT_ing;

        if (heading_diff < 0)
            return CR_PS;

        return CR_SS;
    }

    bool evaluate_risk_of_collision(const Eigen::MatrixXd &ownship_trajectory, const Eigen::MatrixXd &obstacle_trajectory) const
    {
        auto CPA = evaluateCPA(ownship_trajectory, obstacle_trajectory);
        return CPA.closest_distance < colregs_distance;
    }
    enum class CourseChange
    {
        Portwards,
        None,
        Starboardwards
    };

    CourseChange evaluate_course_change(const Eigen::MatrixXd &ownship_trajectory) const
    {
        for (int i = 0; i < ownship_trajectory.cols(); ++i)
        {
            if (ownship_trajectory(CHI, i) - initial_ownship_state.value()(CHI) < max_acceptable_SO_course_change)
                return CourseChange::Portwards;
            if (ownship_trajectory(CHI, i) - initial_ownship_state.value()(CHI) > max_acceptable_SO_course_change)
                return CourseChange::Starboardwards;
        }
        return CourseChange::None;
    }

    enum class SpeedChange
    {
        Lower,
        None,
        Higher
    };
    SpeedChange evaluate_speed_change(const Eigen::MatrixXd &ownship_trajectory) const
    {
        for (int i = 0; i < ownship_trajectory.cols(); ++i)
        {
            if (ownship_trajectory(CHI, i) - initial_ownship_state.value()(U) > max_acceptable_SO_speed_change)
                return SpeedChange::Higher;
            if (ownship_trajectory(CHI, i) - initial_ownship_state.value()(U) < max_acceptable_SO_speed_change)
                return SpeedChange::Lower;
        }
        return SpeedChange::None;
    }
};
