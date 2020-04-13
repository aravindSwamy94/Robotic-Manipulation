/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *
 *      Brief: Wrapper for MoveIt for the manipulator related staff.
 *    Details: Contains functions to plan between states; interpolate between states; execute different format of trajectories;
 *             transforming transformation representations.
 *
 *             Why additional wrapper and not using MoveGroupInterface directly?
 *             There are some functions missing and different formats required for e.g. planning vs cartesian path.
 *             The goal of this interface is to unify the format to be easier to use/understand for the course.
 */

#ifndef EXERCISE4_ROBOT_H
#define EXERCISE4_ROBOT_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class Robot {
public:

    /** @brief Connect to the MoveGroup and MoveIt visual tools initialisation */
    Robot();

    /** @brief Reset underlying MuJoCo simulation via calling ROS service provided by mujoco_ros_control interface. */
    bool resetSimulation() const;

    /** @brief Clear all visual markers for moveit visual tool in RVIZ */
    void clearAllVisualMarkers();

    /** @brief Get current state of the robot. Throw exception if state cannot be obtained in 10s and print error. */
    robot_state::RobotState getCurrentState();

    /** @brief Construct robot state from current state and the given joint values */
    robot_state::RobotState getStateFromJointValues(const std::array<double, 7> &joint_values);

    /** @brief Compute plan between start and goal state; Return empty trajectory on error and print error. */
    moveit_msgs::RobotTrajectory
    planBetweenStates(const robot_state::RobotState &start, const robot_state::RobotState &goal);

    /** @brief Execute trajectory by calling execute on move group interface */
    bool executeTrajectory(const moveit_msgs::RobotTrajectory& trajectory);

    /** @brief Visualise trajectory in RVIZ */
    void visualiseTrajectory(const moveit_msgs::RobotTrajectory& trajectory);

    /** @brief Call service which will load the controller called \e controller */
    bool loadController(const std::string& controller) const;

    /** @brief Switch controllers */
    bool switchControllers(const std::vector<std::string>& stop, const std::vector<std::string>& start) const;

private:

    /** @brief MoveGroupInterface to arm (lumi_joint_1 .. lumi_joint_7) */
    moveit::planning_interface::MoveGroupInterface move_group_arm;

    /** @brief MoveIt visual tools for trajectories/poses visualisation in RVIZ */
    moveit_visual_tools::MoveItVisualTools moveit_visual;

    /** @brief Internal state used to get joint model group / ee link / etc. */
    robot_state::RobotState state;

};


#endif //EXERCISE4_ROBOT_H
