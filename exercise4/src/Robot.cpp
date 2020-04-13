/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise4/Robot.h>
#include <std_srvs/Empty.h>
#include <eigen_conversions/eigen_msg.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>

Robot::Robot() : move_group_arm("lumi_arm"), moveit_visual("base_link"), state(getCurrentState()) {
    moveit_visual.loadMarkerPub(true);
    clearAllVisualMarkers();
}

void Robot::clearAllVisualMarkers() {
    moveit_visual.deleteAllMarkers();
    moveit_visual.trigger();
}

bool Robot::resetSimulation() const {
    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset");
    return ros::service::call("/lumi_mujoco/reset", srv_reset);
}

robot_state::RobotState Robot::getCurrentState() {
    const auto state_ptr = move_group_arm.getCurrentState(10.0);
    if (!state_ptr) {
        ROS_ERROR_STREAM("Cannot get current robot state");
        throw std::runtime_error("Cannot get current robot state");
    }
    return *state_ptr;
}

robot_state::RobotState Robot::getStateFromJointValues(const std::array<double, 7> &joint_values) {
    auto s = state;
    for (size_t i = 0; i < joint_values.size(); ++i) {
        s.setVariablePosition(std::string("lumi_joint") + std::to_string(i + 1), joint_values[i]);
    }
    return s;
}

moveit_msgs::RobotTrajectory
Robot::planBetweenStates(const robot_state::RobotState &start, const robot_state::RobotState &goal) {
    move_group_arm.setStartState(start);
    if (!move_group_arm.setJointValueTarget(goal)) {
        ROS_ERROR_STREAM("Cannot set joint value target for planning");
        return moveit_msgs::RobotTrajectory();
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!move_group_arm.plan(plan)) {
        ROS_ERROR_STREAM("Cannot plan between given states");
        return moveit_msgs::RobotTrajectory();
    }
    return plan.trajectory_;
}

bool Robot::executeTrajectory(const moveit_msgs::RobotTrajectory &trajectory) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return (bool) move_group_arm.execute(plan);
}

void Robot::visualiseTrajectory(const moveit_msgs::RobotTrajectory &trajectory) {
    moveit_visual.publishTrajectoryLine(trajectory, state.getLinkModel("lumi_ee"),
                                        state.getJointModelGroup("lumi_arm"));
    moveit_visual.trigger();
}

bool Robot::loadController(const std::string &controller) const {
    controller_manager_msgs::LoadController srv;
    srv.request.name = controller;
    ros::service::waitForService("/lumi_mujoco/controller_manager/load_controller");
    auto suc = ros::service::call("/lumi_mujoco/controller_manager/load_controller", srv);
    return suc && srv.response.ok;
}

bool Robot::switchControllers(const std::vector<std::string> &stop, const std::vector<std::string> &start) const {
    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers = stop;
    srv.request.start_controllers = start;
    srv.request.strictness = 1;
    ros::service::waitForService("/lumi_mujoco/controller_manager/switch_controller");
    auto suc = ros::service::call("/lumi_mujoco/controller_manager/switch_controller", srv);
    return suc && srv.response.ok;
}

