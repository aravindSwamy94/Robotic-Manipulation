/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#ifndef EXERCISE2_FUNCTIONS_H
#define EXERCISE2_FUNCTIONS_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <fstream>
/** @brief Turn of postprocessing of path after planning by MoveIt */
bool turnOffPathSimplification() {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::Config conf;


	dynamic_reconfigure::BoolParameter param;
	param.name = "simplify_solutions";
	param.value = false;
	conf.bools.push_back(param);

	srv_req.config = conf;
	return ros::service::call("/move_group/ompl/set_parameters", srv_req, srv_resp);
}

/** @brief Return true iff user want to execute trajectory. */
bool askContinue() {
	ROS_INFO_STREAM("Execute [y/N]?");
	const auto ch = getchar();
	getchar(); //capture the enter character
	if (ch != 'y' && ch != 'Y') {
		ROS_ERROR_STREAM("Execution canceled by user.");
		return false;
	}
	return true;
}
/** @brief Reads the file containing grass and store it into a matrix.*/
Eigen::Matrix<double, 2, 3> readGraspFile(const std::string fileName){
	Eigen::Matrix<double, 2, 3> graspMatrix;
	try {
		std::ifstream inFile;
		inFile.open(fileName);
		std::string line;
		int lineNum = 0;
		while (std::getline(inFile, line))
		{
			if(lineNum>1){
				ROS_ERROR_STREAM("Your grasp data file contains more than two lines");
			}
			std::istringstream iss(line);
			double x, y, alpha;
			if(!(iss>> x >> y >> alpha)){
				ROS_ERROR_STREAM("You have some errors with the file " + fileName +". Remember that the file assumes two lines with each line contating three numbers (x position, y position, normal angle) seperated by a space");
				break;
			}
			graspMatrix(lineNum,0) = x;
			graspMatrix(lineNum,1) = y;
			graspMatrix(lineNum,2) = alpha;
			lineNum++;
		}
	}
	catch (const std::ifstream::failure& e) {
		std::cout << "Exception opening/reading file"<<std::endl;
	}
	return graspMatrix;
}
/** @brief Compute gripper trajectory
 *  @param open if true the trajectory will open the gripper; close otherwise
 *  @return empty trajectory on error */
moveit_msgs::RobotTrajectory getGripperTrajectory(moveit::planning_interface::MoveGroupInterface &g_hand, bool open) {
	auto state = g_hand.getCurrentState();
	if (!state) {
		ROS_ERROR_STREAM("Cannot get start state");
		return moveit_msgs::RobotTrajectory();
	}
	state->setVariablePosition("lumi_finger_joint1", open ? 0.04 : 0.0);
	if (!g_hand.setJointValueTarget(*state)) {
		ROS_ERROR_STREAM("Cannot set joint value target");
		return moveit_msgs::RobotTrajectory();
	}
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (!g_hand.plan(plan)) {
		ROS_ERROR_STREAM("Cannot plan to the target");
		return moveit_msgs::RobotTrajectory();
	}
	return plan.trajectory_;
}

/** @brief Plan to the given state. Return empty trajectory on error. */
moveit_msgs::RobotTrajectory planToState(moveit::planning_interface::MoveGroupInterface &g_arm,
		const robot_state::RobotState &state) {
	if (!g_arm.setJointValueTarget(state)) {
		ROS_ERROR_STREAM("Cannot set joint value target");
		return moveit_msgs::RobotTrajectory();
	}
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	if (!g_arm.plan(plan)) {
		ROS_ERROR_STREAM("Cannot plan to the target");
		return moveit_msgs::RobotTrajectory();
	}
	return plan.trajectory_;
}

moveit::planning_interface::MoveGroupInterface::Plan trajectoryToPlan(const moveit_msgs::RobotTrajectory &rtraj) {
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = rtraj;
	return plan;
}

geometry_msgs::TransformStamped createTranform(std::string headerID, std::string childID, Eigen::Matrix4f tMatrix){
	geometry_msgs::TransformStamped static_transformStamped;
	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = headerID;
	static_transformStamped.child_frame_id = childID;
	static_transformStamped.transform.translation.x = tMatrix(0,3);
	static_transformStamped.transform.translation.y = tMatrix(1,3);
	static_transformStamped.transform.translation.z = tMatrix(2,3);
	Eigen::Matrix3f rotMat=  tMatrix.block(0,0,3,3);
	Eigen::Quaternionf q(rotMat);
	static_transformStamped.transform.rotation.x = q.x();
	static_transformStamped.transform.rotation.y = q.y();
	static_transformStamped.transform.rotation.z = q.z();
	static_transformStamped.transform.rotation.w = q.w();
	return static_transformStamped;
}
#endif //EXERCISE2_FUNCTIONS_H
