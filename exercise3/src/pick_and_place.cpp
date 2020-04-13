/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Modified on 08/01/20
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#include <exercise3/functions.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Wait for the simulator to come online and then we reset it
  std_srvs::Empty srv_reset;
  ros::service::waitForService("/lumi_mujoco/reset");
  ros::service::call("/lumi_mujoco/reset", srv_reset);

  // read poses from tf
  geometry_msgs::TransformStamped base_to_pick,base_to_place;
  base_to_pick = tfBuffer.lookupTransform("base_link", "pick",ros::Time(0),ros::Duration(5.0));
  base_to_place = tfBuffer.lookupTransform("base_link", "place",ros::Time(0),ros::Duration(5.0));
  
  // convert poses to Eigen
  
  Eigen::Isometry3d preGrasp,grasp,place;
  tf::transformMsgToEigen(base_to_pick.transform,grasp);
  tf::transformMsgToEigen(base_to_place.transform,place);

  // Load MoveGroup interface and moveit visual tools
  moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
  moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
  moveit_visual_tools::MoveItVisualTools vis("base_link");
  vis.loadMarkerPub(true);
  vis.deleteAllMarkers();
  vis.trigger();

  // Get Start state
  const robot_state::RobotStatePtr state_ptr = g_arm.getCurrentState(10.0);
  if (!state_ptr) {
    ROS_ERROR_STREAM("Cannot get current state");
    return -1;
  }
  robot_state::RobotState state = *state_ptr;

  const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(
      "lumi_arm"); // joint model group used for IK computation

  const std::string ee_link = "lumi_ee"; // Name of the end effector link
  const Eigen::Isometry3d arm_to_ee =
      state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
      state.getGlobalLinkTransform(
          ee_link); // Transformation from base link to end effector link

  // compute required poses and visualize them

  Eigen::Isometry3d e1 = state.getGlobalLinkTransform(ee_link) *
                         Eigen::Translation3d(0.1, 0.0, 0.0);
  Eigen::Isometry3d e2 = state.getGlobalLinkTransform(ee_link) *
                         Eigen::Translation3d(-0.1, 0.0, 0.0) *
                         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());


  preGrasp = grasp * Eigen::Translation3d(0.0, 0.0, 0.1)* Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  grasp = grasp * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  place = place * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  cout << "pre grasp frame"<<preGrasp.matrix()<<endl;
  cout << "grasp frame"<<grasp.matrix()<<endl;
  cout << "place frame"<<place.matrix()<<endl;

  vis.publishAxis(preGrasp);
  vis.publishAxis(grasp);
  vis.publishAxis(place);
  vis.trigger();
  
  std::vector<moveit_msgs::RobotTrajectory> trajectories;

  cout << " g_arm planning frame"<<g_arm.getPlanningFrame().c_str()<<endl;
  // planning to pose
  // Planning from current state to pre grasp pose.
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, preGrasp, ee_link)) {
    ROS_ERROR_STREAM("Cannot set arm position with IK");
    return -1;
  }
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  }
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // Cartesian path from Pre grasp pose to Grasp pose - Cartesian path
  g_arm.setStartState(state);
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(grasp * arm_to_ee.inverse(), pose);
  moveit_msgs::RobotTrajectory rtraj;
  const double d1 = g_arm.computeCartesianPath({pose}, 0.01, 1.4, rtraj);
  if (d1 < 0.99) {
    ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
    return -1;
  }
  trajectories.push_back(rtraj);

  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // Cartesian path from Grasp to pre grasp - Cartesian path

  g_arm.setStartState(state);
  tf::poseEigenToMsg(preGrasp * arm_to_ee.inverse(), pose);
  //moveit_msgs::RobotTrajectory rtraj;
  const double d2 = g_arm.computeCartesianPath({pose}, 0.01, 1.4, rtraj);
  if (d2 < 0.99) {
    ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
    return -1;
  }
  trajectories.push_back(rtraj);


  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);
  // Path from Pre grasp to Place 
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, place, ee_link)) {
    ROS_ERROR_STREAM("Cannot set arm position with IK");
    return -1;
  }
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  }
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // Visualise all trajectories
  for (const moveit_msgs::RobotTrajectory &t : trajectories) {
    vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
  }
  vis.trigger();
  moveit_msgs::RobotTrajectory grip_traj;
  g_hand.setStartStateToCurrentState();
  grip_traj = getGripperTrajectory(g_hand, true);
  if (!grip_traj.joint_trajectory.points.empty()) {
    g_hand.execute(trajectoryToPlan(grip_traj));
  }
  if (askContinue()) {
    int index=0;
    for(std::vector<moveit_msgs::RobotTrajectory>::iterator it=trajectories.begin();it!= trajectories.end();++it){
      if(index == 2){ // close the gripper when we go to the grasp position
        g_hand.setStartStateToCurrentState();
        grip_traj = getGripperTrajectory(g_hand, false);
        if (!grip_traj.joint_trajectory.points.empty()) {
          g_hand.execute(trajectoryToPlan(grip_traj));
        }
      }
      g_arm.execute(trajectoryToPlan(*it));
      index++;
    }
    cout<< "Done executing all the trajectories"<<endl;
    //g_hand.setJointValueTarget(*g_hand.getCurrentState(10.0));
    grip_traj = getGripperTrajectory(g_hand, true);
    if (!grip_traj.joint_trajectory.points.empty()) {
      g_hand.execute(trajectoryToPlan(grip_traj));
    }
  } else {
    cout<<" Try again from the start" ;
  }

/*  // open gripper
  g_hand.setStartStateToCurrentState();
  // Information about getGripperTrajectory can be found in
  // exercise3/include/exercise3/functions.h
  const moveit_msgs::RobotTrajectory traj = getGripperTrajectory(g_hand, true);
  if (!traj.joint_trajectory.points.empty()) {
    g_hand.execute(trajectoryToPlan(traj));
  }
*/
  return 0;
}
