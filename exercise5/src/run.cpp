/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */
#define _USE_MATH_DEFINES
#include <exercise5/functions.h>
#include <std_srvs/Empty.h>
#include <chrono>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <exercise5/Robot.h>
#include <exercise5/Exercise5Controller.h>
#include <cmath>
int main(int argc, char **argv) {
    ros::init(argc, argv, "run");  //initialise ros node with name "run"
    ros::NodeHandle node("~"); // create node handle which is used to access params, topics, etc. in the node namespace

    ros::AsyncSpinner spinner(2); //asynchronous spinner used to process callbacks from ros
    spinner.start(); // start the spinner

    std::string grasp_data_file = "";
    node.getParam("/grasp_data_file", grasp_data_file); // Reads in the name of the grasp_data.txt

    double grasp_force = 0;
    node.getParam("/grasp_force", grasp_force);

    double grasp_height = 0.1; // The default grasping grasp_height which in this case is half of the object height
    node.getParam("/grasp_height", grasp_height);

    Eigen::Matrix<double, 2, 3> graspData = readGraspFile(grasp_data_file); // Read in grasp data
    Robot r1("lumi1"); //create robot object for lumi1, i.e. one of the robots; the object is used to interface the MoveIt interfaces for planning/executing/etc.
    Robot r2("lumi2"); //create robot object for lumi2; the object is used to interface the MoveIt interfaces for planning/executing/etc.
    r1.resetSimulation(); // reset mujoco simulation

    Eigen::AngleAxisd xRot(M_PI, Eigen::Vector3d::UnitX());
    double epsilon = 0.18;
    double alpha1 = graspData(0,2); 
      // converting degrees to radians
    alpha1 *= M_PI/180;
    Eigen::AngleAxisd yRotLumi1(M_PI/2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd x2RotLumi1(alpha1, Eigen::Vector3d::UnitX());

    double x1 = cos(alpha1);
    double y1 = sin(alpha1);
    const auto start_state1 = r1.getCurrentState();
    Eigen::Quaternion<double> q1 = xRot * yRotLumi1 * x2RotLumi1; // The default pose has z-direction downwards. The code, however, expect the z-direction to point in the grasp approach direction. To achieve this, we first rotate 180 degrees around x-axis, then 90 degrees around y-axis, and finally alpha_1 degrees around x-axis
                                        
                                                             

    geometry_msgs::Pose goal_state1;
    goal_state1.orientation.x = q1.x();
    goal_state1.orientation.y = q1.y();
    goal_state1.orientation.z = q1.z();
    goal_state1.orientation.w = q1.w();
    goal_state1.position.x = graspData(0,0)-epsilon*x1; // As the grasp position is located exactly on the object we need to plan a pre-specified amount backward from it to avois collision
    goal_state1.position.y = graspData(0,1)-epsilon*y1;
    goal_state1.position.z = grasp_height;

    double alpha2 = graspData(1,2); 
    alpha2 *= M_PI/180;
    Eigen::AngleAxisd yRotLumi2(M_PI/2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd zRotLumi2(alpha2, Eigen::Vector3d::UnitX());

    double x2 = cos(alpha2);
    double y2 = sin(alpha2);
    const auto start_state2 = r2.getCurrentState();
    Eigen::Quaternion<double> q2= xRot * yRotLumi2 * zRotLumi2;

    geometry_msgs::Pose goal_state2;
    goal_state2.orientation.x = q2.x();
    goal_state2.orientation.y = q2.y();
    goal_state2.orientation.z = q2.z();
    goal_state2.orientation.w = q2.w();
    goal_state2.position.x = graspData(1,0)-epsilon*x2;
    goal_state2.position.y = graspData(1,1)-epsilon*y2;
    goal_state2.position.z = grasp_height;


    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.publishAxisLabeled(goal_state1, "pose1");
    visual_tools.publishAxisLabeled(goal_state2, "pose2");
    visual_tools.trigger();

    const auto plan1= r1.planToPose(start_state1, goal_state1);
    const auto plan2= r2.planToPose(start_state2, goal_state2);
    if (plan1.joint_trajectory.points.empty() or plan2.joint_trajectory.points.empty()) { // cannot plan
        ROS_ERROR_STREAM("Cannot plan for lumi1 or lumi2");
        return -1;
    }


    r1.visualiseTrajectory(plan1);
    r2.visualiseTrajectory(plan2);
    if (!r1.executeTrajectory(plan1) or !r2.executeTrajectory(plan2)) {
        ROS_ERROR_STREAM("Cannot execute trajectory");
        return -1;
    }

    float force_vector_x1 = cos(M_PI+alpha1); // Lumi1 is rotated 180 degrees around z-axis. Here, we rotate it back to have the same orientation as the base_link as it is in this frame we add the forces.
    float force_vector_y1 = sin(M_PI+alpha1);
    std::vector<double> F{force_vector_x1, force_vector_y1, 0};
    node.setParam("/lumi1_force_vector", F); 
    node.setParam("/robot_name", "lumi1");
    r1.loadController("exercise5_lumi1_controller"); //load exercise5 controller
    r1.switchControllers({"lumi1_trajectory_controller"},
                        {"exercise5_lumi1_controller"}); //stop trajectory controller and load exercise5 controller
    F[0] = grasp_force*x2;
    F[1] = grasp_force*y2;
    F[2] = 0;

    node.setParam("/lumi2_force_vector", F); 
    node.setParam("/robot_name", "lumi2");
    r2.loadController("exercise5_lumi2_controller"); //load exercise5 controller
    r2.switchControllers({"lumi2_trajectory_controller"},
                        {"exercise5_lumi2_controller"}); //stop trajectory controller and load exercise5 controller

    ros::Duration(5.0).sleep();
   return 0;
}
