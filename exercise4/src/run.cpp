/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise4/functions.h>
#include <std_srvs/Empty.h>
#include <chrono>

#include <exercise4/Robot.h>
#include <exercise4/Exercise4Controller.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "run");  //initialise ros node with name "run"
    ros::NodeHandle node("~"); // create node handle which is used to access params, topics, etc. in the node namespace

    ros::AsyncSpinner spinner(2); //asynchronous spinner used to process callbacks from ros
    spinner.start(); // start the spinner

    Robot r; //create robot object; the object is used to interface the MoveIt interfaces for planning/executing/etc.
    r.resetSimulation(); // reset mujoco simulation

    const auto start_state = r.getCurrentState();
    const auto goal_state = r.getStateFromJointValues({0.0, 0.09, 0.0, -2.13, 0.0, 2.23, 0.0});

    const auto plan = r.planBetweenStates(start_state, goal_state);
    if (plan.joint_trajectory.points.empty()) { // cannot plan
        return -1;
    }

    //Create callback for joint_state message; Save end effector position and force into the file f
    auto state = r.getCurrentState();
    std::ofstream f("/u/15/swamina1/unix/Exercises/4/Catkin_Ws/exercise4.csv");
    double t0 = NAN;
    boost::function<void(const sensor_msgs::JointState &)> cb_js = [&](const sensor_msgs::JointState &msg) {
        state.setVariablePositions(msg.name, msg.position);
        state.setVariableVelocities(msg.name, msg.velocity);
        state.setVariableEffort(msg.name, msg.effort);
        state.update(true);

        Eigen::Matrix<double, 7, 1> tau;
        for (size_t i = 0; i < tau.rows(); ++i) {
            tau(i, 0) = state.getVariableEffort("lumi_joint" + std::to_string(i + 1));
        }

        const Eigen::MatrixXd J = state.getJacobian(state.getJointModelGroup("lumi_arm"));
        const Eigen::MatrixXd JtpinvL = (J * J.transpose()).inverse() * J;
        const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau;

        if (std::isnan(t0)) {
            t0 = msg.header.stamp.toSec();
        }
        f << msg.header.stamp.toSec() - t0;
        const Eigen::Isometry3d ee = state.getGlobalLinkTransform("lumi_ee");
        for (size_t i = 0; i < 3; ++i) {
            f << "," << ee.translation()[i];
        }
        for (size_t i = 0; i < wrench.rows(); ++i) {
            f << "," << wrench(i, 0);
        }
        f << std::endl;
    };
    const auto sub_js = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, cb_js);

    r.visualiseTrajectory(plan);
    if (!r.executeTrajectory(plan)) {
        ROS_ERROR_STREAM("Cannot execute trajectory");
        return -1;
    }

    r.loadController("exercise4_controller"); //load exercise4 controller
    r.switchControllers({"trajectory_controller"},
                        {"exercise4_controller"}); //stop trajectory controller and load exercise4 controller

    ros::Duration(10.0).sleep(); //log data for 10more seconds

    return 0;
}
