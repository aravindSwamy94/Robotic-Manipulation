/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#include <exercise6/functions.h>
#include <std_srvs/Empty.h>
#include <chrono>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <exercise6/Robot.h>
#include <exercise6/Exercise6Controller.h>
#include <cmath>

int main(int argc, char **argv) {
    ros::init(argc, argv, "run");  //initialise ros node with name "run"
    ros::NodeHandle node("~"); // create node handle which is used to access params, topics, etc. in the node namespace

    ros::AsyncSpinner spinner(2); //asynchronous spinner used to process callbacks from ros
    spinner.start(); // start the spinner

    Robot r1("lumi1");
    Robot r2("lumi2");
    r1.resetSimulation(); // reset mujoco simulation

    //Create callback for joint_state message; Save end effector position and force into the file f
    auto state = r1.getCurrentState();
    std::ofstream f1("/u/15/swamina1/unix/exercise6_robot1.csv");
    std::ofstream f2("/u/15/swamina1/unix/exercise6_robot2.csv");
    double t0 = NAN;
    boost::function<void(const sensor_msgs::JointState &)> cb_js = [&](const sensor_msgs::JointState &msg) {
        state.setVariablePositions(msg.name, msg.position);
        state.setVariableVelocities(msg.name, msg.velocity);
        state.setVariableEffort(msg.name, msg.effort);
        state.update(true);
        if (std::isnan(t0)) {
            t0 = msg.header.stamp.toSec();
        }
        {
            Eigen::Matrix<double, 7, 1> tau;
            for (size_t i = 0; i < tau.rows(); ++i) {
                tau(i, 0) = state.getVariableEffort("lumi1_joint" + std::to_string(i + 1));
            }

            const Eigen::MatrixXd J = state.getJacobian(state.getJointModelGroup("lumi1_arm"));
            const Eigen::MatrixXd JtpinvL = (J * J.transpose()).inverse() * J;
            const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau;

            f1 << msg.header.stamp.toSec() - t0;
            const Eigen::Isometry3d ee =
                    state.getGlobalLinkTransform("lumi1_link0").inverse() * state.getGlobalLinkTransform("lumi1_ee");
            for (size_t i = 0; i < 3; ++i) {
                f1 << "," << ee.translation()[i];
            }
            for (size_t i = 0; i < wrench.rows(); ++i) {
                f1 << "," << wrench(i, 0);
            }
            f1 << std::endl;
        }
        {
            Eigen::Matrix<double, 7, 1> tau;
            for (size_t i = 0; i < tau.rows(); ++i) {
                tau(i, 0) = state.getVariableEffort("lumi2_joint" + std::to_string(i + 1));
            }
            const Eigen::MatrixXd J = state.getJacobian(state.getJointModelGroup("lumi2_arm"));
            const Eigen::MatrixXd JtpinvL = (J * J.transpose()).inverse() * J;
            const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau;

            f2 << msg.header.stamp.toSec() - t0;
            const Eigen::Isometry3d ee =
                    state.getGlobalLinkTransform("lumi2_link0").inverse() * state.getGlobalLinkTransform("lumi2_ee");
            for (size_t i = 0; i < 3; ++i) {
                f2 << "," << ee.translation()[i];
            }
            for (size_t i = 0; i < wrench.rows(); ++i) {
                f2 << "," << wrench(i, 0);
            }
            f2 << std::endl;
        }
    };
    const auto sub_js = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, cb_js);

    node.setParam("/robot_name", "lumi1");
    r1.loadController("exercise6_lumi1_controller"); //load exercise6 controller
    node.setParam("/robot_name", "lumi2");
    r2.loadController("exercise6_lumi2_controller"); //load exercise6 controller

    r1.switchControllers({"lumi1_trajectory_controller"}, {"exercise6_lumi1_controller"});
    r2.switchControllers({"lumi2_trajectory_controller"}, {"exercise6_lumi2_controller"});

    ros::Duration(100.0).sleep(); //log data

    return 0;

}
