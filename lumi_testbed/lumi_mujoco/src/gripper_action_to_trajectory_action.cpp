/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 11/23/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *     This node subsribe to Gripper command action and forward all commands to the JointTrajectoryAction as required
 *     by mujoco simulation. It mimic individual fingers.
 */


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


using namespace control_msgs;
using actionlib::SimpleActionServer;
using actionlib::SimpleActionClient;

class ForwardAction {
public:
    ForwardAction() : node("~"),
                      ac(node, "follow_joint_trajectory"),
                      as(node, "gripper_action", boost::bind(&ForwardAction::execute, this, _1), false) {
        as.start();
        node.getParam("robot_name", robot_name);
    }

    void execute(const GripperCommandGoalConstPtr &goal) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(2, goal->command.position);
        point.effort.resize(2, goal->command.max_effort);
        point.time_from_start = ros::Duration(2.0);
        point.accelerations.resize(2, 0.01);
        point.velocities.resize(2, 0.01);

        FollowJointTrajectoryGoal t;
        t.trajectory.joint_names.push_back(robot_name+"_finger_joint1");
        t.trajectory.joint_names.push_back(robot_name+"_finger_joint2");
        t.trajectory.points.push_back(point);

        const auto state = ac.sendGoalAndWait(t, ros::Duration(60.0), ros::Duration(60.0));
        if (state == state.SUCCEEDED) {
            as.setSucceeded();
        } else {
            as.setAborted();
        }
    }

private:
    ros::NodeHandle node;
    SimpleActionServer<GripperCommandAction> as;
    SimpleActionClient<FollowJointTrajectoryAction> ac;
    std::string robot_name;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_action_to_trajectory_action");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ForwardAction forward_action;
    ros::waitForShutdown();

    return 0;
}
