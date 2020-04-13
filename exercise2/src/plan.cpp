/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 1/21/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */
#include <exercise2/functions.h>
#include <std_srvs/Empty.h>
#include <chrono>

using namespace std;


double computePlanLength(vector<Eigen::Vector3d> traj){
    double length=0;
    for (int i=0;i<traj.size()-1;i++)
    {   
            double euclidean_distance = pow(pow(traj[i][0] -traj[i+1][0], 2) + pow(traj[i][1] -traj[i+1][1], 2) +pow(traj[i][2] -traj[i+1][2], 2), 0.5);
            length += euclidean_distance;
    }
    return length;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "plan");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    int planner=0; // iterator for planner
    ofstream log_file;
    log_file.open("Multiple_planner_test_log.csv",ios::out); // Store the data in log files
    string planner_name;
    int planner_time = 0;
    while (planner< 4) // Run all the four planners in the loop and get the logs
    {
        switch(planner)
        {
            case 0:
                planner_name = "PRM";
                break;
            case 1:
                planner_name = "RRT";
                break;
            case 2:
                planner_name = "RRTstar";
                break;
            case 3:
                planner_name = "KPIECE";
                break;
            default:
                break;
        }
        log_file<<planner_name<<endl;
        log_file<<"Planning time,Plan length"<<endl;
        planner_time=0;
        while(planner_time< 5)
        {
            ROS_INFO("RUNNING %s PLANNER for %d time\n",planner_name.c_str(),planner_time);
            std_srvs::Empty srv_reset;
            ros::service::waitForService("/lumi_mujoco/reset");
            ros::service::call("/lumi_mujoco/reset", srv_reset);

            if (!turnOffPathSimplification()) {
                ROS_ERROR_STREAM("Cannot turn off simplification");
                return -1;
            }

            //Load MoveGroup interface and moveit visual tools
            moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
            moveit_visual_tools::MoveItVisualTools vis("base_link");
            vis.loadMarkerPub(true);
            vis.deleteAllMarkers();
            vis.trigger();

            //Get Start state
            const auto state_ptr = g_arm.getCurrentState(10.0);
            if (!state_ptr) {
                ROS_ERROR_STREAM("Cannot get current state");
                return -1;
            }
            auto state = *state_ptr;

            const auto jmg = state.getJointModelGroup("lumi_arm"); //joint model group used for IK computation



            std::array<double, 7> jvalues = {1.15, -1.55, -1.68, -2.43, -0.14, 2.03, 0.68};
            std::array<std::string, 7> jnames = {"lumi_joint1", "lumi_joint2", "lumi_joint3", "lumi_joint4",
                                                 "lumi_joint5", "lumi_joint6", "lumi_joint7"};


            auto start_state = state;
            auto goal_state = state;
            auto trajectory_state = state;
            for (size_t i = 0; i < jnames.size(); ++i) {
                goal_state.setVariablePosition(jnames[i], jvalues[i]);
            }
            
            //todo set different planner, for available planners look at rosed lumi_moveit_config ompl_planning.yaml
            g_arm.setPlannerId(planner_name);
            g_arm.setPlanningTime(5.0); //keep 5s for all planners

            g_arm.setStartState(start_state); //set start_state

            const auto start = std::chrono::steady_clock::now();

            auto plan = planToState(g_arm, goal_state); //compute plan form start to goal_state

            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
            //std::cout << "Computation time: " << dt.count() << " [ms]" << std::endl;
            ROS_INFO("Computation Time: %d\n [ms]",dt.count());
            vis.publishTrajectoryLine(plan, state.getLinkModel("lumi_ee"), jmg); //visualise plan
            vis.trigger();
            
            std::vector<Eigen::Vector3d> trajecory_points_in_cartesian;


            for(int i =0;i<plan.joint_trajectory.points.size();i++)
            {
                trajectory_state.setVariablePositions(plan.joint_trajectory.joint_names,plan.joint_trajectory.points.at(i).positions);
                Eigen::Affine3d current_end_effector_state = trajectory_state.getGlobalLinkTransform("lumi_ee");
                trajecory_points_in_cartesian.push_back(current_end_effector_state.translation());
            }
            
            double planLength = computePlanLength(trajecory_points_in_cartesian);
            cout<<"Plan length is "<<planLength<<endl;
            log_file<<dt.count()<<","<<planLength<<endl;
            g_arm.execute(trajectoryToPlan(plan));
            ros::Duration(8.0).sleep();// Increased the sleep time to give space and time between one trial and the other.
            planner_time++;
        }
        log_file<<"\n"<<endl;
        planner ++;
    }
    return 0;
}
