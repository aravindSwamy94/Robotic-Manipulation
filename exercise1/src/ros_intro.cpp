/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 11/23/18
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
using namespace std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_intro");
    ros::NodeHandle node("~");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // This is the geometry message that you will populate with the lumi_ee to base_link transformation
    geometry_msgs::TransformStamped ee_to_base_pose;

    // Create a publisher that publishes the transformation to the topic tf_pose    
    ros::Publisher tf_pub =node.advertise<geometry_msgs::TransformStamped>("/tf_pose", 1000);

    // Set the ros loop rate to 25 hz
    ros::Rate loop_rate(25);

    while (ros::ok())
    {
        try {
        //  Read the transformation from lumi_ee to base_link into the variable ee_to_base_pose
        //  Statement Inside the try{} block, so that if any error occurs, it will throw an exception and error is catched in catch{} block
        //  ros::Time(0) to get the latest transform and ros::Duration(5.0) to give duration before timeout.
            ee_to_base_pose = tfBuffer.lookupTransform("lumi_ee", "base_link",ros::Time(0),ros::Duration(5.0));               

        //  Publish the recently read transformation
            tf_pub.publish(ee_to_base_pose);
        } catch (tf2::TransformException &ex) {
            ROS_ERROR_STREAM("Cannot find transformation from lumi_ee to base_link. What: " << ex.what());
            return -1;
        }
        ros::spinOnce();
        loop_rate.sleep();// Added the state to maintain the 25Hz publish rate.
    }
    return 0;
}
