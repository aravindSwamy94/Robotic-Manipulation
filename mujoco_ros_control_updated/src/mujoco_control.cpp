/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <mujoco.h>
#include <mujoco_ros_control/RobotHWMujoco.h>
#include <mujoco_ros_control/RenderImage.h>
#include <image_transport/image_transport.h>
#include <controller_manager/controller_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/conversions.h>

std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/** @brief Update controller manager and current time. If curent time is not initialized than call reset. */
void cb_controller(const mjModel *m, mjData *d, bool reset_controller = false) {
    hw->read(*d);
    cm->update(ros::Time::now(), ros::Duration(m->opt.timestep), reset_controller);
    hw->write(*d);
}

void reset_joints(ros::NodeHandle &node, const mjModel &m, mjData &d) {
    const auto n = (size_t) m.njnt;
    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL) {
            continue;
        }

        const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);
        const auto value = node.param(std::string(joint_name) + "_init", 0.0);
        d.qpos[(size_t) m.jnt_qposadr[i]] = value;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.urdf";
    const auto model_path = node.param("model", default_model_path);

    // The camera look at vector defines direction of the camera which it points to.
    const float default_camera_lookat_at[3] = {0, 0, -0.051};

    float camera_look_at[3];
    camera_look_at[0] = node.param<float>("look_at_x", default_camera_lookat_at[0]);
    camera_look_at[1] = node.param<float>("look_at_y", default_camera_lookat_at[1]);
    camera_look_at[2] = node.param<float>("look_at_z", default_camera_lookat_at[2]);

    const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mjpro200/bin/mjkey.txt";

    if (!mj_activate(key_path.c_str())) {
        ROS_ERROR_STREAM("Cannot activate mujoco with key: " << key_path);
        return -1;
    }

    char error[1000] = "";
    auto m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        ROS_ERROR_STREAM("Cannot load model: " << model_path);
        ROS_ERROR_STREAM(error);
        return -1;
    }

    auto d = mj_makeData(m);
    if (!d) {
        ROS_ERROR_STREAM("Cannot make data structure for model.");
        return -1;
    }

    hw.reset(new RobotHWMujoco(*m));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));
    mjcb_control = [](const mjModel *m, mjData *d) { cb_controller(m, d); };

    hw->compensate_bias = node.param("compensate_bias", false);
    hw->bias_error = node.param("bias_error", 1.0);

    reset_joints(node, *m, *d);
    std::mutex mutex_data;

    const auto timer = node.createTimer(ros::Duration(m->opt.timestep), [&](const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> l(mutex_data);
        mj_step(m, d);
    });

    boost::function<bool(std_srvs::Empty::Request &, std_srvs::Empty::Response &)> reset =
            [&](std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
                std::lock_guard<std::mutex> l(mutex_data);
                mj_resetData(m, d);
                reset_joints(node, *m, *d);
                cb_controller(m, d, true);
                return true;
            };
    const auto reset_server = node.advertiseService("reset", reset);
    const auto init_reset_delay = node.param("mujoco_initial_reset_delay", 1.0);
    const auto initial_reset = node.createTimer(ros::Duration(init_reset_delay), [&](const ros::TimerEvent &e) {
        std_srvs::Empty srv;
        reset(srv.request, srv.response);
    }, true);

    image_transport::ImageTransport it(node);
    image_transport::Publisher pub_rgb = it.advertise("rgb", 1);
    image_transport::Publisher pub_depth = it.advertise("depth", 1);
    ros::Publisher pub_pc = node.advertise<PointCloud>("points2", 1);

    const auto timer_rendered = node.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &e) {
        //thread_local RenderImage renderer(m);
        thread_local RenderImage renderer(m, camera_look_at);

        if ((pub_rgb.getNumSubscribers() == 0) && (pub_depth.getNumSubscribers() == 0)) {
            return;
        }

        {
            std::lock_guard<std::mutex> l(mutex_data);
            renderer.updateScene(m, d);
        }
        cv::Mat rgb, depth;
        std::tie(rgb, depth) = renderer.render();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = renderer.RGBDtoPCL(depth);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (pub_rgb.getNumSubscribers() != 0) {
            header.frame_id = "rgb";
            pub_rgb.publish(cv_bridge::CvImage(header, "bgr8", rgb).toImageMsg());
        }

        if (pub_depth.getNumSubscribers() != 0) {
            header.frame_id = "depth";
            pub_depth.publish(cv_bridge::CvImage(header, "depth", depth).toImageMsg());
        }
        if (pub_pc.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*pc.get(), pc_msg);
            pc_msg.header.frame_id = "camera_link";
            pc_msg.header.stamp = header.stamp;
            pub_pc.publish(pc_msg);
        }

    });

    ros::waitForShutdown();
    mj_deleteModel(m);
    mj_deleteData(d);
    mj_deactivate();

    return 0;
}
