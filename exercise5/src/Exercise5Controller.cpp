/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise5/Exercise5Controller.h>
#include <pluginlib/class_list_macros.h>


void Exercise5Controller::update(const ros::Time &time, const ros::Duration &period) {
    Eigen::Matrix<double, 7, 1> tau_real;
    std::array<double, 7> q{};
    for (size_t i = 0; i < q.size(); ++i) {
        q[i] = joints[i].getPosition();
        tau_real(i, 0) = joints[i].getEffort();
    }

    nh.getParam("/"+robot_name+"_force_vector", force_vector);
    Eigen::Matrix<double, 6, 7> J = computeJacobian(q);
    const Eigen::Matrix<double, 6, 7> JtpinvL = (J * J.transpose()).inverse() * J;
    const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau_real;

    Eigen::Matrix<double, 6, 1> Fd;
    Fd << force_vector[0], force_vector[1], force_vector[2], 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 7, 1> tau;
    tau.setZero();

    // TODO: Copy the code from exercise5 here
    const Eigen::Matrix<double, 6, 1> Fe = Fd - wrench;
    fe_int += Fe;

    Eigen::Matrix<double, 6, 6> Kp, Ki;
    Kp.setIdentity();
    Kp *= 0.0;

    Ki.setIdentity();
    Ki *= 0.0;

    tau = J.transpose() * (Fd + Kp * Fe + Ki * fe_int);

    for (size_t i = 0; i < joints.size(); ++i) {
        joints[i].setCommand(tau(i, 0));
    }
}

bool Exercise5Controller::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    const auto names = hw->getNames();
    std::string robot_name;
    nh = handle;
    handle.getParam("/robot_name", robot_name);
    nh.getParam("/"+robot_name+"_force_vector", force_vector);
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto jname = std::string(robot_name+"_joint") + std::to_string(i + 1);
        if (std::find(names.begin(), names.end(), jname) == names.end()) {
            ROS_ERROR_STREAM("Joint not found: " << jname);
            ROS_ERROR_STREAM("Available joints: ");
            for (const auto &name : names) {
                ROS_ERROR_STREAM(name);
            }
            return false;
        }
        joints[i] = hw->getHandle(jname);
    }

    return Controller::init(hw, handle);
}

void Exercise5Controller::starting(const ros::Time &time1) {
    fe_int.setZero(); //reset integration term
    ControllerBase::starting(time1);
}

void Exercise5Controller::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}

Eigen::Isometry3d Exercise5Controller::dkt(const std::array<double, 7> &q, const Eigen::Vector3d &r) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const auto DHparams = getDHParameters();
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        T = T * DH(DHparams[i][0] + q[i], DHparams[i][1], DHparams[i][2], DHparams[i][3]);
    }
    return T * Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Translation3d(r);
}

Eigen::Isometry3d Exercise5Controller::DH(double theta, double d, double a, double alpha) {
    return Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(a, 0, d) *
           Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
}

std::array<std::array<double, 4>, 7> Exercise5Controller::getDHParameters() const {
    const std::array<std::array<double, 4>, NUM_OF_JOINTS> dh = {{
                                                                         {0, 0.333, 0, -M_PI_2},
                                                                         {0, 0, 0, M_PI_2},
                                                                         {0, 0.316, 0.0825, M_PI_2},
                                                                         {0, 0, -0.0825, -M_PI_2},
                                                                         {0, 0.384, 0.0, M_PI_2},
                                                                         {0, 0, 0.088, M_PI_2},
                                                                         {0, 0.107, 0.0, 0.0}
                                                                 }};
    return dh;
}

Eigen::Matrix<double, 6, 7>
Exercise5Controller::computeJacobian(const std::array<double, 7> &q, const Eigen::Vector3d &ref) const {
    Eigen::Matrix<double, 6, 7> jac;
    Eigen::Isometry3d Tee = dkt(q, ref);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    const auto DHparams = getDHParameters();
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        const Eigen::Vector3d n = T.linear() * Eigen::Vector3d::UnitZ(); //n_i
        const Eigen::Vector3d r = (Tee.translation() - T.translation());
        const Eigen::Vector3d dr = n.cross(r);
        jac.block(0, i, 3, 1) = dr;
        jac.block(3, i, 3, 1) = n;
        T = T * DH(DHparams[i][0] + q[i], DHparams[i][1], DHparams[i][2], DHparams[i][3]);
    }

    return jac;
}

PLUGINLIB_EXPORT_CLASS(Exercise5Controller, controller_interface::ControllerBase)
