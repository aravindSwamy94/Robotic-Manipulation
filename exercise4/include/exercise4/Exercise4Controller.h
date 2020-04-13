/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#ifndef EXERCISE4_EXERCISE5CONTROLLER_H
#define EXERCISE4_EXERCISE5CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


# define USE_FF_PI 1
using namespace std;

class Exercise4Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
    bool init(hardware_interface::EffortJointInterface *t, ros::NodeHandle &handle) override;

    void starting(const ros::Time &time1) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void stopping(const ros::Time &time1) override;

    /** @brief Get array of dh parameters for the panda robot. Order: theta, d, a, alpha */
    std::array<std::array<double, 4>, 7> getDHParameters() const;

    /** @brief Compute Jacobian analytically for the given robot configuration q and a reference frame r specified w.r.t. ee */
    Eigen::Matrix<double, 6, 7> computeJacobian(const std::array<double, 7> &q, const Eigen::Vector3d &r = Eigen::Vector3d::Zero()) const;

    /** @brief Compute direct kinematics for the given joint angles q. Assume ee is offseted by vector r. */
    Eigen::Isometry3d dkt(const std::array<double, 7> &q, const Eigen::Vector3d &r = Eigen::Vector3d::Zero()) const;

    /**
    * @brief Return affine transformation from DH parameters
    * @param theta rotation around z axis
    * @param d translation in z axis
    * @param a translation in x axis
    * @param alpha rotation around x axis
    * @return Rz * Tz * Tx * Rx  */
    static Eigen::Isometry3d DH(double theta, double d, double a, double alpha);

private:
    constexpr static size_t NUM_OF_JOINTS = 7;

    int integration_steps =30; // Integrating over the past 30 errors

    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;

    Eigen::Matrix<double, 6, 1> fe_int;

    vector<Eigen::Matrix<double, 6, 1>> error_buff;

    void addBufferElement(Eigen::Matrix<double, 6, 1> i);

    Eigen::Matrix<double, 6, 1> sum(vector<Eigen::Matrix<double, 6, 1>> t);
    

};

#endif //EXERCISE4_EXERCISE5CONTROLLER_H
