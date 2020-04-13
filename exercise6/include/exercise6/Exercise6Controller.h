/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#ifndef EXERCISE5_EXERCISE5CONTROLLER_H
#define EXERCISE5_EXERCISE5CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class Exercise6Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
    bool init(hardware_interface::EffortJointInterface *t, ros::NodeHandle &handle) override;

    void starting(const ros::Time &time1) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void stopping(const ros::Time &time1) override;

    /** @brief Get array of dh parameters for the panda robot. Order: theta, d, a, alpha */
    std::array<std::array<double, 4>, 7> getDHParameters() const;

    /** @brief Compute Jacobian analytically for the given robot configuration q and a reference frame r specified w.r.t. ee */
    Eigen::Matrix<double, 6, 7>
    computeJacobian(const std::array<double, 7> &q, const Eigen::Vector3d &r = Eigen::Vector3d::Zero()) const;

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

    /** @brief Get the position error for the current state s.t. the predefined fixed trajectory is followed. */
    Eigen::Matrix<double, 6, 1> getPoseError(const Eigen::Isometry3d &current_position,
                                                 const ros::Time &time);

    /** @brief Get the position error for the current state s.t. the predefined fixed trajectory is followed. */
    Eigen::Matrix<double, 3, 1> getPositionError(const Eigen::Isometry3d &current_pose,
                                                 const Eigen::Isometry3d &starting_pose,
                                                 double time_from_start) const;

private:
    constexpr static size_t NUM_OF_JOINTS = 7;

    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;

    Eigen::Matrix<double, 6, 1> fe_int;

    /** @brief Name of the robot, either lumi1 or lumi2 */
    std::string robot_name;

    /** @brief Start time of the controller */
    ros::Time start_time;

    /** @brief Initial pose of the robot. Initialised after first call of getPositionError */
    std::shared_ptr<Eigen::Isometry3d> initial_position;

};

#endif //EXERCISE5_EXERCISE5CONTROLLER_H
