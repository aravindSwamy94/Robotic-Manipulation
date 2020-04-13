/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise6/Exercise6Controller.h>
#include <pluginlib/class_list_macros.h>

void Exercise6Controller::update(const ros::Time &time, const ros::Duration &period) {
    Eigen::Matrix<double, 7, 1> tau_real;
    std::array<double, 7> q{};
    for (size_t i = 0; i < q.size(); ++i) {
        q[i] = joints[i].getPosition();
        tau_real(i, 0) = joints[i].getEffort();
    }

    const Eigen::Matrix<double, 6, 7> J = computeJacobian(q);
    const Eigen::Matrix<double, 6, 7> JtpinvL = (J * J.transpose()).inverse() * J;
    const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau_real;

    Eigen::Matrix<double, 6, 1> Fd;
    Fd << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0; //desired force
    const Eigen::Matrix<double, 6, 1> Xe = getPoseError(dkt(q), time); //this function calls getPositionError internally
    Eigen::Matrix<double, 7, 1> tau = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
    P(0,0) =0;
    P(1,1) =1;
    P(2,2) =1;
    P(3,3) =1;
    P(4,4) =1;
    P(5,5) =1;
    double kp=50;
    tau = J.transpose() * ((kp*P*Xe) + ((I-P)*Fd));
    //todo: compute tau - implement the controller
    //todo: update function getPositionError such that robot will follow the trajectory

    for (size_t i = 0; i < joints.size(); ++i) {
        joints[i].setCommand(tau(i, 0));
    }
}

Eigen::Matrix<double, 3, 1> Exercise6Controller::getPositionError(const Eigen::Isometry3d &current_pose,
                                                                  const Eigen::Isometry3d &starting_pose,
                                                                  double time_from_start) const {

    Eigen::Matrix<double, 3, 1> err;
    Eigen::Translation3d desired_val;

    if (robot_name == "lumi1") {
        if(time_from_start < 10.0) 
            desired_val.z() = 0.0;
        else if((time_from_start > 10.0) && (time_from_start < 20.0))
            desired_val.z() = ((time_from_start - 10.0) * 0.02);
        else if((time_from_start > 20.0) && (time_from_start < 30.0))
            desired_val.z() = 0.2;
        else if((time_from_start > 30.0) && (time_from_start < 40.0))
            desired_val.z() = (( 40.0 - time_from_start) * 0.02);       
        else if( (time_from_start > 40) )
            desired_val.z() = 0.0;
        if(time_from_start < 20.0)
            desired_val.y() = 0;
        else if((time_from_start > 20.0) && (time_from_start < 30.0))
            desired_val.y() = (time_from_start - 20.0) * 0.02;
        else if(time_from_start > 30.0)
            desired_val.y() = 0.2;

    }
    if (robot_name == "lumi2"){
        if(time_from_start < 10.0)
            desired_val.z() = 0.0;
        else if((time_from_start > 10.0) && (time_from_start < 20.0))
            desired_val.z() = ((time_from_start - 10.0) * 0.02);
        else if((time_from_start > 20.0) && (time_from_start < 30.0))
            desired_val.z() = 0.2;
        else if((time_from_start > 30.0) && (time_from_start < 40.0))
            desired_val.z() = (( 40.0 - time_from_start) * 0.02); 
        else if( (time_from_start > 40) )
            desired_val.z() = 0.0;
        if(time_from_start < 20.0)
            desired_val.y() = 0;
		else if((time_from_start > 20.0) && (time_from_start < 30.0))
            desired_val.y() = (time_from_start - 20.0) * (-0.02);
		else if(time_from_start > 30.0)
            desired_val.y() = -0.2;
    }
    // todo: Compute position error s.t. the gripper will follow the trajectory described in assignment
    // The error must be specified in a robot base frame
    
    // If you have desired pose, you can compute error like this:
   //  Eigen::Isometry3d desired_pose ;
    const Eigen::Isometry3d desired_pose =  desired_val * starting_pose ;

    err = desired_pose.linear() * (desired_pose.inverse() * current_pose).translation();

    return err;
}

Eigen::Matrix<double, 6, 1> Exercise6Controller::getPoseError(const Eigen::Isometry3d &current_position,
                                                              const ros::Time &ros_time) {
    if (!initial_position) {
        initial_position = std::make_shared<Eigen::Isometry3d>(current_position);
    }
    const auto time = (ros_time - start_time).toSec();

    const Eigen::Isometry3d t0 = *initial_position;
    const Eigen::Isometry3d t1 = current_position;
    const Eigen::Isometry3d dt = t0.inverse() * t1;
    const Eigen::Vector3d dp_ee = dt.translation();
    const Eigen::Vector3d dphi_ee = Eigen::Vector3d(dt(2, 1), -dt(2, 0), dt(1, 0));
    const Eigen::Vector3d dp_base = t0.linear() * dp_ee;
    const Eigen::Vector3d dphi_base = t0.linear() * dphi_ee;

    Eigen::Matrix<double, 6, 1> err;
    err.head(3) = getPositionError(current_position, *initial_position, time);
    err.tail(3) = dphi_base;
    return -err;
}

bool Exercise6Controller::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    handle.getParam("/robot_name", robot_name);
    const auto names = hw->getNames();
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto jname = std::string(robot_name + "_joint") + std::to_string(i + 1);
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

void Exercise6Controller::starting(const ros::Time &time1) {
    fe_int.setZero(); //reset integration term
    start_time = time1;
    ControllerBase::starting(time1);
}

void Exercise6Controller::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}

Eigen::Isometry3d Exercise6Controller::dkt(const std::array<double, 7> &q, const Eigen::Vector3d &r) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const auto DHparams = getDHParameters();
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        T = T * DH(DHparams[i][0] + q[i], DHparams[i][1], DHparams[i][2], DHparams[i][3]);
    }
    return T * Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Translation3d(r);
}

Eigen::Isometry3d Exercise6Controller::DH(double theta, double d, double a, double alpha) {
    return Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(a, 0, d) *
           Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
}

std::array<std::array<double, 4>, 7> Exercise6Controller::getDHParameters() const {
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
Exercise6Controller::computeJacobian(const std::array<double, 7> &q, const Eigen::Vector3d &ref) const {
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


PLUGINLIB_EXPORT_CLASS(Exercise6Controller, controller_interface::ControllerBase)
