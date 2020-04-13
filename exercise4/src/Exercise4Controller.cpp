/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise4/Exercise4Controller.h>
#include <pluginlib/class_list_macros.h>

void Exercise4Controller::addBufferElement(Eigen::Matrix<double, 6, 1> i)
{
    if(error_buff.size() < integration_steps)
    error_buff.push_back(i);
    else
    {
        vector<Eigen::Matrix<double, 6, 1>>:: iterator it=error_buff.begin();
        error_buff.erase(it);
        error_buff.push_back(i);
    }
}

Eigen::Matrix<double, 6, 1> Exercise4Controller::sum(vector<Eigen::Matrix<double, 6, 1>> t)
{
    Eigen::Matrix<double, 6, 1> result;
    for(int i=0;i<t.size();i++)
    {
        result += t[i];
    }
    return result;
}

void Exercise4Controller::update(const ros::Time &time, const ros::Duration &period) {
    Eigen::Matrix<double, 7, 1> tau_real;
    std::array<double, 7> q{};
    for (size_t i = 0; i < q.size(); ++i) {
        q[i] = joints[i].getPosition();
        tau_real(i, 0) = joints[i].getEffort();
    }

    Eigen::Matrix<double, 6, 7> J = computeJacobian(q); //jacobian for the current joint configuration q
    const Eigen::Matrix<double, 6, 7> JtpinvL = (J * J.transpose()).inverse() * J; //pseudoinverse
    const Eigen::Matrix<double, 6, 1> wrench = JtpinvL * tau_real; //measured force/torque for feedback

    Eigen::Matrix<double, 6, 1> Fd,Fe; //desired force/torque
    Fd << 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 7, 1> tau;
    tau.setZero();

    double Kp=0.8,Ki=0.6;
    Fe = Fd - wrench;

    addBufferElement(Fe);

    //cout<<"Time is "<<time<<endl;

    if(error_buff.size()>= 2)
    {        
        fe_int = (sum(error_buff)) * period.toSec();//(time.toSec()-previous_time.toSec());
    }
    else
    {
        fe_int.setZero();
    }

    // todo:  Compute effort (tau) such that cartesian force is constant and equals to the desired force/torque
    // you can use global variable fe_int (6x1 matrix) to integrate the error for PI controller

#if !USE_FF_PI
    tau = J.transpose() * Fd;
#else
    tau = J.transpose() * (Fd+ (Kp*Fe) + (Ki*fe_int) );
#endif
    // following lines will send the computed effort to the robot
    for (size_t i = 0; i < joints.size(); ++i) {
        joints[i].setCommand(tau(i, 0));
    }
}

bool Exercise4Controller::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    const auto names = hw->getNames();

    for (size_t i = 0; i < joints.size(); ++i) {
        const auto jname = std::string("lumi_joint") + std::to_string(i + 1);
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

void Exercise4Controller::starting(const ros::Time &time1) {
    fe_int.setZero(); //reset integration term
    ControllerBase::starting(time1);
}

void Exercise4Controller::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}

Eigen::Isometry3d Exercise4Controller::dkt(const std::array<double, 7> &q, const Eigen::Vector3d &r) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const auto DHparams = getDHParameters();
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        T = T * DH(DHparams[i][0] + q[i], DHparams[i][1], DHparams[i][2], DHparams[i][3]);
    }
    return T * Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Translation3d(r);
}

Eigen::Isometry3d Exercise4Controller::DH(double theta, double d, double a, double alpha) {
    return Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(a, 0, d) *
           Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
}

std::array<std::array<double, 4>, 7> Exercise4Controller::getDHParameters() const {
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
Exercise4Controller::computeJacobian(const std::array<double, 7> &q, const Eigen::Vector3d &ref) const {
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

PLUGINLIB_EXPORT_CLASS(Exercise4Controller, controller_interface::ControllerBase)
