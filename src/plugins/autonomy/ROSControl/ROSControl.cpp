/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */


#include <rosgraph_msgs/Clock.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#include <scrimmage/plugins/autonomy/ROSControl/ROSControl.h>
#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>

#include <boost/numeric/odeint.hpp>

namespace pl = std::placeholders;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ROSControl, ROSControl_plugin)

namespace scrimmage {
namespace autonomy {

ROSControl::ROSControl() {}

void ROSControl::init(std::map<std::string, std::string> &params) {
    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ += std::to_string(parent_->id().id());

    cmd_vel_sub_ = nh_->subscribe(ros_namespace_ + "/cmd_vel", 1, &ROSControl::cmd_vel_cb, this);

    zero_ctrls();

    x_.resize(3);

    desired_state_->vel() = Eigen::Vector3d::UnitX() * 0;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    //  ofs_.open("/home/syllogismrxs/tmp/data.csv",
    //            std::ofstream::out | std::ofstream::trunc);
    //  ofs_ << "x0, x1, x2" << endl;
}

void ROSControl::zero_ctrls() {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
}

void ROSControl::ctrl_filter(const std::vector<double> &x ,
                             std::vector<double> &dxdt , double t) {
    dxdt[0] = x[1];
    dxdt[1] = -x[2];
    dxdt[2] = 0;
}

bool ROSControl::step_autonomy(double t, double dt) {
    ros::spinOnce(); // check for new ROS messages

    // auto sys = std::bind(&ROSControl::ctrl_filter, this, pl::_1, pl::_2, pl::_3);
    // boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper;
    //
    // x_[2] = cmd_vel_.angular.z;
    // stepper.do_step(sys, x_, 0, dt);
    // //  ofs_ << x_[0] << ", " << x_[1] << ", " << x_[2] << endl;
    //
    // double scale = 0.95;
    // desired_state_->pos()(0) = -scale*cmd_vel_.angular.z; // aileron
    // desired_state_->pos()(1) = scale*cmd_vel_.linear.x;  // elevator
    // desired_state_->pos()(2) = scale*cmd_vel_.linear.z;  // rudder
    // desired_state_->vel()(0) = 1.0;       // constant full thrust
    // Need to saturate state variables before model runs

    desired_state_->vel()(0) = 30;
    desired_state_->vel()(1) = 10*cmd_vel_.angular.z;
    desired_state_->vel()(2) = 10*cmd_vel_.linear.x;

    zero_ctrls();

    return true;
}

void ROSControl::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_ = *msg;
}
} // namespace autonomy
} // namespace scrimmage
