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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/controller/RigidBody6DOFControllerROS/RigidBody6DOFControllerROS.h>
#include <boost/algorithm/string.hpp>

REGISTER_PLUGIN(scrimmage::Controller, scrimmage::controller::RigidBody6DOFControllerROS, RigidBody6DOFControllerROS_plugin)

namespace scrimmage {
namespace controller {

namespace sc = scrimmage;

void RigidBody6DOFControllerROS::init(std::map<std::string, std::string> &params) {
    u_ = std::make_shared<Eigen::Vector4d>();

    if (!ros::isInitialized()) {
        int argc = 0;
        std::string name = "simple_aircraft_3d_controller";
        ros::init(argc, NULL, name);
    }
    nh_ = std::make_shared<ros::NodeHandle>();
    cmd_vel_sub_ = nh_->subscribe("cmd_vel", 1, &RigidBody6DOFControllerROS::cmd_vel_cb, this);
}

bool RigidBody6DOFControllerROS::step(double t, double dt) {
    ros::spinOnce();

    // Convert twist to aircraft control surfaces
    double thrust = 0, elevator = 0, aileron = 0, rudder = 0;

    thrust = cmd_vel_.linear.x;
    elevator = cmd_vel_.angular.y;
    aileron = cmd_vel_.angular.x;
    rudder = cmd_vel_.angular.z;

    (*u_) << thrust, elevator, aileron, rudder;

    return true;
}

void RigidBody6DOFControllerROS::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_ = *msg;
}
} // namespace controller
} // namespace scrimmage
