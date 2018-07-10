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
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>

#include <scrimmage/plugins/autonomy/MOOSAutonomy/MOOSAutonomy.h>
#include <scrimmage/plugins/autonomy/MOOSAutonomy/MOOSNode.h>

namespace sc = scrimmage;
using ang = scrimmage::Angles;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::MOOSAutonomy, MOOSAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

MOOSAutonomy::MOOSAutonomy() {
    angles_from_moos_.set_input_clock_direction(ang::Rotate::CW);
    angles_from_moos_.set_input_zero_axis(ang::HeadingZero::Pos_Y);
    angles_from_moos_.set_output_clock_direction(ang::Rotate::CCW);
    angles_from_moos_.set_output_zero_axis(ang::HeadingZero::Pos_X);

    angles_to_moos_.set_input_clock_direction(ang::Rotate::CCW);
    angles_to_moos_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    angles_to_moos_.set_output_clock_direction(ang::Rotate::CW);
    angles_to_moos_.set_output_zero_axis(ang::HeadingZero::Pos_Y);
}

void MOOSAutonomy::init(std::map<std::string, std::string> &params) {
    moos_app_name_ = sc::get<std::string>("moos_app_name", params, "scrimmage");
    moos_app_name_ += std::string("_") + std::to_string(parent_->id().id());

    moos_script_ = sc::expand_user(sc::get<std::string>("moos_script", params, "launch.sh"));
    moos_mission_file_ = sc::expand_user(sc::get<std::string>("moos_mission_file", params, "alpha.moos"));

    desired_state_->vel() = Eigen::Vector3d::UnitX()*21;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

    // Kick off moos node thread
    moos_node_.set_time_warp(parent_->mp()->time_warp());
    moos_node_thread_ = std::thread(&MOOSAutonomy::run_moos_node, this);
}

bool MOOSAutonomy::ready() {
    return moos_node_.ready();
}

bool MOOSAutonomy::step_autonomy(double t, double dt) {
    // Provide contact locations:
    for (auto &kv : *contacts_) {

        MOOSNode::NodeReportType type;
        if (parent_->id().id() == kv.first) {
            type = MOOSNode::OWNSHIP;
        } else {
            type = MOOSNode::TRUTH_CONTACT;
        }

        // Convert heading to local cartesian
        sc::StatePtr &s = kv.second.state();
        angles_to_moos_.set_angle(ang::rad2deg(s->quat().yaw()));
        double heading = angles_to_moos_.angle();

        moos_node_.PublishNodeReport(type,
                                     std::to_string(kv.first), "",
                                     s->pos()(0), s->pos()(1),
                                     s->vel().norm(), heading,
                                     -s->pos()(2), "kayak", "none",
                                     t, "0");
    }

    // Get desired state from moos
    sc::State s = moos_node_.desired_state();

    // Convert heading to local cartesian
    angles_from_moos_.set_angle(ang::rad2deg(s.quat().yaw()));
    s.quat().set(s.quat().roll(), s.quat().pitch(),
                 ang::deg2rad(angles_from_moos_.angle()));

    desired_state_->vel() = s.vel();
    desired_state_->quat() = s.quat();
    desired_state_->pos() = s.pos();

    // Set the VariableIO output for controller
    vars_.output(desired_alt_idx_, desired_state_->pos()(2));
    vars_.output(desired_speed_idx_, desired_state_->vel()(0));
    vars_.output(desired_heading_idx_, desired_state_->quat().yaw());

    return true;
}

void MOOSAutonomy::run_moos_node() {
    moos_node_.Run(moos_app_name_.c_str(), moos_mission_file_.c_str());
}
} // namespace autonomy
} // namespace scrimmage
