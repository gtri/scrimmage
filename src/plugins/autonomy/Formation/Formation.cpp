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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/sensor/Sensor.h>

#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

using std::cout;
using std::endl;

#include <scrimmage/plugins/autonomy/Formation/Formation.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
namespace fs = boost::filesystem;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::Formation,
                Formation_plugin)

namespace scrimmage {
namespace autonomy {

void Formation::init(std::map<std::string, std::string> &params) {
    speed_ = scrimmage::get("speed", params, 0.0);
    leader_ = scrimmage::get<bool>("leader", params, false);

    // Project goal in front...
    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
    Eigen::Vector3d unit_vector = rel_pos.normalized();
    unit_vector = state_->quat().rotate(unit_vector);
    goal_ = state_->pos() + unit_vector * rel_pos.norm();
    goal_(2) = state_->pos()(2);

    // Nat - what if goal isn't in front? see if it turns
    // Should probably have a tag that is for leader/follower with the given entity
    // Instead of speed, probably want displacement compared to the leader instead.
    // Might be able to stack autonomies for go to waypoint and avoid entity MS
    //
    // Will need logic if the entitiy is not the leader. If it is the leader, it can
    // just continue to fly straight. Maybe a good way to do this if it is not the leader,
    // it should publish messages from follower, saying it is in the formation. The leader
    // should publish messages with its location, that way the follower can do proper displacement

    // Set the desired_z to our initial position.
    // desired_z_ = state_->pos()(2);

    // Register the desired_z parameter with the parameter server
    auto param_cb = [&](const double &desired_z) {
        std::cout << "desired_z param changed at: " << time_->t()
        << ", with value: " << desired_z << endl;
    };
    register_param<double>("desired_z", goal_(2), param_cb);

    frame_number_ = 0;

    enable_boundary_control_ = get<bool>("enable_boundary_control", params, false);

    auto bd_cb = [&](auto &msg) {boundary_ = sci::Boundary::make_boundary(msg->data);};
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);

    auto state_cb = [&](auto &msg) {
        noisy_state_set_ = true;
        noisy_state_ = msg->data;
    };
    subscribe<StateWithCovariance>("LocalNetwork", "StateWithCovariance", state_cb);

    auto cnt_cb = [&](scrimmage::MessagePtr<ContactMap> &msg) { // Nat - this may be good for locating entities nearby
        noisy_contacts_ = msg->data; // Save map of noisy contacts
    };
    subscribe<ContactMap>("LocalNetwork", "ContactsWithCovariances", cnt_cb);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

    // Set up leader publisher
    leader_pub_ = advertise("GlobalNetwork", "FormationLeader");

    // Set up follower subscriber
    auto follower_cb = [&](auto &msg) { // Nat - this may be good for locating entities nearby
        auto leader_x_pos = msg->data.x_pos;
        auto leader_y_pos = msg->data.y_pos;
        auto leader_z_pos = msg->data.z_pos;
    };
    subscribe<scrimmage_msgs::FormationLeader>("GlobalNetwork", "FormationLeader", follower_cb);
}

bool Formation::step_autonomy(double t, double dt) {
    
    // Nat - Need a case where the entity senses a new nearby autonomy
    // If it does, then need to do new leader selection
    if(!leader_){
        goal_(0) = leader_x_pos;
        goal_(1) = leader_y_pos;
        goal_(2) = leader_z_pos;

        cout << "Follower goal positions: x_pos: " << goal_(0)
                                        << " y_pos: " << goal_(1)
                                        << " z_pos: " << goal_(2) 
                                        << endl;
    }

    // Read data from sensors...
    if (!noisy_state_set_) {
        noisy_state_ = *state_;
    }

    if (boundary_ != nullptr && enable_boundary_control_) {
        if (!boundary_->contains(noisy_state_.pos())) {
            // Project goal through center of boundary
            Eigen::Vector3d center = boundary_->center();
            center(2) = noisy_state_.pos()(2); // maintain altitude
            Eigen::Vector3d diff = center - noisy_state_.pos();
            goal_ = noisy_state_.pos() + diff.normalized() * 1e6;
        }
    }

    // If the entity is the leader, the goal is in front
    Eigen::Vector3d diff = goal_ - noisy_state_.pos();
    Eigen::Vector3d v = speed_ * diff.normalized();

    ///////////////////////////////////////////////////////////////////////////
    // Convert desired velocity to desired speed, heading, and pitch controls
    ///////////////////////////////////////////////////////////////////////////
    // Nat - this should be if you are the leader, then do normal straight flying
    if(leader_){
        // If the entity is the leader, the goal is in front
        Eigen::Vector3d diff = goal_ - noisy_state_.pos();
        Eigen::Vector3d v = speed_ * diff.normalized();

        double heading = Angles::angle_2pi(atan2(v(1), v(0)));
        vars_.output(desired_alt_idx_, goal_(2));
        vars_.output(desired_speed_idx_, v.norm());
        vars_.output(desired_heading_idx_, heading);

        auto leader_msg = std::make_shared<Message<scrimmage_msgs::FormationLeader>>();
        leader_msg->data.set_x_pos(state_->pos()(0));
        leader_msg->data.set_y_pos(state_->pos()(1));
        leader_msg->data.set_z_pos(state_->pos()(2));
        leader_pub_->publish(leader_msg);


    } else { // If it is not the leader, take the leader's heading and add a displacement

    }

    // Nat - if you are not the leader, get the leader's position and do proper displacements
    // for desired alt, speed, and heading

    noisy_state_set_ = false;
    return true;
}
} // namespace autonomy
} // namespace scrimmage
