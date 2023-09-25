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
    ent_id = parent_->id().id();
    
    leader_speed_ = scrimmage::get("leader_speed", params, 0.0);
    leader_ = scrimmage::get<bool>("leader", params, false);

    follower_speed_ = scrimmage::get("follower_speed", params, 0.0);

    // Project goal in front...
    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
    Eigen::Vector3d unit_vector = rel_pos.normalized();
    unit_vector = state_->quat().rotate(unit_vector);
    goal_ = state_->pos() + unit_vector * rel_pos.norm();
    goal_(2) = state_->pos()(2);

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

    // Set up follower subscriber
    auto follower_cb = [&](auto &msg) { // Nat - this may be good for locating entities nearby
        if(msg->data.bird_id() == ent_id || msg->data.bird_id() != 1){ // Only compute euc dist. for cur vs ent 1
            cout << "Skip calculation, entity ids are the same" << endl;
            return;
        }

        // If the bird is a follower, offsets could be put here depending on the follower number
        leader_x_pos = msg->data.x_pos();
        leader_y_pos = msg->data.y_pos();
        leader_z_pos = msg->data.z_pos();
        cout << "Leader positions: x_pos: " << leader_x_pos
                                        << " y_pos: " << leader_y_pos
                                        << " z_pos: " << leader_z_pos 
                                        << endl;

        cout << "Current entity " << ent_id << "positions: x_pos: " << state_->pos()(0)
                                        << " y_pos: " << state_->pos()(1)
                                        << " z_pos: " << state_->pos()(2) 
                                        << endl;


        // If the entity is a leader, but it is also receiver other leader messages, calculate
        // the euclidean distance. If it is within a range, need to combine formations and define
        // a new leader. For the time being, make id 1 the leader.
        if(leader_){
            float x_pos_sq = (state_->pos()(0) - leader_x_pos) * (state_->pos()(0) - leader_x_pos);
            float y_pos_sq = (state_->pos()(1) - leader_y_pos) * (state_->pos()(1) - leader_y_pos);
            //float z_pos_sq = (state_->pos()(2) - leader_z_pos) * (state_->pos()(2) - leader_z_pos); // IF using z, will need a diff distance threshold
            float euc_dist = std::sqrt(x_pos_sq + y_pos_sq);

            cout << "Euclidean distance is: " << euc_dist << " and ent id is: " << ent_id << endl;

            if(euc_dist < 50 && ent_id != 1){
                leader_ = false;
                follower_track[ent_id] = follower_track.size();
                cout << "Entity " << ent_id << " is no longer the leader. It is now follower #" << follower_track[ent_id] << endl;
            }
        }
    };
    subscribe<scrimmage_msgs::FormationLeader>("GlobalNetwork", "FormationLeader", follower_cb);
    leader_pub_ = advertise("GlobalNetwork", "FormationLeader");

    // Set up follower tracker subscriber
    // auto tracker_cb = [&](auto &msg) { 
    //     follower_track = msg->data.follower_track();
    //     cout << "In the follower track call back. The map is currently the following size: " << follower_track.size() << endl;
    // };
    // subscribe<scrimmage_msgs::FollowerTrack>("GlobalNetwork", "FollowerTrack", tracker_cb);
    // follower_track_pub_ = advertise("GlobalNetwork", "FollowerTrack");
}

bool Formation::step_autonomy(double t, double dt) {

    // Goal displacements if a follower
    if(!leader_){
        // When adding this kind of offset with the way velocity is calculated,
        // if the drone enters a zone that is closer than the displacement,
        // it turns around. Instead, need to have it slow and if it passes into the
        // displacement zone, decrease speed but never spin in a circle.
        goal_(0) = leader_x_pos - 5;
        goal_(1) = leader_y_pos;
        goal_(2) = leader_z_pos;

        cout << "Updating follower goal positions for entity" << ent_id
                                        << " x_pos: " << goal_(0)
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

    // If the entity is the leader, the goal is in front // look at unicycle pid 
    Eigen::Vector3d diff = goal_ - noisy_state_.pos();
    Eigen::Vector3d v;
    if(!leader_){
        // Calculate velocity using a PID controller
        v = follower_speed_ * diff.normalized();
    } else {
        v = leader_speed_ * diff.normalized();
    }

    // Straight's way of calculating speed
    //Eigen::Vector3d v = speed_ * diff.normalized();

    if(ent_id != 1){
        cout << "The diff between goal and noisy state is: " << diff
                        << " The velocity vector is: " << v << endl;
    }

    double heading = Angles::angle_2pi(atan2(v(1), v(0)));
    vars_.output(desired_alt_idx_, goal_(2));
    vars_.output(desired_speed_idx_, v.norm());
    vars_.output(desired_heading_idx_, heading);

    if(leader_){
        auto leader_msg = std::make_shared<Message<scrimmage_msgs::FormationLeader>>();
        leader_msg->data.set_x_pos(state_->pos()(0));
        leader_msg->data.set_y_pos(state_->pos()(1));
        leader_msg->data.set_z_pos(state_->pos()(2));
        leader_msg->data.set_bird_id(ent_id);
        leader_pub_->publish(leader_msg);
     } //else{
    //     auto follower_track_msg = std::make_shared<Message<scrimmage_msgs::FollowerTrack>>();
    //     follower_track_msg->data.mutable_follower_track()[ent_id] = follower_track.size();
    //     follower_track_pub_->publish(follower_track_msg);
    // }

    noisy_state_set_ = false;
    return true;
}
} // namespace autonomy
} // namespace scrimmage
