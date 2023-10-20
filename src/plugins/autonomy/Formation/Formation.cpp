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
 * @author Natalie Davis <natalie.m.davis@gtri.gatech.edu>
 * @date 01 September 2023
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 * To do:
 * 
 * 1. Change how follower tracks the leader for the follower call back. It should be a param in the 
 * struct that tracks the entity id of the leader if this is allowed information to track. It seems
 * we only know the position, trajectory, and desired formation - so entity ID might not be allowed.
 * 
 * 2. Handle the z position for euclidean distance calculations
 * 
 * 3. Update the follower track to update the x, y, and z positions of the followers. Currently it
 * only adds them from the first time the message is sent
 * 
 * 4. Update color tracking of the spheres. Currently only changes to green if the follow track is not empty
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

#include <scrimmage/common/Shape.h>
#include <scrimmage/proto/Shape.pb.h>

#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

#include <scrimmage/plugins/autonomy/Formation/Formation.h>

#include <vector>
#include <algorithm>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

using std::cout;
using std::endl;

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
    
    // Set up entity data
    ent_id = parent_->id().id();
    
    // Leader information
    leader_speed_ = scrimmage::get("leader_speed", params, 0.0);
    leader_ = scrimmage::get<bool>("leader", params, false);

    // Follower information
    follower_speed_ = scrimmage::get("follower_speed", params, 0.0);
    follow_v_k_ = scrimmage::get("follow_v_k", params, 0.0);
    x_disp_ = scrimmage::get("x_disp", params, 0.0);
    y_disp_ = scrimmage::get("y_disp", params, 0.0);
    z_disp_ = scrimmage::get("z_disp", params, 0.0);
    lead_to_follow_dist_ = scrimmage::get("lead_to_follow_dist", params, 0.0);

    // Toggles spheres around the entity
    show_shapes_ = scrimmage::get<bool>("show_shapes", params, false);

    // Entity avoidance parameters
    avoid_non_team_ = scrimmage::get<bool>("avoid_non_team", params, true);
    sphere_of_influence_ = scrimmage::get("sphere_of_influence", params, 0.0);
    minimum_range_ = scrimmage::get("minimum_range", params, 0.0);
    fat_guard_ = scrimmage::get("fat_guard", params, 0.0);
    avoid_state = false;
    
    // Project initial goal in front of entity
    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
    Eigen::Vector3d unit_vector = rel_pos.normalized();
    unit_vector = state_->quat().rotate(unit_vector);
    goal_ = state_->pos() + unit_vector * rel_pos.norm();
    goal_(2) = state_->pos()(2);

    // Handles entities hitting the edge of the simulation map
    enable_boundary_control_ = get<bool>("enable_boundary_control", params, false);
    auto bd_cb = [&](auto &msg) {boundary_ = sci::Boundary::make_boundary(msg->data);};
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);

    // Handles noise for state and contacts
    auto state_cb = [&](auto &msg) {
        noisy_state_set_ = true;
        noisy_state_ = msg->data;
    };
    subscribe<StateWithCovariance>("LocalNetwork", "StateWithCovariance", state_cb);

    auto cnt_cb = [&](scrimmage::MessagePtr<ContactMap> &msg) { // Nat - this may be good for locating entities nearby
        noisy_contacts_ = msg->data; // Save map of noisy contacts
    };
    subscribe<ContactMap>("LocalNetwork", "ContactsWithCovariances", cnt_cb);

    // Setting up output vars - includes altitude, speed, and heading
    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

    // If initialized as a follower, update tracking
    // follower_map holds a struct for each follower in the formation
    if(!leader_){
        FollowerData follower;
        follower.x_pos = state_->pos()(0);
        follower.y_pos = state_->pos()(1);
        follower.z_pos = state_->pos()(2);
        follower.ent_id = ent_id;

        follower_map[ent_id] = follower;
    }

    // Set up follower subscriber. This listens to leader data that is used to update the
    // leader position tracking, which acts as goal positions for the follower
    auto follower_cb = [&](auto &msg) {

        // Only compute euclidean distance if the message is not from the current entity
        // and if it is from the leader - assumed to be entity #1 for now. Will need to
        // later identify ways a leader should be specified
        if(msg->data.bird_id() == ent_id || msg->data.bird_id() != 1){ // Another instance where leader is defined as ent id 1
            return;
        }

        leader_x_pos = msg->data.x_pos();
        leader_y_pos = msg->data.y_pos();
        leader_z_pos = msg->data.z_pos();

        // If the entity is a leader, but it is also receiver other leader messages, calculate
        // the euclidean distance. If it is within a range, need to combine formations and define
        // a leader.
        if(leader_){
            float x_pos_sq = (state_->pos()(0) - leader_x_pos) * (state_->pos()(0) - leader_x_pos);
            float y_pos_sq = (state_->pos()(1) - leader_y_pos) * (state_->pos()(1) - leader_y_pos);
            //float z_pos_sq = (state_->pos()(2) - leader_z_pos) * (state_->pos()(2) - leader_z_pos); // IF using z, will need a diff distance threshold
            float euc_dist = std::sqrt(x_pos_sq + y_pos_sq);

            if(euc_dist < lead_to_follow_dist_ && ent_id != 1){ // Another instance where leader is defined as ent id 1
                leader_ = false;
            }
        }
    };
    subscribe<scrimmage_msgs::FormationLeader>("GlobalNetwork", "FormationLeader", follower_cb);
    leader_pub_ = advertise("GlobalNetwork", "FormationLeader");

    // Set up follower tracker subscriber. This updates tracking
    // follower_map holds a struct for each follower in the formation
    auto tracker_cb = [&](auto &msg) { 
        if(follower_map.count(msg->data.bird_id()) == 0){
            // cout << "Adding new follower id: " << msg->data.bird_id() << endl;
            FollowerData follower;
            follower.x_pos = msg->data.x_pos();
            follower.y_pos = msg->data.y_pos();
            follower.z_pos = msg->data.z_pos();
            follower.ent_id = msg->data.bird_id();

            follower_map[msg->data.bird_id()] = follower;
        }
    };
    subscribe<scrimmage_msgs::FollowerTrack>("GlobalNetwork", "FollowerTrack", tracker_cb);
    follower_track_pub_ = advertise("GlobalNetwork", "FollowerTrack");
}

// Handles entity avoidance
void Formation::avoidance_vectors(ContactMap &contacts,
                                      std::vector<Eigen::Vector3d> &O_vecs) {
    // cout << "-------------Avoidance Calculation-------------" << endl;

    // Minimum distance used for state switching between avoidance and follower velocity calculations
    double min_dist = sphere_of_influence_ + fat_guard_;

    for (auto it = contacts.begin(); it != contacts.end(); it++) {
        // Ignore own position / id
        if (it->second.id().id() == parent_->id().id()) {
            continue;
        }

        if (!avoid_non_team_ &&
            it->second.id().team_id() != parent_->id().team_id()) {
            continue;
        }

        Eigen::Vector3d diff = it->second.state()->pos() - state_->pos();

        double O_mag = 0;
        double dist = diff.norm();

        // State machine distance track
        if(dist < min_dist){
            min_dist = dist;
        }

        if (dist > sphere_of_influence_) {
            O_mag = 0;
        } else if (minimum_range_ < dist && dist <= sphere_of_influence_) {
            O_mag = (sphere_of_influence_ - dist) /
                (sphere_of_influence_ - minimum_range_);
        } else if (dist <= minimum_range_) {
            O_mag = 1e10;
        }

        Eigen::Vector3d O_dir = -O_mag * diff.normalized();
        // cout << "Sphere val: " << sphere_of_influence_ << " Dist val: " << dist << " Min range: " << minimum_range_
        //     << " O_mag: " << O_mag << endl;

        // cout << "Entity id: " << ent_id 
        // << " and the contact vector id is: " << it->second.id().id() 
        // << " The avoidance vector: " << O_dir << endl;

        O_vecs.push_back(O_dir);
    }

    if(min_dist < (sphere_of_influence_ + fat_guard_)){
        avoid_state = true;
    } else{
        avoid_state = false;
    }

    // cout << "-----------------------------------------------" << endl;
}

bool Formation::step_autonomy(double t, double dt) {

    ////////////////////////// Collision Avoidance //////////////////////////

    // Compute repulsion vector from each robot contact
    std::vector<Eigen::Vector3d> O_vecs;

    avoidance_vectors(noisy_contacts_, O_vecs);

    // Normalize each repulsion vector and sum
    desired_vector_ <<  0, 0, 0;
    for (auto it = O_vecs.begin(); it != O_vecs.end(); it++) {
        if (it->hasNaN()) {
            continue; // ignore misbehaved vectors
        }
        desired_vector_ += *it;
    }

    ////////////////////////// Boundary Control //////////////////////////
   
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

    //////////////////////// Velocity & Goal Calc ////////////////////////

    if(!leader_){
        goal_(0) = leader_x_pos - x_disp_;
        goal_(1) = leader_y_pos - y_disp_;
        goal_(2) = leader_z_pos - z_disp_;

        // cout << "Updating follower goal positions: "
        //                                 << " x_pos: " << goal_(0)
        //                                 << " y_pos: " << goal_(1)
        //                                 << " z_pos: " << goal_(2) 
        //                                 << endl;
    }

    Eigen::Vector3d diff = goal_ - noisy_state_.pos();
    Eigen::Vector3d v;

    if(!leader_){
        // Needs to be more like a state machine, not just checking avoidance every time
        if(!avoid_state){ // avoid state determiner should be based on the min dist value from avoidance function
            v = follow_v_k_ * diff;
            // cout << "--------------------------DIFF VEL--------------------------" << endl;
            // cout << "Using diff for velocity: " << v(0) << ", " << v(1) << ", " << v(2) << endl;
            // cout << "Desired vector: " << desired_vector_(0) << " , " << desired_vector_(1) << " , " << desired_vector_(2) << endl;
            // cout << "Diff is: " << diff(0) << " ," << v(1) << " ," << v(2) << endl;
            // cout << "-------------------------------------------------------------" << endl;
        } else { // only come out of avoidance if you are within a certain min distance + fat guard
            v = desired_vector_ * follower_speed_;
            cout << "--------------------------AVOIDANCE VEL--------------------------" << endl;
            cout << "Using avoidance for velocity: " << v(0) << ", " << v(1) << ", " << v(2) << endl;
            cout << "Desired vector: " << desired_vector_(0) << " , " << desired_vector_(1) << " , " << desired_vector_(2) << endl;
            cout << "Diff is: " << diff(0) << " ," << v(1) << " ," << v(2) << endl;
            cout << "-------------------------------------------------------------" << endl;
        }

        if(v(0)>follower_speed_){
                v(0) = follower_speed_;
        } else if(v(0)<-follower_speed_){
            v(0) = -follower_speed_;
        } 

        if(v(1)>follower_speed_){
            v(1) = follower_speed_;
        } else if(v(1)<-follower_speed_){
            v(1) = -follower_speed_;
        }

        if(v(2)>follower_speed_){
            v(2) = follower_speed_;
        } else if(v(2)<-follower_speed_){
            v(2) = -follower_speed_;
        }
    } else {
        v = leader_speed_ * diff.normalized();
    }

    cout << "Output v for entity: " << ent_id << ": " << v(0) << ", " << v(1) << ", " << v(2) << endl;
    
    double heading = Angles::angle_2pi(atan2(v(1), v(0)));
    vars_.output(desired_alt_idx_, goal_(2));
    vars_.output(desired_speed_idx_, v.norm());
    vars_.output(desired_heading_idx_, heading);

    //////////////////////// Update leader & follower output messages ////////////////////////

    if(leader_){
        auto leader_msg = std::make_shared<Message<scrimmage_msgs::FormationLeader>>();
        leader_msg->data.set_x_pos(state_->pos()(0));
        leader_msg->data.set_y_pos(state_->pos()(1));
        leader_msg->data.set_z_pos(state_->pos()(2));
        leader_msg->data.set_bird_id(ent_id);
        leader_pub_->publish(leader_msg);
     } else{
        auto follower_track_msg = std::make_shared<Message<scrimmage_msgs::FollowerTrack>>();
        follower_track_msg->data.set_x_pos(state_->pos()(0));
        follower_track_msg->data.set_y_pos(state_->pos()(1));
        follower_track_msg->data.set_z_pos(state_->pos()(2));
        follower_track_msg->data.set_bird_id(ent_id);
        follower_track_pub_->publish(follower_track_msg);
    }

    //////////////////////// Handle spheres drawn around entity ////////////////////////

    if (show_shapes_) {
        // Draw the sphere of influence
        if (circle_shape_ == nullptr) {
            circle_shape_ = sc::shape::make_sphere(
                state_->pos(), 2,
                Eigen::Vector3d(255, 0, 0), 0.2);
        } if(!follower_map.empty()){
            sc::set(circle_shape_->mutable_color(), Eigen::Vector3d(0, 255, 0));
        }
        sc::set(circle_shape_->mutable_sphere()->mutable_center(), state_->pos());
        draw_shape(circle_shape_);
    }

    //////////////////////// Return from step function ////////////////////////
    noisy_state_set_ = false;
    return true;
}
} // namespace autonomy
} // namespace scrimmage
