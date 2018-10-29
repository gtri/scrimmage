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

#include <scrimmage/common/ID.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/network/SphereNetwork/SphereNetwork.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <vector>
#include <boost/range/adaptor/map.hpp>

REGISTER_PLUGIN(scrimmage::Network,
                scrimmage::network::SphereNetwork,
                SphereNetwork_plugin)

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace ba = boost::adaptors;

namespace scrimmage {
namespace network {

bool SphereNetwork::init(std::map<std::string, std::string> &mission_params,
                         std::map<std::string, std::string> &plugin_params) {
    network_init(mission_params, plugin_params);

    range_ = std::stod(plugin_params.at("range"));
    prob_transmit_ = std::stod(plugin_params.at("prob_transmit"));
    filter_comms_plane_ = sc::get<bool>("filter_comms_plane", plugin_params,
            filter_comms_plane_);
    if (filter_comms_plane_) {
        comms_boundary_altitude_ = sc::get<double>("comms_boundary_altitude",
                plugin_params, comms_boundary_altitude_);
        comms_boundary_epsilon_ = sc::get<double>("comms_boundary_epsilon",
                plugin_params, comms_boundary_epsilon_);
    }
    return true;
}

bool SphereNetwork::is_reachable(const scrimmage::PluginPtr &pub_plugin,
                                 const scrimmage::PluginPtr &sub_plugin) {
    // If the publisher and subscriber have the same parent, it is reachable
    if (pub_plugin->parent() == sub_plugin->parent()) return true;

    // Is the publisher to subscriber already in the reachable map?
    int pub_id = pub_plugin->parent()->id().id();
    int sub_id = sub_plugin->parent()->id().id();

    // Find the publisher in the mapping
    auto it_pub = reachable_map_.find(pub_id);
    if (it_pub != reachable_map_.end()) {
        // See if the publisher has a reachability mapping for the subscriber
        auto it_sub = it_pub->second.find(sub_id);
        if (it_sub != it_pub->second.end()) {
            return it_sub->second;
        }
    }

    // No reachability mapping exists. Find all entity IDs that are reachable.
    std::vector<sc::ID> neigh;
    rtree_->neighbors_in_range(pub_plugin->parent()->state()->pos(), neigh,
                               range_);

    // Add the entity IDs to the publisher's reachability set
    // Look for the subscriber ID
    reachable_map_[pub_id][sub_id] = false; // Default to not reachable
    reachable_map_[sub_id][pub_id] = false; // Default to not reachable
    for (sc::ID id : neigh) {
        auto ent_neighbor = id_to_ent_map_->find(id.id());
        if (ent_neighbor == id_to_ent_map_->end()) {
            std::cout << "Warning: Sphere Network entity id doesn't exists: "
                << id.id() << std::endl;
            continue;
        }
        if (not filter_comms_plane_) {
            reachable_map_[pub_id][id.id()] = true;
            reachable_map_[id.id()][pub_id] = true;
        } else if (within_planar_boundary(pub_plugin->parent()->state()->pos()[2],
                                          ent_neighbor->second->state()->pos()[2])) {
            reachable_map_[pub_id][id.id()] = true;
            reachable_map_[id.id()][pub_id] = true;
        }
    }
    return reachable_map_[pub_id][sub_id];
}

bool SphereNetwork::is_successful_transmission(const scrimmage::PluginPtr &pub_plugin,
                                               const scrimmage::PluginPtr &sub_plugin) {
    return (random_->rng_uniform(0, 1) <= prob_transmit_);
}

// Return true if the plane does not block communications
bool SphereNetwork::within_planar_boundary(double z1, double z2) {
    bool both_above = (z1 >= (comms_boundary_altitude_ - comms_boundary_epsilon_))
        && (z2 >= (comms_boundary_altitude_ - comms_boundary_epsilon_));
    bool both_below = (z1 <= (comms_boundary_altitude_ + comms_boundary_epsilon_))
        && (z2 <= (comms_boundary_altitude_ + comms_boundary_epsilon_));
    return (both_above || both_below);
}

} // namespace network
} // namespace scrimmage
