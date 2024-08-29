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
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/gpu/OpenCLUtils.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/network/GPUSphereNetwork/GPUSphereNetwork.h>
#include <scrimmage/plugins/network/GPUSphereNetwork/GPUSphereNetworkUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <CL/cl.h>

#include <memory>

#include <CL/opencl.hpp>
#include <boost/range/adaptor/map.hpp>

REGISTER_PLUGIN(scrimmage::Network, scrimmage::network::GPUSphereNetwork, GPUSphereNetwork_plugin)

namespace sc = scrimmage;

namespace scrimmage {
namespace network {

bool GPUSphereNetwork::init(std::map<std::string, std::string>& mission_params,
                            std::map<std::string, std::string>& plugin_params) {
    network_init(mission_params, plugin_params);
    range_ = std::stod(plugin_params.at("range"));
    prob_transmit_ = std::stod(plugin_params.at("prob_transmit"));
    filter_comms_plane_ = sc::get<bool>("filter_comms_plane", plugin_params, filter_comms_plane_);
    network_name_ = sc::get<std::string>("name", plugin_params, "SphereNetwork");
    if (filter_comms_plane_) {
        comms_boundary_altitude_ =
            sc::get<double>("comms_boundary_altitude", plugin_params, comms_boundary_altitude_);
        comms_boundary_epsilon_ =
            sc::get<double>("comms_boundary_epsilon", plugin_params, comms_boundary_epsilon_);
    }
    std::string kernel_name =
        sc::get<std::string>("network_kernel", plugin_params, "GPUSphereNetwork");

    GPUControllerPtr gpu = parent()->gpu_controller();
    const std::map<std::string, GPUPluginBuildParams>& gpu_plugin_params = gpu->get_plugin_params();
    assert(gpu_plugin_params.count(kernel_name) > 0);
    const GPUPluginBuildParams& network_kernel_params = gpu_plugin_params.at(kernel_name);

    utils_ = std::make_unique<GPUSphereNetworkUtils>(range_, network_kernel_params);
    return true;
}

bool GPUSphereNetwork::step(std::map<std::string, std::list<NetworkDevicePtr>>& pubs,
                            std::map<std::string, std::list<NetworkDevicePtr>>& subs) {
    using EntityIdPair = std::pair<int, int>;


    std::map<int, StatePtr> states;
    std::map<int, EntityPtr> ents;

    std::map<int, std::map<std::string, std::list<NetworkDevicePtr>>> ent_pubs;
    std::map<int, std::map<std::string, std::list<NetworkDevicePtr>>> ent_subs;

    for (const auto& kv : pubs) {
        const std::string& topic = kv.first;
        for (NetworkDevicePtr device : kv.second) {
            EntityPtr entity = device->plugin()->parent();
            StatePtr state = entity->state_truth();
            int id = entity->id().id();
            states[id] = state;
            ents[id] = entity;
            ent_pubs[id][topic].push_back(device);
        }
    }

    for (const auto& kv : subs) {
        const std::string& topic = kv.first;
        for (NetworkDevicePtr device : kv.second) {
            EntityPtr entity = device->plugin()->parent();
            StatePtr state = entity->state_truth();
            int id = entity->id().id();
            states[id] = state;
            ents[id] = entity;
            ent_subs[id][topic].push_back(device);
        }
    }

    std::size_t num_entities = states.size();
    if (num_entities <= 1) {
        return true;
    }

    std::set<EntityIdPair> proximity_pairs = utils_->proximity_pairs(states);
    for (const EntityIdPair& prox_pair : proximity_pairs) {
        EntityPtr ent1 = ents.at(prox_pair.first);
        EntityPtr ent2 = ents.at(prox_pair.second);

        Eigen::Vector3d pos1 = states.at(prox_pair.first)->pos();
        Eigen::Vector3d pos2 = states.at(prox_pair.second)->pos();

        const std::map<std::string, std::list<NetworkDevicePtr>>& pubs1 = ent_pubs[prox_pair.first];
        const std::map<std::string, std::list<NetworkDevicePtr>>& subs1 = ent_subs[prox_pair.first];

        const std::map<std::string, std::list<NetworkDevicePtr>>& pubs2 = ent_pubs[prox_pair.second];
        const std::map<std::string, std::list<NetworkDevicePtr>>& subs2 = ent_subs[prox_pair.second];

        deliver_messages(pubs1, subs2);
        deliver_messages(pubs2, subs1);
    }
    return true;
}

bool GPUSphereNetwork::deliver_messages(
    const std::map<std::string, std::list<NetworkDevicePtr>>& pubs,
    const std::map<std::string, std::list<NetworkDevicePtr>>& subs) {
    for (const auto& kv : pubs) {
        const std::string& topic_name = kv.first;
        const std::list<NetworkDevicePtr>& topic_pubs = kv.second;
        const std::list<NetworkDevicePtr>& topic_subs = subs.at(topic_name);

        for (NetworkDevicePtr topic_pub : topic_pubs) {
            deliver_topic_messages(topic_pub, topic_subs);
        }
    }
    return true;
}

bool GPUSphereNetwork::deliver_topic_messages(NetworkDevicePtr pub,
                                              const std::list<NetworkDevicePtr>& subs) {
    pub->enforce_queue_size();
    const std::list<MessageBasePtr> msgs = pub->pop_msgs<MessageBase>();
    if (msgs.empty()) {
        return true;
    }
    for (NetworkDevicePtr sub : subs) {
        if (sub->undelivered_msg_list_size() > 0) {
            // deliver undelivered messages if delay time has passed
            sub->deliver_undelivered_msg(time_->t(), is_stochastic_delay_);
        }
        for (auto& msg : msgs) {
            if (is_successful_transmission(pub->plugin(), sub->plugin())) {
                double msg_delay = get_transmission_delay();

                if (msg_delay < 0) {
                    msg->time = time_->t();
                    sub->add_msg(msg);
                } else {
                    // put msg in subs undelivered msg queue
                    msg->time = time_->t() + msg_delay;
                    sub->add_undelivered_msg(msg, is_stochastic_delay_);
                }
            }
        }
    }
    return true;
}


bool GPUSphereNetwork::is_reachable(const scrimmage::EntityPluginPtr& pub_plugin,
                                    const scrimmage::EntityPluginPtr& sub_plugin) {
    // Never reachable if plugin's entity was destroyed
    if (pub_plugin->parent() == nullptr || sub_plugin->parent() == nullptr) return false;
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
    rtree_->neighbors_in_range(pub_plugin->parent()->state_truth()->pos(), neigh, range_);

    // Add the entity IDs to the publisher's reachability set
    // Look for the subscriber ID
    reachable_map_[pub_id][sub_id] = false;  // Default to not reachable
    reachable_map_[sub_id][pub_id] = false;  // Default to not reachable
    for (sc::ID id : neigh) {
        auto ent_neighbor = id_to_ent_map_->find(id.id());
        if (ent_neighbor == id_to_ent_map_->end()) {
            std::cout << "Warning: Sphere Network entity id doesn't exists: " << id.id()
                      << std::endl;
            continue;
        }
        if (not filter_comms_plane_) {
            reachable_map_[pub_id][id.id()] = true;
            reachable_map_[id.id()][pub_id] = true;
        } else if (within_planar_boundary(pub_plugin->parent()->state_truth()->pos()[2],
                                          ent_neighbor->second->state_truth()->pos()[2])) {
            reachable_map_[pub_id][id.id()] = true;
            reachable_map_[id.id()][pub_id] = true;
        }
    }
    return reachable_map_[pub_id][sub_id];
}

bool GPUSphereNetwork::is_successful_transmission(const scrimmage::EntityPluginPtr& pub_plugin,
                                                  const scrimmage::EntityPluginPtr& sub_plugin) {
    return (random_->rng_uniform(0, 1) <= prob_transmit_);
}

// Return true if the plane does not block communications
bool GPUSphereNetwork::within_planar_boundary(double z1, double z2) {
    bool both_above = (z1 >= (comms_boundary_altitude_ - comms_boundary_epsilon_)) &&
                      (z2 >= (comms_boundary_altitude_ - comms_boundary_epsilon_));
    bool both_below = (z1 <= (comms_boundary_altitude_ + comms_boundary_epsilon_)) &&
                      (z2 <= (comms_boundary_altitude_ + comms_boundary_epsilon_));
    return (both_above || both_below);
}

}  // namespace network
}  // namespace scrimmage
