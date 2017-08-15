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
#include <scrimmage/common/ID.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugins/network/UnitDisk/UnitDisk.h>

#include <vector>
#include <boost/range/adaptor/map.hpp>

REGISTER_PLUGIN(scrimmage::Network, UnitDisk, UnitDisk_plugin)

namespace sc = scrimmage;
namespace ba = boost::adaptors;

void UnitDisk::init(std::map<std::string, std::string> &params) {
    range_ = std::stod(params.at("range"));
}

void UnitDisk::distribute(double t, double dt) {

    ping_map_.clear();

    for (auto &pub_map_kv : pub_map_) {

        std::string topic = pub_map_kv.first;

        for (sc::NetworkDevicePtr &pub : ba::values(pub_map_kv.second)) {

            if (pub->msg_list().empty()) {
                continue;
            }

            std::unordered_set<int> &pingable_ids = ping_ref(pub->plugin());
            auto beg = pub->msg_list().begin();
            auto end = pub->msg_list().end();

            for (sc::NetworkDevicePtr &sub : ba::values(sub_map_[topic])) {
                if (pingable_ids.count(sub->plugin()->get_network_id()) != 0) {
                    sub->msg_list().insert(sub->msg_list().end(), beg, end);
                }
            }

            pub->msg_list().clear();
        }
    }
}

std::unordered_set<int> &UnitDisk::ping_ref(const scrimmage::PluginPtr &plugin) {

    int dev_id = plugin->get_network_id();
    auto it = ping_map_.find(dev_id);

    if (it == ping_map_.end()) {

        std::unordered_set<int> pingable_ids;

        Eigen::Vector3d &pos = plugin->parent()->state()->pos();
        std::vector<sc::ID> neigh;
        rtree_->neighbors_in_range(pos, neigh, range_);

        std::unordered_set<int> neigh_set;
        std::transform(neigh.begin(), neigh.end(),
            std::inserter(neigh_set, neigh_set.end()),
            [&](sc::ID &id) {return id.id();});

        auto add_ids = [&](TopicMap &topic_map) {
            for (auto &dev_map : ba::values(topic_map)) {
                for (auto &dev : ba::values(dev_map)) {
                    const sc::EntityPtr &dev_ent = dev->plugin()->parent();
                    int entity_id = dev_ent->id().id();
                    bool in_range = neigh_set.count(entity_id) != 0;
                    bool no_pos = dev_ent->health_points() <= 0;

                    if (in_range | no_pos) {
                        pingable_ids.insert(dev->plugin()->get_network_id());
                    }
                }
            }
        };

        add_ids(pub_map_);
        add_ids(sub_map_);

        it = ping_map_.insert(std::make_pair(dev_id, pingable_ids)).first;
    }

    return it->second;
}

std::unordered_set<int> UnitDisk::ping(const scrimmage::PluginPtr &plugin) {
    return ping_ref(plugin);
}

bool UnitDisk::ping(const scrimmage::PluginPtr &plugin, int network_id) {
    return ping_ref(plugin).count(network_id) != 0;
}
