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

#include <memory>

#include <scrimmage/entity/External.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/parse/MissionParse.h>

namespace scrimmage {

External::External() : plugin_manager_(std::make_shared<PluginManager>()) {}

NetworkPtr &External::network() {return network_;}

EntityPtr &External::entity() {return entity_;}

FileSearch &External::file_search() {return file_search_;}

PluginManagerPtr &External::plugin_manager() {return plugin_manager_;}

int External::get_max_entities() {return max_entities_;}

void External::set_max_entities(int max_entities) {max_entities_ = max_entities;}

bool External::create_entity(ID id,
        std::map<std::string, std::string> &info,
        const std::string &log_dir) {
    // info needs the following keys:
    // latitude, longitude, x, y, z, heading, autonomy_name, controller
    entity_ = std::make_shared<Entity>();

    double lat = std::stod(info["latitude"]);
    double lon = std::stod(info["longitude"]);

    auto proj = std::make_shared<GeographicLib::LocalCartesian>(
        lat, lon, 0, GeographicLib::Geocentric::WGS84()
    );

    info["team_id"] = std::to_string(id.team_id());

    network_ = std::make_shared<Network>();

    ContactMapPtr contacts = std::make_shared<ContactMap>();
    std::shared_ptr<RTree> rtree = std::make_shared<RTree>();
    rtree->init(max_entities_);

    MissionParsePtr mp(new MissionParse());
    mp->set_log_dir(log_dir);
    AttributeMap overrides;
    bool success =
        entity_->init(
            overrides, info, contacts, mp, proj, id.id(), id.sub_swarm_id(),
            plugin_manager_, network_, file_search_, rtree);

    if (!success) {
        return false;
    }

    RandomPtr random = std::make_shared<Random>();
    entity_->set_random(random);

    return true;
}

} // namespace scrimmage
