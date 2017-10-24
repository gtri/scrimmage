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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/External.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/pubsub/Network.h>

#include <iostream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace scrimmage {

External::External() :
    entity_(std::make_shared<Entity>()),
    plugin_manager_(std::make_shared<PluginManager>()),
    log_(std::make_shared<Log>()),
    last_t_(NAN) {}

bool External::create_entity(int max_entities, const ID &id,
        std::map<std::string, std::string> &info,
        const std::string &log_dir) {

    double lat = get("latitude", info, 35.721025);
    double lon = get("longitude", info, -120.767925);

    auto proj = std::make_shared<GeographicLib::LocalCartesian>(
        lat, lon, 0, GeographicLib::Geocentric::WGS84());

    info["team_id"] = std::to_string(id.team_id());

    ContactMapPtr contacts = std::make_shared<ContactMap>();
    std::shared_ptr<RTree> rtree = std::make_shared<RTree>();
    rtree->init(max_entities);

    FileSearch file_search;
    entity_ = std::make_shared<Entity>();
    auto network = std::make_shared<Network>();

    auto mp = std::make_shared<MissionParse>();
    mp->set_log_dir(log_dir);
    mp->create_log_dir();

    log_->set_enable_log(true);
    log_->init(mp->log_dir(), Log::WRITE);

    AttributeMap overrides;
    bool success =
        entity_->init(
            overrides, info, contacts, mp, proj, id.id(), id.sub_swarm_id(),
            plugin_manager_, network, file_search, rtree);

    if (!success) {
        return false;
    } else {
        RandomPtr random = std::make_shared<Random>();
        random->seed();
        entity_->set_random(random);
        return true;
    }
}

EntityPtr &External::entity() {
    return entity_;
}

} // namespace scrimmage
