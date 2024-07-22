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
 * @author Edward Stevens <edward.stevens@gtri.gatech.edu>
 * @author William Syre <william.syre@gtri.gatech.edu>
 * @date 31 June 2022
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Shape.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/terrain/TerrainMap.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/interaction/TerrainCollision/TerrainCollision.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/LocalCartesian.hpp>

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::TerrainCollision,
                TerrainCollision_plugin)

namespace scrimmage {
namespace interaction {

TerrainCollision::TerrainCollision()
    : remove_on_collision_(true),
      enable_startup_collisions_(true),
      interpolate_(false) {}

bool TerrainCollision::init(std::map<std::string, std::string> &mission_params,
                            std::map<std::string, std::string> &plugin_params) {
  interpolate_ = scrimmage::get<bool>("interpolate_terrain", plugin_params, false);

  return true;
}

bool TerrainCollision::step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                               double t,
                                               double dt) {
  if (!terrain_map_) {
    return true;
  }
  for (scrimmage::EntityPtr ent : ents) {
    // Position conversion to UTM
    double x = ent->state()->pos().x();
    double y = ent->state()->pos().y();
    double z = ent->state()->pos().z();
    double lat, lon, alt;

    Eigen::Vector3d start = {x, y, z};
    Eigen::Vector3d end = {x, y, z + 25};
    parent_->projection()->Reverse(x, y, z, lat, lon, alt);

    double elevation = terrain_map_->QueryLongLat(lon, lat, interpolate_);
    bool collision = elevation >= z;

    if (collision) {
      ShapePtr segment = scrimmage::shape::make_line(start, end, Eigen::Vector3d(255, 0, 0), 1);
      draw_shape(segment);
      std::cout << "[TC] Terrain Collision: Agent " << ent->id().id() << " Altitude: " << z << " m"
                << std::endl;

      ent->collision();
    }
  }
  if (ents.empty()) {
    return true;
  }

  return true;
}
}  // namespace interaction
}  // namespace scrimmage
