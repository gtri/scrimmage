/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2024 by the Georgia Tech Research Institute (GTRI)
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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 31 Jan 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/Terrain/Terrain.h>
#include <scrimmage/plugins/interaction/Terrain/TerrainMap.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <memory>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
    scrimmage::interaction::Terrain,
    Terrain_plugin)

  namespace scrimmage {
    namespace interaction {

      Terrain::Terrain():
        is_published_(false), successful_init_(false)
      {}

      bool Terrain::init(std::map<std::string, std::string> &mission_params,
          std::map<std::string, std::string> &plugin_params) 
      {
        terrain_map_ = std::make_shared<TerrainMap>();
        std::string terrain_filename = scrimmage::get<std::string>(
            "terrain_interaction",
            plugin_params,
            "");
        
        std::string terrain_topic = "Elevation"; // Eventually get from params

        terrain_filename = parent()->mp()->utm_terrain()->poly_data_file();
        std::cout << "Terrain: " << terrain_filename << std::endl;
        successful_init_ = terrain_map_->init(
            terrain_filename,
            parent()->mp()->utm_terrain()->zone(),
            parent()->mp()->utm_terrain()->hemisphere() == "NORTH");

        if(successful_init_) {
          pub_terrain_ = advertise("GlobalNetwork", terrain_topic);
        }
        return true;
      }


      bool Terrain::step_entity_interaction(std::list<sc::EntityPtr> &ents,
          double t, double dt) {

        // We only want to publish a ptr to a successfully initalized map
        if (!is_published_ && successful_init_) {
          auto msg = std::make_shared<scrimmage::Message<TerrainMapPtr>>(terrain_map_);
          pub_terrain_->publish(msg);
        }
        return true;
      }
    } // namespace interaction
  } // namespace scrimmage
