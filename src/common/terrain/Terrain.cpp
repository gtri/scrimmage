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

#include "scrimmage/parse/TerrainReaders/VTKTerrainReader.h"
#include <scrimmage/common/terrain/Terrain.h>
#include <scrimmage/common/terrain/TerrainMap.h>
#include <scrimmage/common/terrain/WGSTerrainMap.h>
#include <scrimmage/common/terrain/UTMTerrainMap.h>
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

namespace scrimmage {

    Terrain::Terrain():
      is_published_(false), successful_init_(false)
    {}

    bool Terrain::init(const scrimmage_proto::UTMTerrain& utm_terrain) 
    {

      std::string terrain_filename{utm_terrain.poly_data_file()}

      std::size_t extension_ind = terrain_filename.find_last_of(".");
      if(extension_ind == std::string::npos) { 
        std::cout << "Terrain file \'" << terrain_filename 
          << "\' does not have proper extension\n" ;
        return true;
      }
      std::string extension = terrain_filename.substr(extension_ind + 1);

      // Deterine instance of TerrainReader and TerrainMap to
      // make available
      TerrainReader tr;
      if (extension.find("vtk") == 0) {
        terrain_map_ = std::make_shared<UTMTerrainMap>();
        tr = VTKTerrainReader(terrain_filename);
      } else if (extension.find("dt") == 0 && extension.size() == 3) {
        terrain_map_ = std::make_shared<WGSTerrainMap>();
        tr = DTEDReader(terrain_filename);
      } else {
        std::cout << "Terrain file \'" << terrain_filename 
          << "\' does not have proper extension\n" ;
        return true;
      }

      successful_init_ = terrain_map_->init(
          tr.Parse(), 
          utm_terrain);
    }
} // namespace scrimmage
