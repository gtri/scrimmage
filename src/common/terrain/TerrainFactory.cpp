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

#include <scrimmage/common/Random.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/terrain/TerrainFactory.h>
#include <scrimmage/common/terrain/TerrainMap.h>
#include <scrimmage/common/terrain/UTMTerrainMap.h>
#include <scrimmage/common/terrain/WGSTerrainMap.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/TerrainReaders/DTEDReader.h>
#include <scrimmage/parse/TerrainReaders/TerrainReader.h>
#include <scrimmage/parse/TerrainReaders/VTKTerrainReader.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <filesystem>
#include <regex>  // NOLINT

namespace scrimmage {
namespace terrain {

namespace fs = std::filesystem;

TerrainMapPtr TerrainFactory::MakeTerrain(
    std::shared_ptr<scrimmage_proto::UTMTerrain> utm_terrain) {
  fs::path terrain_file{utm_terrain->poly_data_file()};
  if (!fs::exists(terrain_file)) {
    return nullptr;
  }

  std::string extension = terrain_file.extension().string();
  if (extension != ".vtk" && extension.find("dt") == std::string::npos) {
    std::cout << "Terrain file \'" << terrain_file << "\' does not have proper extension\n";
    return nullptr;
  }
  std::regex vtk_re{R"_(.vtk)_"};
  std::regex dted_re{R"_(.dt[0-2])_"};

  // Determine instance of TerrainReader and TerrainMap to
  // make available
  TerrainMapPtr terrain_map = nullptr;
  std::unique_ptr<TerrainReader> terrain_reader = nullptr;
  if (std::regex_search(extension, vtk_re)) {
    terrain_map = std::make_shared<UTMTerrainMap>();
    terrain_reader =
        std::make_unique<VTKTerrainReader>(terrain_file, utm_terrain->terrain_extent());
  } else if (std::regex_search(extension, dted_re)) {
    terrain_map = std::make_shared<WGSTerrainMap>();
    terrain_reader = std::make_unique<DTEDReader>(terrain_file, utm_terrain->terrain_extent());
  } else {
    std::cout << "Terrain file \'" << terrain_file << "\' does not have proper extension\n";
    return nullptr;
  }
  terrain_map->init(terrain_reader->Parse(), *utm_terrain);
  return terrain_map;
}
}  // namespace terrain
}  // namespace scrimmage
