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
#include <scrimmage/common/terrain/TerrainMap.h>
#include <scrimmage/common/terrain/UTMTerrainMap.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/TerrainReaders/VTKTerrainReader.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <optional>

#include <GeographicLib/GeoCoords.hpp>

// TerrainMap that is indexed directly with UTM Coordinates

namespace scrimmage {
namespace terrain {
UTMTerrainMap::UTMTerrainMap() {}

bool UTMTerrainMap::init(std::unique_ptr<ElevationGrid> elevation_grid,
                         const scrimmage_proto::UTMTerrain& utm) {
  constexpr int max_utm_zone = 60;
  constexpr int min_utm_zone = 1;
  utm_zone_ = utm.zone();
  utm_northern_hemisphere_ = utm.hemisphere() == "north";
  z_translate_ = utm.z_translate();

  if (utm_zone_ < min_utm_zone || utm_zone_ > max_utm_zone) {
    std::cout << "The utm zone \'" << utm_zone_
              << "\' is invalid."
                 "Valid Zones must be between "
              << min_utm_zone << " and " << max_utm_zone << "\n";
    return false;
  }
  elevation_grid_.swap(elevation_grid);
  return elevation_grid_ != nullptr;
}

double UTMTerrainMap::QueryUTM(const double easting,
                               const double northing,
                               const bool interpolate) const {
  // Default Coordinates are already in easting/northing
  return elevation_grid_->Query(easting, northing, interpolate) - z_translate_;
}

double UTMTerrainMap::QueryLongLat(const double longitude,
                                   const double latitude,
                                   const bool interpolate) const {
  GeographicLib::GeoCoords GC = GeographicLib::GeoCoords(latitude, longitude, utm_zone_);
  return QueryUTM(GC.Easting(), GC.Northing(), interpolate);
}

std::optional<double> UTMTerrainMap::RaycastUTM(const double easting,
                                                const double northing,
                                                const double altitude,
                                                const double polar,
                                                const double azithmual,
                                                const double max_distance,
                                                const bool interpolate) const {
  std::optional<double> raw_elevation = elevation_grid_->Raycast(
      easting, northing, altitude, polar, azithmual, max_distance, interpolate);

  if (raw_elevation) {
    raw_elevation.value() -= z_translate_;
  }
  return raw_elevation;
}

std::optional<double> UTMTerrainMap::RaycastLongLat(const double longitude,
                                                    const double latitude,
                                                    const double altitude,
                                                    const double polar,
                                                    const double azithmual,
                                                    const double max_distance,
                                                    const bool interpolate) const {
  GeographicLib::GeoCoords GC = GeographicLib::GeoCoords(latitude, longitude, utm_zone_);
  return RaycastUTM(
      GC.Easting(), GC.Northing(), altitude, polar, azithmual, max_distance, interpolate);
}
}  // namespace terrain
}  // namespace scrimmage
