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
#include <scrimmage/common/terrain/WGSTerrainMap.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <memory>
#include <optional>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace scrimmage {
namespace terrain {
WGSTerrainMap::WGSTerrainMap() {}

bool WGSTerrainMap::init(std::unique_ptr<ElevationGrid> elevation_grid,
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

/*
 * Queries terrain map by choosing the closest x and y positions
 * to the ones provided to determine the terrain elevation.
 * The elevation map is layed out as three vectors corresponding
 * to the x (index 0), y (index 1), and z (index 2) directions.
 *
 * Searching these values can be thought of as searching a 2D matrix formed
 * by the x and y values with y-values as rows, and x-values as columns.
 * It is assumed that all y-values (rows) are increasing, and all
 * x-values (colums) are increasing within each row. (This is for efficent
 * search using std::upper_bound/lower_bound).
 */
double WGSTerrainMap::QueryLongLat(const double longitude,
                                   const double latitude,
                                   const bool interpolate) const {
  return elevation_grid_->Query(longitude, latitude, interpolate) - z_translate_;
}

double WGSTerrainMap::QueryUTM(const double easting,
                               const double northing,
                               const bool interpolate) const {
  GeographicLib::GeoCoords GC =
      GeographicLib::GeoCoords(utm_zone_, utm_northern_hemisphere_, easting, northing);

  return QueryLongLat(GC.Longitude(), GC.Latitude(), interpolate);
}

std::optional<double> WGSTerrainMap::RaycastUTM(const double easting,
                                                const double northing,
                                                const double altitude,
                                                const double polar,
                                                const double azithmual,
                                                const double max_distance,
                                                const bool interpolate) const {
  GeographicLib::GeoCoords GC =
      GeographicLib::GeoCoords(utm_zone_, utm_northern_hemisphere_, easting, northing);

  return RaycastLongLat(
      GC.Longitude(), GC.Latitude(), altitude, polar, azithmual, max_distance, interpolate);
}

std::optional<double> WGSTerrainMap::RaycastLongLat(const double longitude,
                                                    const double latitude,
                                                    const double altitude,
                                                    const double polar,
                                                    const double azithmual,
                                                    const double max_distance,
                                                    const bool interpolate) const {
  std::optional<double> raw_elevation = elevation_grid_->Raycast(
      longitude, latitude, altitude, polar, azithmual, max_distance, interpolate);

  if (raw_elevation) {
    raw_elevation.value() -= z_translate_;
  }
  return raw_elevation;
}

}  // namespace terrain
}  // namespace scrimmage
