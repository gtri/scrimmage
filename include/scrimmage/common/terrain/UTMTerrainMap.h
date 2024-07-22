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

#ifndef INCLUDE_SCRIMMAGE_COMMON_TERRAIN_UTMTERRAINMAP_H_
#define INCLUDE_SCRIMMAGE_COMMON_TERRAIN_UTMTERRAINMAP_H_

#include <scrimmage/common/terrain/TerrainMap.h>
#include <scrimmage/proto/Visual.pb.h>

#include <memory>
#include <optional>
#include <string>

namespace scrimmage {
namespace terrain {
class UTMTerrainMap : public TerrainMap {
 public:
  UTMTerrainMap();

  bool init(std::unique_ptr<ElevationGrid> elevation_grid,
            const scrimmage_proto::UTMTerrain& utm) override;

  // ----- Query Functions ----
  double QueryUTM(const double easting,
                  const double northing,
                  const bool interpolate = false) const override;

  double QueryLongLat(const double longitude,
                      const double latitude,
                      const bool interpolate = false) const override;

  std::optional<double> RaycastUTM(const double easting,
                                   const double northing,
                                   const double altitude,
                                   const double polar,
                                   const double azithmual,
                                   const double max_distance,
                                   const bool interpolate = false) const override;

  std::optional<double> RaycastLongLat(const double longitude,
                                       const double latitude,
                                       const double altitude,
                                       const double polar,
                                       const double azithmual,
                                       const double max_distance,
                                       const bool interpolate = false) const override;

 protected:
  bool InitFromFile(const std::string& filename);

 private:
};
}  // namespace terrain
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_TERRAIN_UTMTERRAINMAP_H_
