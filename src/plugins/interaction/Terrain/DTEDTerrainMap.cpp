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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugins/interaction/Terrain/TerrainMap.h>
#include <scrimmage/plugins/interaction/Terrain/DTEDTerrainMap.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/parse/DTEDParse.h>

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/GeoCoords.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace scrimmage {
  namespace interaction {

    DTEDTerrainMap::DTEDTerrainMap() {};

    bool DTEDTerrainMap::init(const std::string& filename, const int utm_zone, const bool northern_hemisphere) {
      constexpr int max_utm_zone = 60;
      constexpr int min_utm_zone = 1;
      utm_zone_ = utm_zone;
      utm_northern_hemisphere_ = northern_hemisphere;

      if (utm_zone_ < min_utm_zone || utm_zone_ > max_utm_zone) {
        std::cout << "The utm zone \'" << utm_zone << "\' is invalid."
          "Valid Zones must be between " << min_utm_zone
          << " and " << max_utm_zone << "\n";
        return false;
      }
      bool init_success = InitFromFile(filename);
      const std::vector<double>& y_vec = elevation_map_->at(1);
      stride_ = std::upper_bound(y_vec.begin(), y_vec.end(), y_vec[0]) - y_vec.begin();
      return init_success;
    }

    bool DTEDTerrainMap::InitFromFile(const std::string& filename) {
      elevation_map_ = DTEDParse::Parse(filename);  
      return true;
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
    std::optional<double> DTEDTerrainMap::QueryLongLat(
        const double longitude, 
        const double latitude, 
        const bool interpolate) const {
      return Query(longitude, latitude, interpolate);
    }

    std::optional<double> DTEDTerrainMap::QueryUTM(
        const double easting, 
        const double northing,
        const bool interpolate) const {
      
      GeographicLib::GeoCoords GC = GeographicLib::GeoCoords(
          utm_zone_, 
          utm_northern_hemisphere_,
          easting,
          northing);

      return QueryLongLat(GC.Latitude(), GC.Longitude());
      
    }
  } // namespace interaction
} // namespace scrimmage
