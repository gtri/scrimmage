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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINMAP_TERRAINMAP_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINMAP_TERRAINMAP_H_

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace scrimmage {
  namespace interaction {
    class TerrainMap {
      public:
        TerrainMap(); 

        virtual bool init(
            const std::string& filename,
            const int utm_zone,
            const bool northern_hemisphere = true) = 0;

        // ----- Query Functions ----
        virtual std::optional<double> QueryUTM(
            const double easting, const double northing) const = 0;

        virtual std::optional<double> QueryLongLat(
            const double longitude, const double latitude) const = 0;


        // ----- Accessors -------
        int utm_zone() const { return utm_zone_; }
        bool northern_hemisphere() const { return utm_northern_hemisphere_; }


      protected:
        //bool InitFromVTK(const std::string filename);
        //bool InitFromDTED(const std::string filename);
        
        std::optional<double> QueryTerrain(
            const double xpos, const double ypos) const;
  
        void SearchY(std::vector<double> const& y_vec,
            double positionY, int* y_index, int *vec_width) const;

        void SearchX(std::vector<double> const& x_vec,
            double positionX, int search_start, int vec_width, int *x_index) const;
        

        int utm_zone_;
        bool utm_northern_hemisphere_;

        std::unique_ptr<std::array<std::vector<double>, 3>> elevation_map_;
        std::size_t number_points() const { 
          if(elevation_map_) {
            return elevation_map_->at(0).size(); 
          } else {
            return 0;
          }
        }

      private:
    };

    using TerrainMapPtr = std::shared_ptr<TerrainMap>;
  } // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINMAP_TERRAINMAP_H_
