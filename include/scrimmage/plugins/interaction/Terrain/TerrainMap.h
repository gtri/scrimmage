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

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace scrimmage {
  namespace interaction {
    class TerrainMap {
      public:
        TerrainMap(); 

        bool init(int utm_zone_, std::string filename);

        // ----- Query Functions ----
        std::optional<double> queryTerrain(const double xpos, const double ypos) const;
        int utm_zone() { return utm_zone_; }

      protected:
        bool initFromVTK(std::string filename);

        void search_y(std::vector<double> const& y_vec,
            double positionY, int* y_index, int *vec_width) const;

        void search_x(std::vector<double> const& x_vec,
            double positionX, int search_start, int vec_width, int *x_index) const;

        int utm_zone_;

        std::array<std::vector<double>, 3> elevation_map_;

      private:
    };

  } // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINMAP_TERRAINMAP_H_
