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

#include <scrimmage/plugins/interaction/Terrain/TerrainMap.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace scrimmage {
namespace interaction {

  TerrainMap::TerrainMap():
    utm_zone_(0) {};


  bool TerrainMap::init(const int utm_zone, const std::string filename) {
    constexpr int max_zone = 60;
    constexpr int min_zone = 1;
    utm_zone_ = utm_zone;
    return initFromVTK(filename) && (min_zone <= utm_zone) && (utm_zone <= max_zone);
  }

  bool TerrainMap::initFromVTK(const std::string filename) {
    // Use vtkPolyReader
        // Read the terrain polydata
        vtkSmartPointer<vtkPolyDataReader> elevation_reader =
            vtkSmartPointer<vtkPolyDataReader>::New();

        elevation_reader->SetFileName(filename.c_str());
        bool validFile = elevation_reader->IsFilePolyData() != 0;
        if (!validFile) { 
          std::cout << "Invalid VTK File: \'" << filename <<
            "\'. Elevation information is unavailable\n";
          return false; 
        }

        elevation_reader->Update();
        vtkSmartPointer<vtkPolyData> polydata;
        polydata = elevation_reader->GetOutput();

        std::size_t number_points = polydata->GetNumberOfPoints();
        for(int i = 0; i < 3; i++) {
          elevation_map_[i].reserve(number_points);
        }

        for(size_t n = 0; n < number_points; n++){
          // Copies the point information from polydata into the raw array
          // of the elevation map
          std::array<double, 3> tmp_point;
          polydata->GetPoint(static_cast<vtkIdType>(n), tmp_point.data());
          //Sort for binary lookup? 
          elevation_map_[0].push_back(tmp_point[0]);
          elevation_map_[1].push_back(tmp_point[1]);
          elevation_map_[2].push_back(tmp_point[2]);
        }
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
  std::optional<double> TerrainMap::queryTerrain(
      const double xpos, const double ypos) const {
    // Do some checks here that we are within the 
    // boundary of terrain I suppose
    int y_idx, x_idx, stride;
    
    const std::vector<double>& x_vec = elevation_map_[0];
    const std::vector<double>& y_vec = elevation_map_[1];
    const std::vector<double>& z_vec = elevation_map_[2];
    
    // Find the width" of the terrain. In otherwords, how many 
    // x-values corresond to a single y-value
    stride = std::upper_bound(y_vec.begin(), y_vec.end(), y_vec[0]) - y_vec.begin();
    y_idx = std::lower_bound(y_vec.begin(), y_vec.end(), ypos) - y_vec.begin(); 

    auto x_search_start = x_vec.begin() + y_idx;
    x_idx = std::lower_bound(x_search_start, x_search_start + stride, xpos) - x_vec.begin();
    
    // We may want to interpolate here at some point, as we are 
    // always selecting the lower bound of indicies. But for now 
    // just reutrn the z-value

    return  z_vec[x_idx];
  }
} // namespace interaction
} // namespace scrimmage
