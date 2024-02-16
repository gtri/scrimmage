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
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

//#include <vtkSmartPointer.h>
//#include <vtkPolyData.h>
//#include <vtkPolyDataReader.h>

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace scrimmage {
  namespace interaction {

    TerrainMap::TerrainMap():
      utm_zone_(0), utm_northern_hemisphere_(true), stride_(0) {};

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
    std::optional<double> TerrainMap::Query(
        const double xpos, const double ypos, const bool interpolate) const {
      std::size_t y_idx, pt_idx;

      const std::vector<double>& x_vec = elevation_map_->at(0);
      const std::vector<double>& y_vec = elevation_map_->at(1);
      const std::vector<double>& z_vec = elevation_map_->at(2);

      // Find the width" of the terrain. In otherwords, how many 
      // x-values corresond to a single y-value
      y_idx = std::lower_bound(y_vec.begin(), y_vec.end(), ypos) - y_vec.begin(); 

      auto x_search_start = x_vec.begin() + y_idx;
      pt_idx = std::lower_bound(x_search_start, x_search_start + stride_, xpos) - x_vec.begin();

      if (interpolate) {
        return Interpolate(xpos, ypos, pt_idx);
      }

      // We may want to interpolate here at some point, as we are 
      // always selecting the lower bound of indicies. But for now 
      // just reutrn the z-value

      return  z_vec[pt_idx];
    }

    double TerrainMap::Interpolate(const double  xpos, 
        const double ypos, 
        const std::size_t pt_idx0) const {

      const std::vector<double>& x_vec = elevation_map_->at(0);
      const std::vector<double>& y_vec = elevation_map_->at(1);
      const std::vector<double>& z_vec = elevation_map_->at(2);

      // Is the sample point at the end of a row (i.e. it does not 
      // have a proper right neighbor)
      bool is_in_rightmost_col = pt_idx0 % stride_ == (stride_ - 1);

      // Is the sample point at the top of our matrix (i.e. it does not 
      // have a proper top neighbor)
      bool is_in_top_row = (pt_idx0 / stride_) == ((y_vec.size()-1) / stride_);

      if(!is_in_rightmost_col && !is_in_top_row) {
        /* These pts define the points of a quad (right now we assume a 
         * square in the interpolation). As the original pt_idx we find
         * is always the tightest lower-bound for both xpos and ypos,
         * it is the lower-left corner of this quad. The rest of the
         * quad is layed out as follows.
         * 2------3
         * |      |
         * |      |
         * 0------1
         *
         *  Coordinate components are indexed as: 
         *    0 = bottom_left
         *    1 = bottom_right
         *    2 = top_left
         *    3 = top_right
         */ 
        int pt_idx1, pt_idx2, pt_idx3; 
        double z0, z1, z2, z3, 
               x0, x1, y0, y2,
               fx, fy; 

        double x_spacing, y_spacing, dx, dy;
        double z0_weight, z1_weight, z2_weight, z3_weight;

        pt_idx1 = pt_idx0 + 1;
        pt_idx2 = pt_idx0 + stride_;
        pt_idx3 = pt_idx0 + stride_ + 1;

        z0 = z_vec[pt_idx0];          
        z1 = z_vec[pt_idx1];          
        z2 = z_vec[pt_idx2];          
        z3 = z_vec[pt_idx3];          

        x0 = x_vec[pt_idx0];          
        x1 = x_vec[pt_idx1];          

        y0 = y_vec[pt_idx0];          
        y2 = y_vec[pt_idx2];          

        x_spacing = x1 - x0;
        y_spacing = y2 - y0;
        dx = xpos - x0;
        dy = ypos - y0;

        fx = dx  / x_spacing;
        fy = dy / y_spacing;

        z0_weight = (1-fx)*(1-fy);
        z1_weight = (fx)*(1-fy);
        z2_weight = (1-fx)*(fy);
        z3_weight = (fx)*(fy);
        
        return z0_weight*z0 + z1_weight*z1 + z2_weight*z2 + z3_weight*z3;
      } else {
        return z_vec[pt_idx0];
      }
    }
  } // namespace interaction
} // namespace scrimmage
