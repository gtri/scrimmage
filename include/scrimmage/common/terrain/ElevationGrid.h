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
 * @date 19 Feb 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_COMMON_TERRAIN_ELEVATIONGRID_H_
#define INCLUDE_SCRIMMAGE_COMMON_TERRAIN_ELEVATIONGRID_H_

#include <Eigen/Dense>

#include <array>
#include <optional>
#include <vector>

namespace scrimmage {
namespace terrain {
class ElevationGrid {
 public:
  ElevationGrid(std::vector<double>&& x, std::vector<double>&& y, std::vector<double>&& z);

  // Determine elevation of the point that intersects the
  // ray originating form the vector pos, and pointed in
  // the direction specifed by the polar and azimuthal angles (relative to the elevation grid)
  std::optional<double> Raycast(double xpos,
                                double ypos,
                                double zpos,
                                double polar,
                                double azimuthal,
                                double max_distance,
                                bool interpolate = false) const;

  double Query(double xpos, double ypos, bool interpolate = false) const;

  void GetPoint(std::size_t index, std::array<double, 3>& buffer) const {
    buffer[0] = x_[index];
    buffer[1] = y_[index];
    buffer[2] = z_[index];
  }

  std::size_t number_points() const { return x_.size(); }

  std::size_t stride() const { return stride_; }

 protected:
  std::size_t stride_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> z_;

  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

  double Interpolate(double xpos, double ypos, std::size_t pt_idx0) const;

 private:
};
}  // namespace terrain
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_TERRAIN_ELEVATIONGRID_H_
