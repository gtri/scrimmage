/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/TerrainGenerator/TerrainMap.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

using std::cout;
using std::endl;

using boost::algorithm::clamp;

namespace scrimmage {
namespace interaction {

TerrainMap::TerrainMap() {}

TerrainMap::TerrainMap(std::shared_ptr<std::normal_distribution<double>> rng,
                       std::shared_ptr<std::default_random_engine> gener,
                       const Eigen::Vector3d &center,
                       const double &x_length, const double &y_length,
                       const double &x_resolution, const double &y_resolution,
                       const double &z_min, const double &z_max,
                       const Eigen::Vector3d &color) :
        rng_(rng), gener_(gener), center_(center), x_length_(x_length),
        y_length_(y_length),
        x_resolution_(x_resolution),
        y_resolution_(y_resolution), z_min_(z_min), z_max_(z_max),
        color_(color),
        num_x_cols_(x_length_ / x_resolution_),
        num_y_rows_(y_length_ / y_resolution_),
        grid_(std::vector<std::vector<Node>>(num_y_rows_, std::vector<Node>(num_x_cols_))) {
    generate();
}

bool TerrainMap::generate() {
    if (gener_ == nullptr || rng_ == nullptr) {
        cout << "default_random_enginer is nullptr" << endl;
        return false;
    }

    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            // If this node is already set, skip (continue)
            if (grid_[row][col].is_set) {
                continue;
            }
            // Get the average of the neighbors that are already set
            double height_avg = get_neighbor_avg(row, col);
            grid_[row][col].height = clamp(height_avg + (*rng_)(*gener_), z_min_, z_max_);
            grid_[row][col].is_set = true;
        }
    }

    // Make sure the grid's center is located at the appropriate height.
    double offset = center_(2) -grid_[std::round(num_y_rows_/2.0)][std::round(num_x_cols_/2.0)].height;
    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            grid_[row][col].height += offset;
        }
    }

    // Create the shape associated with this terrain
    shape_ = std::make_shared<sp::Shape>();
    shape_->set_persistent(true);
    sc::set(shape_->mutable_color(), color_(0), color_(1), color_(2));
    shape_->mutable_pointcloud()->set_size(3);

    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        double y = center_(1)-y_length_ / 2.0 + row * y_resolution_;
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            sp::Vector3d *p = shape_->mutable_pointcloud()->add_point();
            double x = center_(0)-x_length_ / 2.0 + col * x_resolution_;
            sc::set(p, x, y, grid_[row][col].height);
        }
    }

    return true;
}

double TerrainMap::get_neighbor_avg(const int &row, const int &col) {
    unsigned int neighbors = 0;
    double height_sum = 0;

    if (row > 0) {
        height_sum += grid_[row-1][col].height;
        ++neighbors;
    }
    if (col > 0) {
        height_sum += grid_[row][col-1].height;
        ++neighbors;
    }
    if (row > 0 && col > 0) {
        height_sum += grid_[row-1][col-1].height;
        ++neighbors;
    }

    return (neighbors == 0) ? 0 : height_sum / neighbors;
}

} // namespace interaction
} // namespace scrimmage
