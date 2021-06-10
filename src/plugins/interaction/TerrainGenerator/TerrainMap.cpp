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
                       const Technique &technique,
                       const Eigen::Vector3d &center,
                       const double &x_length, const double &y_length,
                       const double &x_resolution, const double &y_resolution,
                       const double &z_min, const double &z_max,
                       const Eigen::Vector3d &color) :
        rng_(rng), gener_(gener), technique_(technique), center_(center),
        x_length_(x_length),
        y_length_(y_length),
        x_resolution_(x_resolution),
        y_resolution_(y_resolution), z_min_(z_min), z_max_(z_max),
        color_(color),
        num_x_cols_(x_length_ / x_resolution_),
        num_y_rows_(y_length_ / y_resolution_),
        grid_(std::vector<std::vector<Node>>(num_y_rows_, std::vector<Node>(num_x_cols_))) {
    generate();
}

TerrainMap::TerrainMap(const scrimmage_msgs::Terrain &terrain) :
        rng_(nullptr), gener_(nullptr), center_(Eigen::Vector3d(0, 0, 0)),
        x_length_(terrain.x_length()), y_length_(terrain.y_length()),
        x_resolution_(terrain.x_resolution()),
        y_resolution_(terrain.y_resolution()), z_min_(terrain.z_min()),
        z_max_(terrain.z_max()), num_x_cols_(x_length_ / x_resolution_),
        num_y_rows_(y_length_ / y_resolution_),
        grid_(std::vector<std::vector<Node>>(num_y_rows_, std::vector<Node>(num_x_cols_))) {
    sc::set(center_, terrain.center());

    // Populate the grid
    for (int r = 0; r < terrain.map().row_size(); ++r) {
        for (int c = 0; c < terrain.map().row(r).col_size(); ++c) {
            grid_[r][c].height = terrain.map().row(r).col(c);
            grid_[r][c].is_set = true;
        }
    }
}

bool TerrainMap::generate() {
    if (gener_ == nullptr || rng_ == nullptr) {
        cout << "default_random_enginer is nullptr" << endl;
        return false;
    }

    if (technique_ == Technique::LINEAR) {
        return generate_linear();
    } else if (technique_ == Technique::LINEAR_WALK) {
        return generate_linear_walk();
    }
    return generate_random_walk();
}

bool TerrainMap::generate_linear() {
    // Force the altitude center to be at the midpoint between z_max and z_min
    center_(2) = (z_max_ + z_min_) / 2.0;

    // Initialize the map, such that row 0 is at z_min and the last row is at
    // z_max, with a linear interpolation across the rows. Also, add noise to
    // each height value.
    double z_step = (z_max_ - z_min_) / num_y_rows_;

    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        double height = z_step * row;
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            grid_[row][col].height = height + (*rng_)(*gener_);
            grid_[row][col].is_set = true;
        }
    }
    center_height_adjust();
    clamp_height();
    return true;
}

bool TerrainMap::generate_random_walk() {
    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            // If this node is already set, skip (continue)
            if (grid_[row][col].is_set) {
                continue;
            }
            // Get the average of the neighbors that are already set
            double height_avg = get_neighbor_avg(row, col);
            grid_[row][col].height = height_avg + (*rng_)(*gener_);
            grid_[row][col].is_set = true;
        }
    }
    center_height_adjust();
    clamp_height();
    return true;
}

bool TerrainMap::generate_linear_walk() {
    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            // If this node is already set, skip (continue)
            if (grid_[row][col].is_set) {
                continue;
            }
            // Get the average of the neighbors that are already set
            double height_avg = get_neighbor_avg(row, col);
            grid_[row][col].height = height_avg + (*rng_)(*gener_);
            grid_[row][col].is_set = true;
        }
    }
    center_height_adjust();
    clamp_height();
     // Force the altitude center to be at the midpoint between z_max and z_min
    center_(2) = (z_max_ + z_min_) / 2.0;

    // Initialize the map, such that row 0 is at z_min and the last row is at
    // z_max, with a linear interpolation across the rows. Also, add noise to
    // each height value.
    double z_step = (z_max_ - z_min_) / num_y_rows_;

    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        double height_l = z_step * row;
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            grid_[row][col].height = grid_[row][col].height + height_l;
        }
    }
    center_height_adjust();
    clamp_height();
    return true;
}

void TerrainMap::center_height_adjust() {
    // Make sure the grid's center is located at the appropriate height.
    double offset = center_(2) - grid_[std::round(num_y_rows_/2.0)][std::round(num_x_cols_/2.0)].height;
    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            grid_[row][col].height += offset;
        }
    }
}

void TerrainMap::clamp_height() {
    // Ensure all height values fall within z_min and z_max
    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            grid_[row][col].height = clamp(grid_[row][col].height, z_min_, z_max_);
        }
    }
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

scrimmage::ShapePtr TerrainMap::shape() {
    // Create the shape associated with this terrain
    sc::ShapePtr shape = std::make_shared<sp::Shape>();
    shape->set_persistent(true);
    sc::set(shape->mutable_color(), color_(0), color_(1), color_(2));
    shape->mutable_pointcloud()->set_size(3);

    for (unsigned int row = 0; row < num_y_rows_; ++row) {
        double y = center_(1)-y_length_ / 2.0 + row * y_resolution_;
        for (unsigned int col = 0; col < num_x_cols_; ++col) {
            sp::Vector3d *p = shape->mutable_pointcloud()->add_point();
            double x = center_(0)-x_length_ / 2.0 + col * x_resolution_;
            sc::set(p, x, y, grid_[row][col].height);
        }
    }
    return shape;
}

scrimmage_msgs::Terrain TerrainMap::proto() {
    scrimmage_msgs::Terrain terrain;

    sc::set(terrain.mutable_center(), center_);
    terrain.set_x_length(x_length_);
    terrain.set_y_length(y_length_);
    terrain.set_x_resolution(x_resolution_);
    terrain.set_y_resolution(y_resolution_);
    terrain.set_z_min(z_min_);
    terrain.set_z_max(z_max_);

    for (unsigned int r = 0; r < num_y_rows_; ++r) {
        scrimmage_msgs::Array1D *row = terrain.mutable_map()->add_row();
        for (unsigned int c = 0; c < num_x_cols_; ++c) {
            row->add_col(grid_[r][c].height);
        }
    }
    return terrain;
}

boost::optional<double> TerrainMap::height_at(const double &x, const double &y) {
    int row = std::floor((y + y_length_/2.0 - center_(1)) / y_resolution_);
    int col = std::floor((x + x_length_/2.0 - center_(0)) / x_resolution_);
    if (row < 0 || row >= static_cast<int>(num_y_rows_) ||
        col < 0 || col >= static_cast<int>(num_x_cols_) ||
        not grid_[row][col].is_set) {
        return boost::optional<double>{};
    }
    return grid_[row][col].height;
}

} // namespace interaction
} // namespace scrimmage
