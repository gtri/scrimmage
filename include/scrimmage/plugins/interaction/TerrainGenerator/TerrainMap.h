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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINGENERATOR_TERRAINMAP_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINGENERATOR_TERRAINMAP_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/msgs/Terrain.pb.h>

#include <vector>
#include <string>
#include <limits>
#include <random>
#include <memory>

#include <boost/optional.hpp>

namespace scrimmage {
namespace interaction {

class TerrainMap {
 public:
    enum class Technique { RANDOM_WALK, LINEAR, LINEAR_WALK };

    TerrainMap();
    TerrainMap(std::shared_ptr<std::normal_distribution<double>> rng,
               std::shared_ptr<std::default_random_engine> gener,
               const Technique &technique,
               const Eigen::Vector3d &center,
               const double &x_length, const double &y_length,
               const double &x_resolution, const double &y_resolution,
               const double &z_min, const double &z_max,
               const Eigen::Vector3d &color);
    explicit TerrainMap(const scrimmage_msgs::Terrain &terrain);
    scrimmage::ShapePtr shape();
    scrimmage_msgs::Terrain proto();
    boost::optional<double> height_at(const double &x, const double &y);

 protected:
    bool generate();
    bool generate_random_walk();
    bool generate_linear();
    bool generate_linear_walk();
    void center_height_adjust();
    void clamp_height();

    std::shared_ptr<std::normal_distribution<double>> rng_;
    std::shared_ptr<std::default_random_engine> gener_;
    Technique technique_ = Technique::RANDOM_WALK;

    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double x_length_ = 10;
    double y_length_ = 10;
    double x_resolution_ = 1.0;
    double y_resolution_ = 1.0;
    double z_min_ = -std::numeric_limits<double>::infinity();
    double z_max_ = +std::numeric_limits<double>::infinity();
    Eigen::Vector3d color_ = Eigen::Vector3d::Zero();

    unsigned int num_x_cols_ = x_length_ / x_resolution_;
    unsigned int num_y_rows_ = y_length_ / y_resolution_;

    class Node {
     public:
        double height = 0;
        bool is_set = false;
    };

    std::vector<std::vector<Node>> grid_;
    double get_neighbor_avg(const int &row, const int &col);

 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_TERRAINGENERATOR_TERRAINMAP_H_
