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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_SPHERE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_SPHERE_H_

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>

using std::cout;
using std::endl;

namespace scrimmage {
namespace interaction {

class Sphere : public BoundaryBase {
 public:
    Sphere() : radius_(1.0) {
    }

    void set_radius(const double &radius) { radius_ = radius; }
    void set_center(const Eigen::Vector3d &center) { center_ = center; }

    double radius() { return radius_; }
    Eigen::Vector3d center() override { return center_; }

    bool contains(Eigen::Vector3d p) override {
        return (p-center_).norm() < radius_;
    }

    void set_visual(int R, int G, int B, double opacity) override {
        sc::ShapePtr shape(new sp::Shape);
        shape->set_type(sp::Shape::Sphere);
        shape->set_opacity(opacity);
        sc::set(shape->mutable_color(), R, G, B);
        shape->set_persistent(true);
        sc::set(shape->mutable_center(), center_);
        shape->set_radius(radius_);
        shapes_.push_back(shape);
    }

 protected:
    double radius_;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_SPHERE_H_
