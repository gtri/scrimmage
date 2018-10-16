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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARYBASE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARYBASE_H_

#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <Eigen/Dense>

#include <memory>
#include <list>
#include <tuple>
#include <vector>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace interaction {

class BoundaryBase {
 public:
    BoundaryBase() : center_(0, 0, 0) {}
    virtual ~BoundaryBase() {}
    explicit BoundaryBase(const Eigen::Vector3d &center) : center_(center) {}
    virtual bool contains(Eigen::Vector3d p) = 0;
    const sc::ShapePtr shape() const { return shape_; }
    void set_shape(const sc::ShapePtr &shape) { shape_ = shape; }
    virtual void set_visual(int R, int G, int B, double opacity) = 0;
    virtual Eigen::Vector3d center() { return center_; }

    virtual const std::vector<std::tuple<double, double>> &extents() {
        return extents_;
    }

 protected:
    sc::ShapePtr shape_;
    std::list<sc::ShapePtr> shapes_;
    Eigen::Vector3d center_;
    std::vector<std::tuple<double, double>> extents_;
};

} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_BOUNDARYBASE_H_
