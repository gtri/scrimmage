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

#ifndef INCLUDE_SCRIMMAGE_COMMON_SHAPE_H_
#define INCLUDE_SCRIMMAGE_COMMON_SHAPE_H_

#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Quaternion.h>

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <list>

namespace scrimmage {
namespace shape {

using ShapePtr = std::shared_ptr<scrimmage_proto::Shape>;

ShapePtr make_state(const scrimmage::State &state,
                    const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                    const double &opacity = 1.0);

ShapePtr make_circle(const Eigen::Vector3d &center,
                     const scrimmage::Quaternion &quat,
                     const double &radius,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_line(const Eigen::Vector3d &start,
                   const Eigen::Vector3d &stop,
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                   const double &opacity = 1.0);

ShapePtr make_text(const std::string &text,
                   const Eigen::Vector3d &center,
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 0),
                   const double &opacity = 1.0);

ShapePtr make_sphere(const Eigen::Vector3d &center,
                     const double &radius = 1.0,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_pointcloud(const std::list<Eigen::Vector3d> &points,
                         const std::list<Eigen::Vector3d> &point_colors,
                         const double &size = 1.0,
                         const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                         const double &opacity = 1.0);

} // namespace shape
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_SHAPE_H_
