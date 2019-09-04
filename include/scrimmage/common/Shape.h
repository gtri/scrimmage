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

ShapePtr make_shape(const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                    const double &opacity = 1.0);

ShapePtr make_arc(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                  const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                  const double &radius = 1.0,
                  const double &angle = M_PI / 2.0,
                  const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                  const double &opacity = 1.0);

ShapePtr make_arrow(const Eigen::Vector3d &pos = Eigen::Vector3d(0, 0, 0),
                    const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                    const double &length = 10.0,
                    const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                    const double &opacity = 1.0);

ShapePtr make_circle(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                     const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                     const double &radius = 1.0,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_cone(const Eigen::Vector3d &direction = Eigen::Vector3d(0, 0, 1),
                   const Eigen::Vector3d &apex = Eigen::Vector3d(0, 0, 1),
                   const double &height = 1.0,
                   const double &base_radius = 1.0,
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                   const double &opacity = 1.0);

ShapePtr make_cuboid(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                     const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                     const double &x_length = 1.0,
                     const double &y_length = 1.0,
                     const double &z_length = 1.0,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_ellipse(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                      const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                      const double &x_radius = 2.0,
                      const double &y_radius = 1.0,
                      const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                      const double &opacity = 1.0);

ShapePtr make_line(const Eigen::Vector3d &start = Eigen::Vector3d(0, 0, 0),
                   const Eigen::Vector3d &stop = Eigen::Vector3d(1, 0, 0),
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                   const double &opacity = 1.0);

ShapePtr make_mesh(const std::string &name = "volkswagen",
                   const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                   const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                   const double &scale = 1.0,
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                   const double &opacity = 1.0);

ShapePtr make_plane(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                    const scrimmage::Quaternion &quat = scrimmage::Quaternion(0, 0, 0),
                    const double &x_length = 10.0,
                    const double &y_length = 10.0,
                    const std::string &texture = "",
                    const bool &diffuse_lighting = false,
                    const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                    const double &opacity = 1.0);

ShapePtr make_pointcloud(const std::list<Eigen::Vector3d> &points,
                         const std::list<Eigen::Vector3d> &point_colors,
                         const double &size = 1.0,
                         const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                         const double &opacity = 1.0);

ShapePtr make_polydata(const std::list<Eigen::Vector3d> &points,
                       const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                       const double &opacity = 1.0);

ShapePtr make_polygon(const std::list<Eigen::Vector3d> &points,
                      const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                      const double &opacity = 1.0);

ShapePtr make_polyhedron(const std::list<Eigen::Vector3d> &points,
                         const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                         const double &opacity = 1.0);

ShapePtr make_polyline(const std::list<Eigen::Vector3d> &points,
                       const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                       const double &opacity = 1.0);

ShapePtr make_sphere(const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                     const double &radius = 1.0,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_spline(const std::list<Eigen::Vector3d> &points,
                     const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                     const double &opacity = 1.0);

ShapePtr make_text(const std::string &text,
                   const Eigen::Vector3d &center = Eigen::Vector3d(0, 0, 0),
                   const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                   const double &opacity = 1.0);

ShapePtr make_triangle(const Eigen::Vector3d &point0 = Eigen::Vector3d(1, 0, 0),
                       const Eigen::Vector3d &point1 = Eigen::Vector3d(0, 1, 0),
                       const Eigen::Vector3d &point2 = Eigen::Vector3d(-1, 0, 0),
                       const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 255),
                       const double &opacity = 1.0);

} // namespace shape
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_SHAPE_H_
