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

#include <scrimmage/common/Shape.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <vector>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace shape {

ShapePtr make_shape(const Eigen::Vector3d &color,
                    const double &opacity) {
    auto shape = std::make_shared<scrimmage_proto::Shape>();
    sc::set(shape->mutable_color(), color);
    shape->set_opacity(opacity);
    shape->set_persistent(true);
    shape->set_ttl(0);
    return shape;
}

ShapePtr make_arc(const Eigen::Vector3d &center,
                  const scrimmage::Quaternion &quat,
                  const double &radius,
                  const double &angle,
                  const Eigen::Vector3d &color,
                  const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_arc()->mutable_circle()->mutable_center(), center);
    sc::set(shape->mutable_arc()->mutable_circle()->mutable_quat(), quat);
    shape->mutable_arc()->mutable_circle()->set_radius(radius);
    shape->mutable_arc()->set_angle(angle);
    return shape;
}

ShapePtr make_arrow(const Eigen::Vector3d &pos,
                    const scrimmage::Quaternion &quat,
                    const double &length,
                    const Eigen::Vector3d &color,
                    const double &opacity) {
    auto shape = make_shape(color, opacity);
    Eigen::Vector3d head = pos + quat * (Eigen::Vector3d::UnitX() * length);
    sc::set(shape->mutable_arrow()->mutable_tail(), pos);
    sc::set(shape->mutable_arrow()->mutable_head(), head);
    return shape;
}

ShapePtr make_circle(const Eigen::Vector3d &center,
                     const scrimmage::Quaternion &quat,
                     const double &radius,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_circle()->mutable_center(), center);
    sc::set(shape->mutable_circle()->mutable_quat(), quat);
    shape->mutable_circle()->set_radius(radius);
    shape->mutable_circle()->set_solid(true);
    return shape;
}

ShapePtr make_cone(const Eigen::Vector3d &direction,
                   const Eigen::Vector3d &apex,
                   const double &height,
                   const double &base_radius,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_cone()->mutable_direction(), direction);
    sc::set(shape->mutable_cone()->mutable_apex(), apex);
    shape->mutable_cone()->set_height(height);
    shape->mutable_cone()->set_base_radius(base_radius);
    return shape;
}

ShapePtr make_cuboid(const Eigen::Vector3d &center,
                     const scrimmage::Quaternion &quat,
                     const double &x_length,
                     const double &y_length,
                     const double &z_length,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = make_shape(color, opacity);
    shape->mutable_cuboid()->set_x_length(x_length);
    shape->mutable_cuboid()->set_y_length(y_length);
    shape->mutable_cuboid()->set_z_length(z_length);
    sc::set(shape->mutable_cuboid()->mutable_center(), center);
    sc::set(shape->mutable_cuboid()->mutable_quat(), quat);
    return shape;
}

ShapePtr make_ellipse(const Eigen::Vector3d &center,
                      const scrimmage::Quaternion &quat,
                      const double &x_radius,
                      const double &y_radius,
                      const Eigen::Vector3d &color,
                      const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_ellipse()->mutable_center(), center);
    sc::set(shape->mutable_ellipse()->mutable_quat(), quat);
    shape->mutable_ellipse()->set_x_radius(x_radius);
    shape->mutable_ellipse()->set_y_radius(y_radius);
    return shape;
}

ShapePtr make_line(const Eigen::Vector3d &start,
                   const Eigen::Vector3d &stop,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_line()->mutable_start(), start);
    sc::set(shape->mutable_line()->mutable_end(), stop);
    return shape;
}

ShapePtr make_mesh(const std::string &name,
                   const Eigen::Vector3d &center,
                   const scrimmage::Quaternion &quat,
                   const double &scale,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto shape = make_shape(color, opacity);
    shape->mutable_mesh()->set_name(name);
    sc::set(shape->mutable_mesh()->mutable_center(), center);
    sc::set(shape->mutable_mesh()->mutable_quat(), quat);
    shape->mutable_mesh()->set_scale(scale);
    return shape;
}

ShapePtr make_plane(const Eigen::Vector3d &center,
                    const scrimmage::Quaternion &quat,
                    const double &x_length,
                    const double &y_length,
                    const std::string &texture,
                    const bool &diffuse_lighting,
                    const Eigen::Vector3d &color,
                    const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_plane()->mutable_center(), center);
    sc::set(shape->mutable_plane()->mutable_quat(), quat);
    shape->mutable_plane()->set_x_length(x_length);
    shape->mutable_plane()->set_y_length(y_length);
    shape->mutable_plane()->set_texture(texture);
    shape->mutable_plane()->set_diffuse_lighting(diffuse_lighting);
    return shape;
}

ShapePtr make_pointcloud(const std::list<Eigen::Vector3d> &points,
                         const std::list<Eigen::Vector3d> &point_colors,
                         const double &size,
                         const Eigen::Vector3d &color,
                         const double &opacity) {
    auto shape = make_shape(color, opacity);
    shape->mutable_pointcloud()->set_size(size);
    for (const Eigen::Vector3d &p : points) {
        auto *point = shape->mutable_pointcloud()->add_point();
        sc::set(point, p);
    }
    return shape;
}

ShapePtr make_polydata(const std::list<Eigen::Vector3d> &points,
                       const Eigen::Vector3d &color,
                       const double &opacity) {
    auto shape = make_shape(color, opacity);
    for (auto &point : points) {
        auto p = shape->mutable_polydata()->add_point();
        sc::set(p, point);
    }
    return shape;
}

ShapePtr make_polygon(const std::list<Eigen::Vector3d> &points,
                      const Eigen::Vector3d &color,
                      const double &opacity) {
    auto shape = make_shape(color, opacity);
    for (auto &point : points) {
        auto p = shape->mutable_polygon()->add_point();
        sc::set(p, point);
    }
    return shape;
}

ShapePtr make_polyhedron(const std::list<Eigen::Vector3d> &points,
                         const Eigen::Vector3d &color,
                         const double &opacity) {
    auto shape = make_shape(color, opacity);
    for (auto point : points) {
        auto p = shape->mutable_polyhedron()->add_point();
        sc::set(p, point);
    }
    return shape;
}

ShapePtr make_polyline(const std::list<Eigen::Vector3d> &points,
                       const Eigen::Vector3d &color,
                       const double &opacity) {
    auto shape = make_shape(color, opacity);
    for (auto &point : points) {
        auto p = shape->mutable_polyline()->add_point();
        sc::set(p, point);
    }
    return shape;
}

ShapePtr make_sphere(const Eigen::Vector3d &center,
                     const double &radius,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = make_shape(color, opacity);
    shape->mutable_sphere()->set_radius(radius);
    sc::set(shape->mutable_sphere()->mutable_center(), center);
    return shape;
}

ShapePtr make_spline(const std::list<Eigen::Vector3d> &points,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = make_shape(color, opacity);
    for (auto point : points) {
        auto p = shape->mutable_spline()->add_point();
        sc::set(p, point);
    }
    return shape;
}

ShapePtr make_text(const std::string &text,
                   const Eigen::Vector3d &center,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto shape = make_shape(color, opacity);
    shape->mutable_text()->set_text(text);
    shape->mutable_text()->set_scale(1.0);
    sc::set(shape->mutable_text()->mutable_center(), center);
    return shape;
}

ShapePtr make_triangle(const Eigen::Vector3d &point0,
                       const Eigen::Vector3d &point1,
                       const Eigen::Vector3d &point2,
                       const Eigen::Vector3d &color,
                       const double &opacity) {
    auto shape = make_shape(color, opacity);
    sc::set(shape->mutable_triangle()->mutable_point0(), point0);
    sc::set(shape->mutable_triangle()->mutable_point1(), point1);
    sc::set(shape->mutable_triangle()->mutable_point2(), point2);
    return shape;
}

} // namespace shape
} // namespace scrimmage
