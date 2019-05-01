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

ShapePtr make_state(const scrimmage::State &state,
                    const Eigen::Vector3d &color,
                    const double &opacity) {
    Eigen::Vector3d tail = state.pos_const();
    Eigen::Vector3d head = tail + state.quat_const() * (Eigen::Vector3d::UnitX() * 10);

    auto shape = std::make_shared<scrimmage_proto::Shape>();
    shape->set_opacity(opacity);
    shape->set_persistent(true);
    sc::set(shape->mutable_color(), color);
    sc::set(shape->mutable_arrow()->mutable_tail(), tail);
    sc::set(shape->mutable_arrow()->mutable_head(), head);
    return shape;
}

ShapePtr make_circle(const Eigen::Vector3d &center,
                     const scrimmage::Quaternion &quat,
                     const double &radius,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = std::make_shared<sp::Shape>();
    shape->set_persistent(true);
    shape->set_opacity(opacity);
    sc::set(shape->mutable_color(), color);
    sc::set(shape->mutable_circle()->mutable_center(), center);
    sc::set(shape->mutable_circle()->mutable_quat(), quat);
    shape->mutable_circle()->set_radius(radius);
    return shape;
}

ShapePtr make_line(const Eigen::Vector3d &start,
                   const Eigen::Vector3d &stop,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto shape = std::make_shared<sp::Shape>();
    shape->set_persistent(true);
    shape->set_opacity(opacity);
    sc::set(shape->mutable_color(), color);
    sc::set(shape->mutable_line()->mutable_start(), start);
    sc::set(shape->mutable_line()->mutable_end(), stop);
    return shape;
}

ShapePtr make_text(const std::string &text,
                   const Eigen::Vector3d &center,
                   const Eigen::Vector3d &color,
                   const double &opacity) {
    auto text_shape = std::make_shared<sp::Shape>();
    text_shape->set_opacity(opacity);
    sc::set(text_shape->mutable_color(), color);
    text_shape->set_persistent(true);
    text_shape->mutable_text()->set_text(text);
    text_shape->mutable_text()->set_scale(1.0);
    sc::set(text_shape->mutable_text()->mutable_center(), center);
    return text_shape;
}

ShapePtr make_sphere(const Eigen::Vector3d &center,
                     const double &radius,
                     const Eigen::Vector3d &color,
                     const double &opacity) {
    auto shape = std::make_shared<sp::Shape>();
    shape->set_persistent(true);
    shape->set_opacity(opacity);
    shape->mutable_sphere()->set_radius(radius);
    sc::set(shape->mutable_sphere()->mutable_center(), center);
    sc::set(shape->mutable_color(), color);
    return shape;
}

ShapePtr make_pointcloud(const std::list<Eigen::Vector3d> &points,
                         const std::list<Eigen::Vector3d> &point_colors,
                         const double &size,
                         const Eigen::Vector3d &color,
                         const double &opacity) {
    auto pc = std::make_shared<scrimmage_proto::Shape>();
    pc->set_opacity(opacity);
    pc->set_persistent(true);
    sc::set(pc->mutable_color(), color);
    pc->mutable_pointcloud()->set_size(size);

    for (const Eigen::Vector3d &p : points) {
        auto *point = pc->mutable_pointcloud()->add_point();
        sc::set(point, p);
    }
    return pc;
}


} // namespace shape
} // namespace scrimmage
