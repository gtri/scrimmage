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

#include <scrimmage/plugins/autonomy/ShapeDraw/ShapeDraw.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::ShapeDraw,
                ShapeDraw_plugin)

namespace scrimmage {
namespace autonomy {

ShapeDraw::ShapeDraw() : follow_id_(-1), init_(true) {
}

void ShapeDraw::init(std::map<std::string, std::string> &params) {
    double initial_speed = sc::get<double>("initial_speed", params, 21);

    desired_state_->vel() = Eigen::Vector3d::UnitX()*initial_speed;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);
}

bool ShapeDraw::step_autonomy(double t, double dt) {
    draw_ellipse(t, dt);

    // cuboid adds current rotation to its transformed position,
    // so repeatedly specifying a cuboid at the same location will
    // cause it to continuously rotate, which we don't want
    if (init_) {
        draw_cuboid(t, dt);
        init_ = false;
    }

    return true;
}

void ShapeDraw::draw_ellipse(double t, double dt) {
    auto shape = std::make_shared<scrimmage_proto::Shape>();
    shape->set_hash(84276524578);
    shape->set_hash_set(true);
    shape->set_opacity(1.0);
    shape->mutable_ellipse()->set_x_radius(10);
    shape->mutable_ellipse()->set_y_radius(5);

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0));
    sc::set(shape->mutable_ellipse()->mutable_quat(), quat);

    shape->set_persistent(true);
    shape->set_ttl(60);
    sc::set(shape->mutable_ellipse()->mutable_center(), -20, -20, 0);
    sc::set(shape->mutable_color(), 0, 0, 255);
    draw_shape(shape);
}

void ShapeDraw::draw_cuboid(double t, double dt) {
    auto shape = std::make_shared<scrimmage_proto::Shape>();
    shape->set_hash(64927777123);
    shape->set_hash_set(true);
    shape->set_opacity(1.0);
    sc::set(shape->mutable_cuboid()->mutable_center(), 20, 20, 0);
    shape->mutable_cuboid()->set_x_length(10);
    shape->mutable_cuboid()->set_y_length(5);
    shape->mutable_cuboid()->set_z_length(5);

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0));
    sc::set(shape->mutable_cuboid()->mutable_quat(), quat);

    shape->set_persistent(true);
    shape->set_ttl(60);
    sc::set(shape->mutable_color(), 0, 255, 0);
    draw_shape(shape);
}

} // namespace autonomy
} // namespace scrimmage
