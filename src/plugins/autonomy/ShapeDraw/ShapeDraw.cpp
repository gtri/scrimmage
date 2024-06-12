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

    circle_shape_ = std::make_shared<scrimmage_proto::Shape>();
    sphere_shape_ = std::make_shared<scrimmage_proto::Shape>();
    ellipse_shape_ = std::make_shared<scrimmage_proto::Shape>();
    cuboid_shape_ = std::make_shared<scrimmage_proto::Shape>();
    mesh_shape_ = std::make_shared<scrimmage_proto::Shape>();
    line_shape_ = std::make_shared<scrimmage_proto::Shape>();
    polyline_shape_ = std::make_shared<scrimmage_proto::Shape>();
}

bool ShapeDraw::step_autonomy(double t, double dt) {
    draw_circle(t, dt);
    draw_sphere(t, dt);
    draw_ellipse(t, dt);
    draw_cuboid(t, dt);
    draw_mesh(t, dt);
    draw_line(t, dt);
    draw_polyline(t, dt);

    return true;
}

void ShapeDraw::draw_ellipse(double t, double dt) {
    ellipse_shape_->set_opacity(1.0);
    if (t > 5) {
        std::cout << "Setting ellipse opacity to low to hide it!" << std::endl;
        ellipse_shape_->set_opacity(0.01);
    }

    ellipse_shape_->mutable_ellipse()->set_x_radius(10);
    ellipse_shape_->mutable_ellipse()->set_y_radius(5);

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0));
    sc::set(ellipse_shape_->mutable_ellipse()->mutable_quat(), quat);

    ellipse_shape_->set_persistent(true);
    sc::set(ellipse_shape_->mutable_ellipse()->mutable_center(), -20, -20, 0);
    sc::set(ellipse_shape_->mutable_color(), 0, 0, 255);
    draw_shape(ellipse_shape_);
}

void ShapeDraw::draw_cuboid(double t, double dt) {
    cuboid_shape_->set_opacity(1.0);
    sc::set(cuboid_shape_->mutable_cuboid()->mutable_center(), 20, 20, 0);
    cuboid_shape_->mutable_cuboid()->set_x_length(10);
    cuboid_shape_->mutable_cuboid()->set_y_length(5);
    cuboid_shape_->mutable_cuboid()->set_z_length(5);

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0),
                        sc::Angles::deg2rad(45.0));
    sc::set(cuboid_shape_->mutable_cuboid()->mutable_quat(), quat);

    cuboid_shape_->set_persistent(true);
    sc::set(cuboid_shape_->mutable_color(), 0, 255, 0);
    draw_shape(cuboid_shape_);
}

void ShapeDraw::draw_mesh(double t, double dt) {
    mesh_shape_->set_opacity(1.0);
    sc::set(mesh_shape_->mutable_mesh()->mutable_center(), 20, -20, 0);
    // corresponds to zephyr-red.xml
    mesh_shape_->mutable_mesh()->set_name("zephyr-red");

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0),
                        sc::Angles::deg2rad(90.0));
    sc::set(mesh_shape_->mutable_mesh()->mutable_quat(), quat);
    mesh_shape_->mutable_mesh()->set_scale(10);
    sc::set(mesh_shape_->mutable_color(), 255, 255, 255);

    mesh_shape_->set_persistent(true);

    draw_shape(mesh_shape_);
}

void ShapeDraw::draw_line(double t, double dt) {
    line_shape_->set_opacity(1.0);
    sc::set(line_shape_->mutable_line()->mutable_start(), 0, 0, 0);
    sc::set(line_shape_->mutable_line()->mutable_end(), 200, 0, 0);
    // corresponds to zephyr-red.xml
    line_shape_->mutable_line()->set_width(2);

    sc::set(line_shape_->mutable_color(), 0, 255, 255);
    line_shape_->set_persistent(true);
    draw_shape(line_shape_);
}

void ShapeDraw::draw_polyline(double t, double dt) {
    polyline_shape_->set_opacity(1.0);
    sc::set(polyline_shape_->mutable_color(), 255, 0, 255);
    polyline_shape_->set_persistent(true);

    double x = 0;
    double y = 0;
    double sgn = 1;
    polyline_shape_->mutable_polyline()->clear_point();
    for (int i=0; i<10; ++i) {
      auto line = polyline_shape_->mutable_polyline()->add_point();
      // zigzag
      sc::set(line, x, y, 20);
      y += 50;
      x += sgn * 30;
      sgn *= -1;
    }
    draw_shape(polyline_shape_);
}

void ShapeDraw::draw_circle(double t, double dt) {
    circle_shape_->set_opacity(1.0);
    if (t > 5) {
        std::cout << "Setting circle opacity to low to hide it!" << std::endl;
        circle_shape_->set_opacity(0.01);
    }

    circle_shape_->mutable_circle()->set_radius(10);

    sc::Quaternion quat(sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(0.0),
                        sc::Angles::deg2rad(45.0));
    sc::set(circle_shape_->mutable_circle()->mutable_quat(), quat);

    circle_shape_->set_persistent(true);
    sc::set(circle_shape_->mutable_circle()->mutable_center(), -20, -20, 0);
    sc::set(circle_shape_->mutable_color(), 0, 0, 255);
    draw_shape(circle_shape_);
}

void ShapeDraw::draw_sphere(double t, double dt) {
    sphere_shape_->set_opacity(1.0);
    if (t > 5) {
        std::cout << "Setting sphere opacity to low to hide it!" << std::endl;
        sphere_shape_->set_opacity(0.01);
    }

    sphere_shape_->mutable_sphere()->set_radius(20);

    sphere_shape_->set_persistent(true);
    sc::set(sphere_shape_->mutable_sphere()->mutable_center(), -20, -20, 0);
    sc::set(sphere_shape_->mutable_color(), 0, 0, 255);
    draw_shape(sphere_shape_);
}

} // namespace autonomy
} // namespace scrimmage
