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

#include <gtest/gtest.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/proto/Shape.pb.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>

namespace sc = scrimmage;
using Eigen::Vector3d;

void shape_defaults(scrimmage::shape::ShapePtr shape) {
    EXPECT_EQ(shape->opacity(), 1.0);
    EXPECT_EQ(shape->color().r(), 0);
    EXPECT_EQ(shape->color().g(), 0);
    EXPECT_EQ(shape->color().b(), 255);
    EXPECT_EQ(shape->persistent(), true);
}

TEST(test_shape, defaults) {
    auto arc = sc::shape::make_arc();
    shape_defaults(arc);

    auto arrow = sc::shape::make_arrow();
    shape_defaults(arrow);

    auto circle = sc::shape::make_circle();
    shape_defaults(circle);

    auto cone = sc::shape::make_cone();
    shape_defaults(cone);

    auto cuboid = sc::shape::make_cuboid();
    shape_defaults(cuboid);

    auto ellipse = sc::shape::make_ellipse();
    shape_defaults(ellipse);

    auto line = sc::shape::make_line();
    shape_defaults(line);

    auto mesh = sc::shape::make_mesh();
    shape_defaults(mesh);

    auto plane = sc::shape::make_plane();
    shape_defaults(plane);

    auto pc = sc::shape::make_pointcloud(std::list<Eigen::Vector3d>{},
                                         std::list<Eigen::Vector3d>{});
    shape_defaults(pc);

    auto polydata = sc::shape::make_polydata(std::list<Eigen::Vector3d>{});
    shape_defaults(polydata);

    auto polygon = sc::shape::make_polygon(std::list<Eigen::Vector3d>{});
    shape_defaults(polygon);

    auto polyhedron = sc::shape::make_polyhedron(std::list<Eigen::Vector3d>{});
    shape_defaults(polyhedron);

    auto polyline = sc::shape::make_polyline(std::list<Eigen::Vector3d>{});
    shape_defaults(polyline);

    auto sphere = sc::shape::make_sphere();
    shape_defaults(sphere);

    auto spline = sc::shape::make_spline(std::list<Eigen::Vector3d>{});
    shape_defaults(spline);

    auto text = sc::shape::make_text("Hello");
    shape_defaults(text);

    auto triangle = sc::shape::make_triangle();
    shape_defaults(triangle);
}
