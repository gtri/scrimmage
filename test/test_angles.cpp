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
#include <gmock/gmock.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::Vector3d;
namespace sc = scrimmage;

TEST(test_quaternion, rotation) {
    sc::Angles ang;
    ang.set_input_clock_direction(sc::Angles::Rotate::CW);
    ang.set_input_zero_axis(sc::Angles::HeadingZero::Pos_Y);
    ang.set_output_clock_direction(sc::Angles::Rotate::CCW);
    ang.set_output_zero_axis(sc::Angles::HeadingZero::Pos_X);

    ang.set_angle(134);
    EXPECT_NEAR(ang.angle(), 360-44, 1e-1);
}
