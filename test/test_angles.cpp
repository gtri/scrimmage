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
#include <scrimmage/math/Angles.h>

namespace sc = scrimmage;
using ang = sc::Angles;

TEST(test_angles, rotation) {
    sc::Angles ang;
    ang.set_input_clock_direction(ang::Rotate::CW);
    ang.set_input_zero_axis(ang::HeadingZero::Pos_Y);
    ang.set_output_clock_direction(ang::Rotate::CCW);
    ang.set_output_zero_axis(ang::HeadingZero::Pos_X);

    ang.set_angle(134);
    EXPECT_NEAR(ang.angle(), 360-44, 1e-1);
}

TEST(test_angles, angle_within) {
    for (int i = -360 * 2; i < 360 * 2; i++) {
        for (int j = 1; j < 89; j++) {
            int low = i;
            int high = i+ 90;
            int ang = i + j;

            EXPECT_TRUE(ang::angle_within(low, high, ang));
            EXPECT_TRUE(ang::angle_within(high, low, ang));

            EXPECT_FALSE(ang::angle_within(high + 90, high + 180, ang));
            EXPECT_FALSE(ang::angle_within(high + 180, high + 90, ang));

            double low_rad = ang::deg2rad(low);
            double high_rad = ang::deg2rad(high);
            double ang_rad = ang::deg2rad(ang);

            EXPECT_TRUE(ang::angle_within(low_rad, high_rad, ang_rad));
            EXPECT_TRUE(ang::angle_within(high_rad, low_rad, ang_rad));

            EXPECT_FALSE(ang::angle_within(high_rad + M_PI / 2, high_rad + M_PI, ang_rad));
            EXPECT_FALSE(ang::angle_within(high_rad + M_PI, high_rad + M_PI / 2, ang_rad));
        }
    }
}
