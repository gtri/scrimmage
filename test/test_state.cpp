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
#include <scrimmage/math/State.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>

namespace sc = scrimmage;
using Eigen::Vector3d;

TEST(test_state, local_frames) {

    sc::State state1, state2;
    state1.pos() = Vector3d(1, 0, 0);
    state1.quat().set(M_PI, 0, 0);

    state2.pos() = Vector3d(1, 0, 1);
    state2.quat().set(0, 0, 0);
    Vector3d position_local_to_state1 = \
        state1.rel_pos_local_frame(state2.pos());

    EXPECT_NEAR(0, position_local_to_state1(0), 1e-10);
    EXPECT_NEAR(0, position_local_to_state1(1), 1e-10);
    EXPECT_NEAR(-1, position_local_to_state1(2), 1e-10);

    Vector3d position_local_to_state2 = \
        state2.rel_pos_local_frame(state1.pos());

    EXPECT_NEAR(0, position_local_to_state2(0), 1e-10);
    EXPECT_NEAR(0, position_local_to_state2(1), 1e-10);
    EXPECT_NEAR(-1, position_local_to_state2(2), 1e-10);
}

TEST(test_state, rel_az) {
    sc::State state;
    state.pos() << 0, 0, 0;
    state.vel() << 0, 0, 0;
    state.quat().set(M_PI / 4, M_PI / 4, M_PI / 4);

    Eigen::Vector3d vec1(1, 0, 50);
    double rel_az1 = state.rel_az(vec1);

    Eigen::Vector3d vec2(0, 1, 50);
    double rel_az2 = state.rel_az(vec2);

    EXPECT_NEAR(-M_PI / 4, rel_az1, 1e-10);
    EXPECT_NEAR(M_PI / 4, rel_az2, 1e-10);
}
