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

#include <scrimmage/common/ID.h>

#include <sstream>

namespace sc = scrimmage;

TEST(test_id, cout) {
    sc::ID id(1, 2, 3);
    std::stringstream ss;
    ss << id;
    EXPECT_EQ(ss.str(), "1, 2, 3");
}

TEST(test_id, operator_equals) {
    sc::ID id1(1, 2, 3);
    sc::ID id2(1, 2, 3);
    EXPECT_EQ(id1, id2);

    bool lt = id1 < id2;
    EXPECT_FALSE(lt);

    id2.set_id(2);
    EXPECT_LT(id1, id2);

    id2.set_id(1);
    id2.set_sub_swarm_id(3);
    EXPECT_LT(id1, id2);

    id2.set_id(1);
    id2.set_sub_swarm_id(2);
    id2.set_team_id(4);
    EXPECT_LT(id1, id2);
}
