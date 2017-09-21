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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Keys.h>
#include <scrimmage/common/Values.h>

#include <vector>
#include <map>
#include <unordered_set>
#include <set>

using Eigen::Vector3d;
namespace sc = scrimmage;

TEST(test_utilities, linspace) {
    std::vector<double> vec = sc::linspace(0, 1, 3);
    EXPECT_EQ(vec.size(), static_cast<size_t>(3));
    EXPECT_NEAR(vec[0], 0, 1e-8);
    EXPECT_NEAR(vec[1], 0.5, 1e-8);
    EXPECT_NEAR(vec[2], 1, 1e-8);

    vec = sc::linspace(M_PI, 0, 5);
    EXPECT_EQ(vec.size(), static_cast<size_t>(5));
    EXPECT_NEAR(vec[0], M_PI, 1e-8);
    EXPECT_NEAR(vec[1], 3 * M_PI / 4, 1e-8);
    EXPECT_NEAR(vec[2], M_PI / 2, 1e-8);
    EXPECT_NEAR(vec[3], M_PI / 4, 1e-8);
    EXPECT_NEAR(vec[4], 0, 1e-8);
}

TEST(test_utilities, keys_class) {
    std::map<int, int> map;
    for (int i = 1; i < 10; i++) {
        map[i] = -i;
    }

    // testing the create new memory version
    auto keys_as_set = sc::keys(map).as<std::set<int>>();
    auto values_as_set = sc::values(map).as<std::set<int>>();

    EXPECT_EQ(keys_as_set.size(), static_cast<size_t>(9));
    EXPECT_EQ(values_as_set.size(), static_cast<size_t>(9));

    for (int i = 1; i < 10; i++) {
        EXPECT_EQ(keys_as_set.count(i), static_cast<size_t>(1));
        EXPECT_EQ(values_as_set.count(i), static_cast<size_t>(0));
    }

    for (int i = -9; i < 0; i++) {
        EXPECT_EQ(keys_as_set.count(i), static_cast<size_t>(0));
        EXPECT_EQ(values_as_set.count(i), static_cast<size_t>(1));
    }

    // testing the 0 memory cost version
    for (int key : sc::keys(map)) {
        EXPECT_EQ(keys_as_set.count(key), static_cast<size_t>(1));
        EXPECT_EQ(values_as_set.count(key), static_cast<size_t>(0));
    }

    for (int value : sc::values(map)) {
        EXPECT_EQ(keys_as_set.count(value), static_cast<size_t>(0));
        EXPECT_EQ(values_as_set.count(value), static_cast<size_t>(1));
    }
}
