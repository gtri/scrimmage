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

#include <scrimmage/common/Algorithm.h>

#include <map>

namespace sc = scrimmage;

TEST(test_algorithm, remove_if) {
    std::map<int, int> map;
    for (int i = 0; i < 10; i++) {
        map[i] = i - 5;
    }
    sc::remove_if(map, [&](auto &kv) {return kv.second < 0;});
    EXPECT_EQ(static_cast<int>(map.size()), 5);

    for (int i = 5; i < 10; i++) {
        EXPECT_EQ(map[i], i - 5);
    }
}

TEST(test_algorithm, set_difference) {
    std::unordered_set<int> set1 {1, 2, 3, 4, 5};
    std::unordered_set<int> set2 {1, 3, 5};

    std::unordered_set<int> set_diff = sc::set_difference(set1, set2);
    EXPECT_EQ(set_diff.count(1), static_cast<uint8_t>(0));
    EXPECT_EQ(set_diff.count(2), static_cast<uint8_t>(1));
    EXPECT_EQ(set_diff.count(3), static_cast<uint8_t>(0));
    EXPECT_EQ(set_diff.count(4), static_cast<uint8_t>(1));
    EXPECT_EQ(set_diff.count(5), static_cast<uint8_t>(0));
}

TEST(test_algorithm, set_union) {
    std::unordered_set<int> set1 {1, 2, 3, 4, 5};
    std::unordered_set<int> set2 {1, 3, 5, 6};

    std::unordered_set<int> out = sc::set_union(set1, set2);
    EXPECT_EQ(out.count(1), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(2), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(3), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(4), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(5), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(6), static_cast<uint8_t>(1));
}

TEST(test_algorithm, set_intersection) {
    std::unordered_set<int> set1 {1, 2, 3, 4, 5};
    std::unordered_set<int> set2 {1, 3, 5, 6};

    std::unordered_set<int> out = sc::set_intersection(set1, set2);
    EXPECT_EQ(out.count(1), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(2), static_cast<uint8_t>(0));
    EXPECT_EQ(out.count(3), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(4), static_cast<uint8_t>(0));
    EXPECT_EQ(out.count(5), static_cast<uint8_t>(1));
    EXPECT_EQ(out.count(6), static_cast<uint8_t>(0));
}
