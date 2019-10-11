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
#include <scrimmage/parse/ParseUtils.h>

#include <vector>

#include <iostream>
using std::cout;
using std::endl;

namespace sc = scrimmage;

TEST(test_parse_utils, get_vec_of_vecs_spaces) {
    std::string test_str = "[a b c d]"
                           "[e f g h]"
                           "[h i j k]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, " "));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"a", "b", "c", "d"},
        {"e", "f", "g", "h"},
        {"h", "i", "j", "k"},
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}

TEST(test_parse_utils, get_vec_of_vecs_commas) {
    std::string test_str = "[a,b,c,d]"
                           "[e,f,g,h]"
                           "[h,i,j,k]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, ","));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"a", "b", "c", "d"},
        {"e", "f", "g", "h"},
        {"h", "i", "j", "k"}
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}

TEST(test_parse_utils, get_vec_of_vecs_spaces_and_commas) {
    std::string test_str = "[a,  b, c,  d]"
                           "[e,    f, g, h]"
                           "[h, i, j, k]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, " ,"));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"a", "b", "c", "d"},
        {"e", "f", "g", "h"},
        {"h", "i", "j", "k"}
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}

TEST(test_parse_utils, get_vec_of_vecs_empty_items) {
    std::string test_str = "[, b, , d]"
                           "[e, , g, ]"
                           "[h, i, j, ]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, " ,"));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"b", "d"},
        {"e", "g"},
        {"h", "i", "j"}
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}

TEST(test_parse_utils, get_vec_of_vecs_internal_vecs) {
    std::string test_str = "[a, [q, w], c, d]"
                           "[[ e, x ], f, g, h]"
                           "[h, i, j, [k, x, z]]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, " ,"));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"a", "[qw]", "c", "d"},
        {"[ex]", "f", "g", "h"},
        {"h", "i", "j", "[kxz]"}
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}

TEST(test_parse_utils, get_vec_of_vecs_internal_vecs_complicated) {
    std::string test_str =
            "[ MyPlanner print_planner_settings='true' "
            "show_path='true' "
            "goal='1000,1000,600,0,0,0' "
            "waypoint_list_topic='WaypointList'] "
            "[ MyController "
            "translation_bounds='[-1900,1900][-1900,1900][-2000,2000]]";

    std::vector<std::vector<std::string>> test_vecs;
    ASSERT_TRUE(sc::get_vec_of_vecs(test_str, test_vecs, " "));

    std::vector<std::vector<std::string>> expected_vecs = {
        {"MyPlanner", "print_planner_settings='true'", "show_path='true'", "goal='1000,1000,600,0,0,0'", "waypoint_list_topic='WaypointList'"},
        {"MyController", "translation_bounds='[-1900,1900][-1900,1900][-2000,2000]"},
    };
    EXPECT_EQ(expected_vecs, test_vecs);
}
