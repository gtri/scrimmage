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

#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <fstream>

#include <boost/optional.hpp>

namespace sc = scrimmage;

TEST(test_angles, rotation) {
    const std::string mission = "straight";
    auto log_dir = sc::run_test(mission);

    bool success = log_dir ? true : false;
    EXPECT_TRUE(success);
    if (!log_dir) return;

    std::cout << *log_dir << std::endl;
    std::ifstream summary_file(*log_dir + "/summary.csv");
    EXPECT_TRUE(summary_file.is_open());
    if (!summary_file.is_open()) return;

    std::list<std::string> lines;
    std::string str;
    while (std::getline(summary_file, str)) {
        lines.push_back(str);
    }

    EXPECT_FALSE(lines.empty());
    if (lines.empty()) return;

    auto vec = sc::str2vec<double>(lines.back(), ",");
    EXPECT_EQ(static_cast<int>(vec.size()), 8);
    if (vec.size() != 8) return;

    EXPECT_GT(vec.rbegin()[1], 0); // expect collisions
}
