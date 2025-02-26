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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <gtest/gtest.h>
#include <scrimmage/common/FileSearch.h>

#include <boost/optional.hpp>

namespace sc = scrimmage;

TEST(test_find_files, find_files) {
    sc::FileSearch fs;
    std::unordered_map<std::string, std::list<std::string>> result;

    fs.find_files("SCRIMMAGE_DATA_PATH", ".js", result);
    EXPECT_TRUE(result.size() > 0);

    fs.find_files("SCRIMMAGE_DATA_PATH", "/", result);
    EXPECT_TRUE(result.size() > 0);
}

TEST(test_find_files, find_file) {
    sc::FileSearch fs;
    std::string result;
    EXPECT_TRUE(fs.find_file("app", "js", "SCRIMMAGE_DATA_PATH", result));
    EXPECT_TRUE(fs.find_file("straight", "xml", "SCRIMMAGE_MISSION_PATH", result));
    EXPECT_FALSE(fs.find_file("straight", "xml", "SCRIMMAGE_DATA_PATH", result));
}

TEST(test_find_files, find_dir) {
    sc::FileSearch fs;
    std::string result;
    EXPECT_TRUE(fs.find_dir("cesium_default", "SCRIMMAGE_DATA_PATH", result));
    std::cout << "cesium web dir: " << result  << std::endl;
}
