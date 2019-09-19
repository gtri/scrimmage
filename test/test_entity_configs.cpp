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

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#include <scrimmage/simcontrol/SimUtils.h>

#include <boost/optional.hpp>

namespace sc = scrimmage;

TEST(test_entity_configs, valid_entity_configs) {
    const std::string mission = "test_valid_entity_configs";
    auto log_dir = sc::run_test(mission, false, false);

    bool success = log_dir ? true : false;
    EXPECT_TRUE(success);
}

TEST(test_entity_configs, missing_autonomy) {
    const std::string mission = "test_missing_autonomy";
    auto log_dir = sc::run_test(mission, false, false);

    bool success = log_dir ? true : false;
    EXPECT_FALSE(success);
}

TEST(test_entity_configs, missing_controller) {
    const std::string mission = "test_missing_controller";
    auto log_dir = sc::run_test(mission, false, false);

    bool success = log_dir ? true : false;
    EXPECT_FALSE(success);
}

class PyTestEnv : public ::testing::Environment {
public:
    virtual ~PyTestEnv() {}

    virtual void SetUp() {
        Py_Initialize();
    }

    virtual void TearDown() {
        Py_Finalize();
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new PyTestEnv);
    return RUN_ALL_TESTS();
}
