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

#include <scrimmage/common/CSV.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <boost/optional.hpp>
#include <chrono>
#include <thread>

namespace sc = scrimmage;
namespace py = pybind11;
using namespace pybind11::literals; // NOLINT

void runner(bool x_discrete, bool ctrl_y, bool y_discrete, bool grpc_mode,
            size_t num_actors, const std::map<int, double> &expected_rewards) {

    py::object test_open_ai_module = py::module::import("test_openai");
    py::object write_temp_mission = test_open_ai_module.attr("_write_temp_mission");

    write_temp_mission(
        "x_discrete"_a = x_discrete, "ctrl_y"_a = ctrl_y, "y_discrete"_a = y_discrete,
        "num_actors"_a = num_actors, "grpc_mode"_a = grpc_mode, "end"_a = 1000);

    const std::string mission = ".rlsimple";
    auto log_dir = sc::run_test(mission, false, false);

    bool success = log_dir ? true : false;
    EXPECT_TRUE(success);
    if (!log_dir) return;

    sc::CSV csv;
    bool rewards_found = csv.read_csv(*log_dir + "/rewards.csv");
    EXPECT_TRUE(rewards_found);
    if (!rewards_found) return;

    for (size_t row = 0; row < csv.rows(); row++) {
        int id = csv.at(row, "id");
        double actual_reward = csv.at(row, "reward");
        EXPECT_EQ(actual_reward, expected_rewards.at(id));
    }
}

TEST(TestOpenAI, one_dim_discrete) {
    runner(true, false, true, false, 1, {{1, 4}});
}

TEST(TestOpenAI, two_dim_discrete) {
    runner(true, true, true, false, 1, {{1, 4}});
}

TEST(TestOpenAI, one_dim_continuous) {
    runner(false, false, true, false, 1, {{1, 4}});
}

TEST(TestOpenAI, two_dim_continuous) {
    runner(false, true, false, false, 1, {{1, 4}});
}

TEST(TestOpenAI, two_dim_tuple) {
    runner(false, true, true, false, 1, {{1, 4}});
}

TEST(TestOpenAI, combined_one_dim_discrete) {
    // when there are multiple vehicles we need to specify an action
    // that is specific to an entity
    runner(true, false, true, false, 2, {{1, 4}, {2, 4}});
}

// Setting up testing environment according to gtest documentation
// https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
class TestOpenAIEnvironment : public ::testing::Environment {
public:
    virtual ~TestOpenAIEnvironment() {}

    virtual void SetUp() {
        Py_Initialize();
    }

    virtual void TearDown() {
        Py_Finalize();
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new TestOpenAIEnvironment);
    return RUN_ALL_TESTS();
}
