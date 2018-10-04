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

namespace sc = scrimmage;
namespace py = pybind11;
using namespace pybind11::literals; // NOLINT

void runner(bool x_discrete, bool ctrl_y, bool y_discrete, size_t num_actors,
            const std::string &actor_func,
            const std::map<int, double> &expected_rewards) {

    py::object test_open_ai_module = py::module::import("test_openai");
    py::object write_temp_mission = test_open_ai_module.attr("_write_temp_mission");

    write_temp_mission(
        "x_discrete"_a = x_discrete, "ctrl_y"_a = ctrl_y, "y_discrete"_a = y_discrete,
        "num_actors"_a = num_actors, "end"_a = 1000,
        "actor_func"_a = actor_func);

    const std::string mission = ".rlsimple";
    auto log_dir = sc::run_test(mission, false);

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
    Py_Initialize();
    runner(true, false, true, 1, "get_action_test_one_dim_discrete", {{1, 4}});
    runner(true, true, true, 1, "get_action_test_two_dim_discrete", {{1, 4}});
    runner(false, false, true, 1, "get_action_test_one_dim_continuous", {{1, 4}});
    runner(false, true, false, 1, "get_action_test_two_dim_continuous", {{1, 4}});
    runner(false, true, true, 1, "get_action_test_two_dim_tuple", {{1, 4}});

    // when there are multiple vehicles we need to specify an action
    // that is specific to an entity
    runner(true, false, true, 2, "get_action_test_one_dim_discrete", {{1, 4}, {2, 4}});
    Py_Finalize();
}
