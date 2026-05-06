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

#include <boost/optional.hpp>
#include <gtest/gtest.h>

#include <filesystem>
#include <vector>
#include <algorithm>

#include "scrimmage/common/CSV.h"
#include "scrimmage/entity/RuntimePluginOverrides.h"
#include "scrimmage/msgs/Event.pb.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/simcontrol/SimControl.h"
#include "scrimmage/simcontrol/SimUtils.h"

namespace sc = scrimmage;

namespace {

std::vector<std::filesystem::path> find_prefixed_csvs(
    const std::filesystem::path& dir,
    const std::string& prefix) {
    std::vector<std::filesystem::path> matches;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto& path = entry.path();
        if (path.extension() != ".csv") {
            continue;
        }

        const std::string filename = path.filename().string();
        if (filename.rfind(prefix, 0) == 0) {
            matches.push_back(path);
        }
    }

    std::sort(matches.begin(), matches.end());
    return matches;
}

void expect_api_tester_csv_row(
    const std::filesystem::path& file_path,
    int expected_int_value) {
    sc::CSV csv;
    ASSERT_TRUE(csv.read_csv(file_path.string()));
    ASSERT_EQ(csv.rows(), 1u);

    EXPECT_EQ(csv.at<std::string>(0, "my_test_bool"), "false");
    EXPECT_EQ(csv.at<int>(0, "my_test_int"), expected_int_value);
    EXPECT_DOUBLE_EQ(csv.at<double>(0, "my_test_float"), 1.0);
    EXPECT_DOUBLE_EQ(csv.at<double>(0, "my_test_double"), 100.0);
}

}  // namespace

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

TEST(test_entity_configs, generate_entity_runtime_plugin_overrides) {
    sc::SimControl simcontrol;
    ASSERT_TRUE(simcontrol.init("missions/test/test_generate_entity_runtime_overrides.xml", false));
    ASSERT_TRUE(simcontrol.start());

    auto& params = simcontrol.mp()->entity_descriptions().at(0);
    sc::RuntimePluginOverrides plugin_overrides;
    plugin_overrides["autonomy"][0]["my_test_int"] = "42";
    plugin_overrides["autonomy"][0]["csv_file_name"] = "runtime_override.csv";

    EXPECT_TRUE(simcontrol.generate_entity(0, params, plugin_overrides));

    const std::filesystem::path csv_path =
        std::filesystem::path(simcontrol.mp()->log_dir()) / "runtime_override.csv";
    ASSERT_TRUE(std::filesystem::exists(csv_path));

    expect_api_tester_csv_row(csv_path, 42);

    EXPECT_TRUE(simcontrol.shutdown(false));
}

TEST(test_entity_configs, parse_runtime_plugin_overrides_merges_duplicate_blocks_last_write_wins) {
    sc::SimControl simcontrol;
    ASSERT_TRUE(simcontrol.init("missions/test/test_generate_entity_runtime_overrides.xml", false));
    ASSERT_TRUE(simcontrol.start());

    scrimmage_msgs::GenerateEntity proto;

    auto* first_override = proto.add_plugin_override();
    first_override->set_plugin_type("autonomy");
    first_override->set_plugin_index(0);

    auto* csv_file_name = first_override->add_params();
    csv_file_name->set_key("csv_file_name");
    csv_file_name->set_value("runtime_override_merged.csv");

    auto* initial_value = first_override->add_params();
    initial_value->set_key("my_test_int");
    initial_value->set_value("10");

    auto* second_override = proto.add_plugin_override();
    second_override->set_plugin_type("autonomy");
    second_override->set_plugin_index(0);

    auto* replacement_value = second_override->add_params();
    replacement_value->set_key("my_test_int");
    replacement_value->set_value("42");

    sc::RuntimePluginOverrides runtime_plugin_overrides;
    ASSERT_TRUE(sc::parse_runtime_plugin_overrides(proto, simcontrol.mp(), 0, runtime_plugin_overrides));
    ASSERT_EQ(runtime_plugin_overrides["autonomy"][0]["csv_file_name"], "runtime_override_merged.csv");
    ASSERT_EQ(runtime_plugin_overrides["autonomy"][0]["my_test_int"], "42");

    auto& params = simcontrol.mp()->entity_descriptions().at(0);
    EXPECT_TRUE(simcontrol.generate_entity(0, params, runtime_plugin_overrides));

    const std::filesystem::path csv_path =
        std::filesystem::path(simcontrol.mp()->log_dir()) / "runtime_override_merged.csv";
    ASSERT_TRUE(std::filesystem::exists(csv_path));

    expect_api_tester_csv_row(csv_path, 42);

    EXPECT_TRUE(simcontrol.shutdown(false));
}

TEST(test_entity_configs, generate_entity_runtime_plugin_override_rejects_missing_plugin_index) {
    sc::SimControl simcontrol;
    ASSERT_TRUE(simcontrol.init("missions/test/test_generate_entity_runtime_overrides.xml", false));
    ASSERT_TRUE(simcontrol.start());

    auto& params = simcontrol.mp()->entity_descriptions().at(0);
    sc::RuntimePluginOverrides plugin_overrides;
    plugin_overrides["autonomy"][1]["my_test_int"] = "42";

    EXPECT_FALSE(simcontrol.generate_entity(0, params, plugin_overrides));

    EXPECT_TRUE(simcontrol.shutdown(false));
}

TEST(test_entity_configs, generate_entity_runtime_plugin_override_rejects_unknown_param_name) {
    sc::SimControl simcontrol;
    ASSERT_TRUE(simcontrol.init("missions/test/test_generate_entity_runtime_overrides.xml", false));
    ASSERT_TRUE(simcontrol.start());

    auto& params = simcontrol.mp()->entity_descriptions().at(0);
    sc::RuntimePluginOverrides plugin_overrides;
    plugin_overrides["autonomy"][0]["definitely_not_a_real_param"] = "42";

    EXPECT_FALSE(simcontrol.generate_entity(0, params, plugin_overrides));

    EXPECT_TRUE(simcontrol.shutdown(false));
}

TEST(test_entity_configs, generate_entity_runtime_plugin_override_allows_unknown_param_name_when_disabled) {
    sc::SimControl simcontrol;
    ASSERT_TRUE(simcontrol.init("missions/test/test_generate_entity_runtime_overrides.xml", false));
    ASSERT_TRUE(simcontrol.start());
    simcontrol.mp()->params()["strict_runtime_plugin_params"] = "false";

    auto& params = simcontrol.mp()->entity_descriptions().at(0);
    sc::RuntimePluginOverrides plugin_overrides;
    plugin_overrides["autonomy"][0]["definitely_not_a_real_param"] = "42";
    plugin_overrides["autonomy"][0]["csv_file_name"] = "runtime_override_unknown_allowed.csv";

    EXPECT_TRUE(simcontrol.generate_entity(0, params, plugin_overrides));

    const std::filesystem::path csv_path =
        std::filesystem::path(simcontrol.mp()->log_dir()) / "runtime_override_unknown_allowed.csv";
    ASSERT_TRUE(std::filesystem::exists(csv_path));

    EXPECT_TRUE(simcontrol.shutdown(false));
}

TEST(test_entity_configs, generate_entity_runtime_plugin_overrides_stress_mission) {
    auto log_dir = sc::run_test("missions/test/test_generate_entity_runtime_override_stress.xml", false, false);
    ASSERT_TRUE(log_dir);

    const std::filesystem::path log_dir_path(*log_dir);
    const auto csv_files = find_prefixed_csvs(log_dir_path, "runtime_override_stress_");
    ASSERT_EQ(csv_files.size(), 120);

    const std::filesystem::path first_csv = log_dir_path / "runtime_override_stress_000.csv";
    const std::filesystem::path last_csv = log_dir_path / "runtime_override_stress_119.csv";
    ASSERT_TRUE(std::filesystem::exists(first_csv));
    ASSERT_TRUE(std::filesystem::exists(last_csv));

    expect_api_tester_csv_row(first_csv, 1000);
    expect_api_tester_csv_row(last_csv, 1119);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
