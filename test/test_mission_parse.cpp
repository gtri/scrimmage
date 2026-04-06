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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 29 Feb 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <gmock/gmock.h>

#include "scrimmage/parse/MissionParse.h"

namespace {

const std::map<std::string, scrimmage::EntityPluginInfo>& get_entity_plugins(
    const scrimmage::MissionParse& mission_parse, int entity_block_id) {
    static const std::map<std::string, scrimmage::EntityPluginInfo> empty_plugins;
    const auto& all_plugins = mission_parse.all_entity_plugins();
    auto plugin_it = all_plugins.find(entity_block_id);
    EXPECT_NE(plugin_it, all_plugins.end());
    if (plugin_it == all_plugins.end()) {
        return empty_plugins;
    }
    return plugin_it->second;
}

}  // namespace

class MissionParseTest : public testing::TestWithParam<std::string> {
 protected:
    void SetUp() override {
        std::string filename = GetParam();
        mp.parse(filename);
    }
    scrimmage::MissionParse mp;
};

// Values should match the ones in test_mission_parse_mission.xml
TEST_P(MissionParseTest, mission_parse_test_runtag_info) {
    // Test that the runtag info matches
    EXPECT_DOUBLE_EQ(mp.t0(), 0);
    EXPECT_DOUBLE_EQ(mp.tend(), 100);
    EXPECT_DOUBLE_EQ(mp.dt(), 0.1);
    EXPECT_DOUBLE_EQ(mp.time_warp(), 10);
    EXPECT_DOUBLE_EQ(mp.motion_multiplier(), 1);
    EXPECT_EQ(mp.enable_gui(), true);
    EXPECT_EQ(mp.network_gui(), false);
    EXPECT_EQ(mp.start_paused(), true);
    EXPECT_EQ(mp.full_screen(), true);
    EXPECT_EQ(mp.window_width(), 800);
    EXPECT_EQ(mp.window_height(), 600);
}
void compare_attributes(
    scrimmage::AttributeMap& attributes,
    std::string node_name,
    std::list<std::string> attribute_names,
    std::list<std::string> expected_attribute_values) {

    ASSERT_EQ(attribute_names.size(), expected_attribute_values.size());
    auto name_it = attribute_names.begin();
    auto values_it = expected_attribute_values.begin();
    for (; name_it != attribute_names.cend() && values_it != expected_attribute_values.cend();
         ++name_it, ++values_it) {
        EXPECT_STREQ(attributes[node_name][*name_it].c_str(), values_it->c_str());
    }
}

TEST_P(MissionParseTest, mission_parse_test_entity_interaction) {
    std::list<std::string> eis = mp.entity_interactions();
    ASSERT_EQ(eis.size(), 3);
    auto ei_it = eis.begin();

    ASSERT_STREQ(ei_it->c_str(), "Boundary");
    compare_attributes(
        mp.attributes(),
        "Boundary",
        {"type", "lengths", "center", "rpy"},
        {"cuboid", "2000, 2000, 1000", "0, 0, 500", "0, 0, 0"});

    ++ei_it;
    ASSERT_STREQ(ei_it->c_str(), "SimpleCollision");

    ++ei_it;
    ASSERT_STREQ(ei_it->c_str(), "GroundCollision");
}

TEST_P(MissionParseTest, mission_parse_test_terrain) {
    auto terrain = mp.utm_terrain();

    EXPECT_DOUBLE_EQ(terrain->grid_spacing(), 10);
    EXPECT_DOUBLE_EQ(terrain->grid_size(), 1000);
    EXPECT_DOUBLE_EQ(terrain->origin_length(), 10);
    EXPECT_STREQ(terrain->terrain_name().c_str(), "mcmillan");
    EXPECT_TRUE(terrain->show_origin());
    EXPECT_EQ(terrain->zone(), 10);
    EXPECT_STREQ(terrain->hemisphere().c_str(), "north");
}

TEST_P(MissionParseTest, mission_parse_test_unique_tags) {
    // Test that unique tags are set coorrectly. These are
    // tags that are not repeated in mission files

    EXPECT_DOUBLE_EQ(mp.latitude_origin(), 35.721025);
    EXPECT_DOUBLE_EQ(mp.longitude_origin(), -120.767925);
    EXPECT_DOUBLE_EQ(mp.altitude_origin(), 300);
    EXPECT_STREQ(mp.params()["stream_port"].c_str(), "50051");
    EXPECT_STREQ(mp.params()["stream_ip"].c_str(), "localhost");
}

TEST_P(MissionParseTest, mission_parse_test_param_common) {
    compare_attributes(mp.attributes(), "LocalNetwork", {"csv_filename"}, {"bar.csv"});

    auto &network_names = mp.network_names();
    EXPECT_NE(std::find(network_names.begin(), network_names.end(), "CommsNetwork"),
              network_names.end());
    EXPECT_EQ(mp.attributes()["CommsNetwork"]["ORIGINAL_PLUGIN_NAME"], "SphereNetwork");

    // Plugin params are now in all_entity_plugins()
    const auto& plugins = get_entity_plugins(mp, 0);
    auto it = plugins.find("autonomy:Straight");
    ASSERT_NE(it, plugins.end());
    EXPECT_EQ(it->second.params.at("speed"), "20");
}

TEST_P(MissionParseTest, mission_parse_test_gen_info) {
    auto gen_info = mp.gen_info();
    ASSERT_EQ(gen_info.size(), 4);

    auto compare_gen_info = [](scrimmage::GenerateInfo actual, scrimmage::GenerateInfo expected) {
        EXPECT_EQ(actual.total_count, expected.total_count);
        EXPECT_EQ(actual.gen_count, expected.gen_count);
        EXPECT_EQ(actual.first_in_group, expected.first_in_group);
        EXPECT_DOUBLE_EQ(actual.start_time, expected.start_time);
        EXPECT_DOUBLE_EQ(actual.rate, expected.rate);
        EXPECT_DOUBLE_EQ(actual.time_variance, expected.time_variance);
    };

    auto gen_it = gen_info.begin();
    scrimmage::GenerateInfo expected_gi;
    ASSERT_EQ(gen_it->first, 0);

    expected_gi.total_count = 1;
    expected_gi.gen_count = 1;
    expected_gi.rate = -1;
    expected_gi.first_in_group = true;
    expected_gi.time_variance = 0;
    expected_gi.start_time = -1;

    compare_gen_info(gen_it->second, expected_gi);

    ++gen_it;
    ASSERT_EQ(gen_it->first, 1);
    // Everything but total/gen counts stay the same
    expected_gi.total_count = 30;
    expected_gi.gen_count = 30;

    compare_gen_info(gen_it->second, expected_gi);

    ++gen_it;
    ASSERT_EQ(gen_it->first, 2);
    expected_gi.total_count = 0;
    expected_gi.gen_count = 0;
    compare_gen_info(gen_it->second, expected_gi);

    ++gen_it;
    ASSERT_EQ(gen_it->first, 3);
    expected_gi.total_count = 10;
    expected_gi.gen_count = 2;
    expected_gi.time_variance = 10;
    expected_gi.rate = 5;
    expected_gi.start_time = 8;
    compare_gen_info(gen_it->second, expected_gi);
}

TEST_P(MissionParseTest, mission_parse_test_entities) {
    std::map<int, std::map<std::string, std::string>> descriptions = mp.entity_descriptions();
    ASSERT_EQ(descriptions.size(), 4);

    auto compare_entity_descriptor = [&](std::map<std::string, std::string> actual_map,
                                         std::map<std::string, std::string> expected_map) {
        // EXPECT_EQ(actual_map.size(), expected_map.size());
        for (auto it : expected_map) {
            EXPECT_STREQ(actual_map[it.first].c_str(), it.second.c_str());
        }
    };

    auto descriptor_it = descriptions.begin();
    // Entity properties (plugins are now in all_entity_plugins())
    std::map<std::string, std::string> expected_map{
        std::make_pair("name", "uav_entity"),
        std::make_pair("team_id", "1"),
        std::make_pair("color", "77 77 255"),
        std::make_pair("health", "1"),
        std::make_pair("radius", "1"),
        std::make_pair("variance_x", "20"),
        std::make_pair("variance_y", "20"),
        std::make_pair("variance_z", "10"),
        std::make_pair("x", "-900"),
        std::make_pair("y", "0"),
        std::make_pair("z", "195"),
        std::make_pair("heading", "0"),
        std::make_pair("visual_model", "zephyr-blue"),
    };

    compare_entity_descriptor(descriptor_it->second, expected_map);

    // Verify plugins are in all_entity_plugins()
    const auto& plugins = get_entity_plugins(mp, 0);
    ASSERT_EQ(plugins.size(), 3);  // autonomy, controller, motion_model

    auto it_autonomy = plugins.find("autonomy:Straight");
    ASSERT_NE(it_autonomy, plugins.end());
    EXPECT_EQ(it_autonomy->second.name, "Straight");
    EXPECT_EQ(it_autonomy->second.type, "autonomy");
    EXPECT_EQ(it_autonomy->second.order, 0);
    EXPECT_EQ(it_autonomy->second.entity_name, "uav_entity");

    auto it_controller = plugins.find("controller:SimpleAircraftControllerPID");
    ASSERT_NE(it_controller, plugins.end());
    EXPECT_EQ(it_controller->second.name, "SimpleAircraftControllerPID");
    EXPECT_EQ(it_controller->second.type, "controller");

    auto it_motion = plugins.find("motion_model:SimpleAircraft");
    ASSERT_NE(it_motion, plugins.end());
    EXPECT_EQ(it_motion->second.name, "SimpleAircraft");
    EXPECT_EQ(it_motion->second.type, "motion_model");
}

TEST_P(MissionParseTest, mission_parse_test_team_info) {
    using ::testing::ElementsAreArray;

    auto compare_team_info = [](scrimmage::TeamInfo actual, scrimmage::TeamInfo expected) {
        EXPECT_EQ(actual.team_id, expected.team_id);

        auto actual_base_it = actual.bases.begin();
        auto expected_base_it = expected.bases.begin();
        for (; actual_base_it != actual.bases.cend() && expected_base_it != expected.bases.cend();
             ++actual_base_it, ++expected_base_it) {
            EXPECT_TRUE(actual_base_it->isApprox(*expected_base_it, 1e-4))
                << "Expected Vector\n"
                << *expected_base_it << "\n"
                << "Actual Vector\n"
                << *actual_base_it;
        }
        EXPECT_THAT(actual.radii, ElementsAreArray(expected.radii));
        EXPECT_THAT(actual.opacities, ElementsAreArray(expected.opacities));
        EXPECT_EQ(actual.color.r(), expected.color.r());
        EXPECT_EQ(actual.color.g(), expected.color.g());
        EXPECT_EQ(actual.color.b(), expected.color.b());
    };

    std::map<int, scrimmage::TeamInfo> team_infos = mp.team_info();
    auto team_it = team_infos.begin();
    scrimmage::TeamInfo expected_team_info;
    expected_team_info.team_id = 1;
    expected_team_info.bases.emplace_back(-215.3526, 9.65605, -0.00363862);
    expected_team_info.radii.push_back(25);
    expected_team_info.opacities.push_back(0.5);
    expected_team_info.color.set_r(77);
    expected_team_info.color.set_g(77);
    expected_team_info.color.set_b(255);
    compare_team_info(team_it->second, expected_team_info);
}

TEST(MissionParseStandaloneTest, entity_common_plugins_are_imported_into_entity_plugins) {
    scrimmage::MissionParse mp;
    ASSERT_TRUE(mp.parse("missions/test/test_entity_common_plugins.xml"));

    const auto& plugins = get_entity_plugins(mp, 0);
    ASSERT_EQ(plugins.size(), 5);

    auto it_common_auto = plugins.find("autonomy:Straight");
    ASSERT_NE(it_common_auto, plugins.end());
    EXPECT_EQ(it_common_auto->second.order, 0);
    EXPECT_EQ(it_common_auto->second.params.at("speed"), "10");

    auto it_entity_auto = plugins.find("autonomy:Straight:1");
    ASSERT_NE(it_entity_auto, plugins.end());
    EXPECT_EQ(it_entity_auto->second.order, 1);
    EXPECT_EQ(it_entity_auto->second.params.at("speed"), "20");

    auto it_controller = plugins.find("controller:SimpleAircraftControllerPID");
    ASSERT_NE(it_controller, plugins.end());
    EXPECT_EQ(it_controller->second.order, 0);
    EXPECT_EQ(it_controller->second.params.at("gain"), "1.5");

    auto it_motion = plugins.find("motion_model:SimpleAircraft");
    ASSERT_NE(it_motion, plugins.end());
    EXPECT_EQ(it_motion->second.order, 0);
    EXPECT_EQ(it_motion->second.params.at("max_speed"), "25");

    auto it_sensor = plugins.find("sensor:NoisyState");
    ASSERT_NE(it_sensor, plugins.end());
    EXPECT_EQ(it_sensor->second.order, 0);
    EXPECT_EQ(it_sensor->second.params.at("noise"), "0.1");

    for (const auto& [key, plugin] : plugins) {
        EXPECT_EQ(plugin.entity_name, "entity_common_stack");
        EXPECT_EQ(plugin.entity_tag, "common_entity");
        EXPECT_EQ(plugin.entity_block_id, 0);
    }
}

TEST(MissionParseStandaloneTest, entity_local_motion_model_overrides_entity_common_motion_model) {
    scrimmage::MissionParse mp;
    ASSERT_TRUE(mp.parse("missions/test/test_entity_common_motion_override.xml"));

    const auto& plugins = get_entity_plugins(mp, 0);
    EXPECT_EQ(plugins.count("motion_model:SimpleAircraft"), 0);

    auto it_motion = plugins.find("motion_model:Ballistic");
    ASSERT_NE(it_motion, plugins.end());
    EXPECT_EQ(it_motion->second.order, 0);

    auto motion_plugins = mp.get_plugins_by_type(0, "motion_model");
    ASSERT_EQ(motion_plugins.size(), 1);
    EXPECT_EQ(motion_plugins.front().name, "Ballistic");
    EXPECT_EQ(motion_plugins.front().params.at("max_speed"), "30");
}

INSTANTIATE_TEST_SUITE_P(
    MissionParsingTestSuite,
    MissionParseTest,
    testing::Values(
        "test/test_mission_parse_mission.xml",
        "test/test_mission_include/test_mission_parse_mission_include.xml"));
