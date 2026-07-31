/*!
 * @file
 */

#include <gmock/gmock.h>

#include "scrimmage/common/FileSearch.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/MissionValidation.h"

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

TEST(MissionValidationTest, diagnostics_are_stored_in_validation_result) {
    auto mp = std::make_shared<scrimmage::MissionParse>();
    ASSERT_TRUE(mp->parse("missions/test-duplicate-invalid-plugin.xml"));

    scrimmage::FileSearch file_search;
    scrimmage::MissionValidation validator;

    auto result = validator.validate(mp, file_search);

    EXPECT_FALSE(result.valid());
    ASSERT_EQ(result.errors.size(), 2);

    auto entity_it = result.entity_plugin_diagnostics.find(0);
    ASSERT_NE(entity_it, result.entity_plugin_diagnostics.end());

    auto first_it = entity_it->second.find("autonomy:Straigt");
    ASSERT_NE(first_it, entity_it->second.end());
    EXPECT_EQ(first_it->second.status, scrimmage::PluginValidationInfo::Status::config_not_found);
    EXPECT_THAT(first_it->second.suggestions, testing::Contains("Straight"));

    auto second_it = entity_it->second.find("autonomy:Straigt:1");
    ASSERT_NE(second_it, entity_it->second.end());
    EXPECT_EQ(second_it->second.status, scrimmage::PluginValidationInfo::Status::config_not_found);
    EXPECT_THAT(second_it->second.suggestions, testing::Contains("Straight"));

    const auto& plugins = get_entity_plugins(*mp, 0);
    ASSERT_NE(plugins.find("autonomy:Straigt"), plugins.end());
    ASSERT_NE(plugins.find("autonomy:Straigt:1"), plugins.end());
}