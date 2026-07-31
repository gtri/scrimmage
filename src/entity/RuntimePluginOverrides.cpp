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
 *   the Free Software Foundation, either version 3 of the License, or (at your
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
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @date 13 April 2026
 * @version 0.1.0
 * @brief Implementation of runtime plugin override utilities.
 * @section DESCRIPTION
 * Parsing and validation logic for per-spawn plugin parameter overrides
 * passed through the GenerateEntity mechanism.
 *
 */

#include "scrimmage/entity/RuntimePluginOverrides.h"

#include <map>
#include <mutex>
#include <set>
#include <string>

#include "scrimmage/common/FileSearch.h"
#include "scrimmage/log/Logger.h"
#include "scrimmage/msgs/Event.pb.h"
#include "scrimmage/parse/ConfigParse.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/MissionValidation.h"
#include "scrimmage/parse/ParseUtils.h"

namespace scrimmage {

namespace {

// Strict runtime override validation derives allowed keys from
// ConfigParse::params(). That map includes parser metadata and plugin loading
// fields, so strip those out before treating the remaining names as user
// overridable runtime parameters.
const std::set<std::string> kIgnoredPluginConfigParams = {
    "XML_DIR",
    "XML_FILENAME",
    "library"
};

// Sensor placement keys that Entity.cpp reads to mount sensors on the vehicle.
// Unlike behavior params (e.g., max_range, fov), these are NOT declared in
// plugin XML files like RayTrace.xml — they're construction-time placement
// params that Entity.cpp handles specially. If a mission template omits them
// (using default placement), they won't appear in either:
//   - Source 1 (plugin XML config), or
//   - Source 2 (mission inline attributes)
// But you may still want to override them at runtime (e.g., spawn one drone
// with sensor pointing left, another pointing right). So we hardcode them as
// always-allowed for sensors.
const std::set<std::string> kSensorRuntimeOverrideParams = {
    "rpy",
    "xyz"
};

constexpr int kNoPluginNameFound = -1;
constexpr int kDuplicatePluginNameFound = -2;

}  // namespace

bool is_plugin_name_override_key(const std::string& key) {
    if (kEntityPluginTypes.count(key) > 0) {
        return true;
    }

    const size_t suffix_end = key.find_last_not_of("0123456789");
    if (suffix_end == std::string::npos) {
        return false;
    }

    return kEntityPluginTypes.count(key.substr(0, suffix_end + 1)) > 0;
}

// Resolve plugin_name to plugin_index within the given plugin type for an entity.
// Returns kNoPluginNameFound if name not found,
// kDuplicatePluginNameFound if name matches multiple plugins.
static int resolve_plugin_name_to_index(
    MissionParsePtr mp,
    int ent_desc_id,
    const std::string& plugin_type,
    const std::string& plugin_name) {
    const auto plugins = mp->get_plugins_by_type(ent_desc_id, plugin_type);
    int found_index = kNoPluginNameFound;
    for (size_t i = 0; i < plugins.size(); ++i) {
        if (plugins[i].name == plugin_name) {
            if (found_index >= 0) {
                return kDuplicatePluginNameFound;
            }
            found_index = static_cast<int>(i);
        }
    }
    return found_index;
}

bool parse_runtime_plugin_overrides(
    const scrimmage_msgs::GenerateEntity& data,
    MissionParsePtr mp,
    int ent_desc_id,
    RuntimePluginOverrides& runtime_plugin_overrides) {
    for (int i = 0; i < data.plugin_override_size(); ++i) {
        const auto& proto_override = data.plugin_override(i);
        if (kEntityPluginTypes.count(proto_override.plugin_type()) == 0) {
            LOG_ERROR("GenerateEntity: Unknown plugin_type '" << proto_override.plugin_type()
                     << "'. Expected one of autonomy, controller, motion_model, sensor.");
            return false;
        }

        // Resolve plugin_name to plugin_index if specified
        int resolved_index = static_cast<int>(proto_override.plugin_index());
        if (proto_override.has_plugin_name()) {
            resolved_index = resolve_plugin_name_to_index(
                mp, ent_desc_id, proto_override.plugin_type(), proto_override.plugin_name());
            if (resolved_index == kNoPluginNameFound) {
                LOG_ERROR("GenerateEntity: Plugin name '" << proto_override.plugin_name()
                         << "' not found in " << proto_override.plugin_type()
                         << " plugins for entity block " << ent_desc_id << ".");
                return false;
            }
            if (resolved_index == kDuplicatePluginNameFound) {
                LOG_ERROR("GenerateEntity: Plugin name '" << proto_override.plugin_name()
                         << "' matches multiple " << proto_override.plugin_type()
                         << " plugins. Use plugin_index to disambiguate.");
                return false;
            }
        }

        // Allow callers to build up overrides across nested control flow.
        // Repeated plugin_override entries targeting the same plugin instance
        // are merged in message order, with later values overwriting earlier
        // ones for the same key.
        auto& instance_overrides =
            runtime_plugin_overrides[proto_override.plugin_type()][resolved_index];

        for (int j = 0; j < proto_override.params_size(); ++j) {
            const auto& param = proto_override.params(j);
            instance_overrides[param.key()] = param.value();
        }
    }

    return true;
}

// Builds the set of parameter names that are valid for runtime overrides.
// This combines two sources:
//   1. Params declared in the plugin's XML config file (e.g., Straight.xml)
//      These are the defaults the plugin author defined. Cached because the
//      plugin XML doesn't change, and parsing it repeatedly is expensive.
//   2. Params set inline in the mission file (e.g., <autonomy speed="21">)
//      These are entity-specific and come from plugin_info.params. Not cached
//      because they vary per entity block.
// The union of both sets forms the allowed runtime override keys.
bool get_allowed_runtime_plugin_param_names(
    const EntityPluginInfo& plugin_info,
    FileSearch& file_search,
    std::set<std::string>& allowed_param_names) {

    // --- Source 1: Plugin XML config params (cached) ---
    // These are params declared in the plugin's own XML file, like:
    //   include/scrimmage/plugins/autonomy/Straight/Straight.xml
    // They represent what the plugin implementation accepts, regardless of
    // whether the mission file mentions them.
    static std::map<std::string, std::set<std::string>> cached_param_names;
    static std::mutex cache_mutex;

    const std::string cache_key = plugin_info.type + ":" + plugin_info.name;

    // Check cache first to avoid re-parsing the same plugin XML repeatedly.
    {
        std::lock_guard<std::mutex> lock(cache_mutex);
        auto cache_it = cached_param_names.find(cache_key);
        if (cache_it != cached_param_names.end()) {
            allowed_param_names = cache_it->second;
        }
    }

    // Cache miss: parse the plugin XML to extract declared param names.
    if (allowed_param_names.empty()) {
        ConfigParse config_parse;
        config_parse.set_required("library");
        const std::map<std::string, std::string> empty_overrides;
        if (!config_parse.parse(empty_overrides, plugin_info.name, "SCRIMMAGE_PLUGIN_PATH", file_search)) {
            LOG_ERROR("GenerateEntity: Failed to inspect plugin config for '" << plugin_info.name
                     << "' while validating runtime overrides.");
            return false;
        }

        for (const auto& [key, value] : config_parse.params()) {
            static_cast<void>(value);
            if (kIgnoredPluginConfigParams.count(key) == 0) {
                allowed_param_names.insert(key);
            }
        }

        std::lock_guard<std::mutex> lock(cache_mutex);
        cached_param_names[cache_key] = allowed_param_names;
    }

    // --- Source 2: Mission file inline params (not cached) ---
    // These are params written inline in the mission XML, like:
    //   <autonomy speed="21">Straight</autonomy>
    // They vary per entity block, so we add them on every call.
    for (const auto& [key, value] : plugin_info.params) {
        static_cast<void>(value);
        allowed_param_names.insert(key);
    }

    // Sensors also accept transform keys (rpy, xyz) for placement overrides.
    if (plugin_info.type == "sensor") {
        allowed_param_names.insert(kSensorRuntimeOverrideParams.begin(), kSensorRuntimeOverrideParams.end());
    }

    return true;
}

bool validate_runtime_plugin_overrides(
    MissionParsePtr mp,
    FileSearchPtr file_search,
    int ent_desc_id,
    const RuntimePluginOverrides& runtime_plugin_overrides) {
    const bool strict_runtime_plugin_params =
        get("strict_runtime_plugin_params", mp->params(), true);

    for (const auto& [plugin_type, instance_overrides] : runtime_plugin_overrides) {
        if (kEntityPluginTypes.count(plugin_type) == 0) {
            LOG_ERROR("GenerateEntity: Unknown runtime override plugin type '" << plugin_type << "'.");
            return false;
        }

        const auto plugins = mp->get_plugins_by_type(ent_desc_id, plugin_type);
        for (const auto& [plugin_index, param_overrides] : instance_overrides) {
            if (plugin_index >= static_cast<int>(plugins.size())) {
                LOG_ERROR("GenerateEntity: Entity block " << ent_desc_id
                         << " does not define " << plugin_type << "[" << plugin_index
                         << "] for runtime overrides.");
                return false;
            }

            if (param_overrides.empty()) {
                LOG_WARN("GenerateEntity: Empty runtime override for " << plugin_type << "["
                         << plugin_index << "] ignored.");
            }

            if (!strict_runtime_plugin_params) {
                continue;
            }

            std::set<std::string> allowed_param_names;
            if (!get_allowed_runtime_plugin_param_names(plugins[plugin_index], *file_search, allowed_param_names)) {
                return false;
            }

            for (const auto& [param_name, param_value] : param_overrides) {
                static_cast<void>(param_value);
                if (allowed_param_names.count(param_name) == 0) {
                    LOG_ERROR("GenerateEntity: Runtime override key '" << param_name << "' is not declared for "
                             << plugin_type << "[" << plugin_index << "] plugin '"
                             << plugins[plugin_index].name << "'. "
                             << "Set strict_runtime_plugin_params=false in the mission file to allow unknown keys.");
                    return false;
                }
            }
        }
    }

    return true;
}

std::map<std::string, std::string> resolve_plugin_params(
    const EntityPluginInfo& plugin_info,
    const RuntimePluginOverrides& runtime_plugin_overrides) {
    std::map<std::string, std::string> resolved_params = plugin_info.params;

    auto type_it = runtime_plugin_overrides.find(plugin_info.type);
    if (type_it == runtime_plugin_overrides.end()) {
        return resolved_params;
    }

    auto order_it = type_it->second.find(plugin_info.order);
    if (order_it == type_it->second.end()) {
        return resolved_params;
    }

    for (const auto& [key, value] : order_it->second) {
        resolved_params[key] = value;
    }

    return resolved_params;
}

}  // namespace scrimmage
