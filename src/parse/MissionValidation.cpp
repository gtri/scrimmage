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
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @date 06 April 2026
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include "scrimmage/parse/MissionValidation.h"

#include <algorithm>
#include <filesystem>
#include <iostream>

#include "scrimmage/common/FileSearch.h"
#include "scrimmage/parse/MissionParse.h"

namespace fs = std::filesystem;

namespace scrimmage {

ValidationResult MissionValidation::validate(
    const std::shared_ptr<MissionParse>& mp,
    FileSearch& file_search) {

    ValidationResult result;

    // Discover available plugins from SCRIMMAGE_PLUGIN_PATH, organized by type.
    // Plugin type is determined from directory structure: plugins/<type>/<PluginName>/
    // Works for both core SCRIMMAGE and external projects (they append to the path).
    result.available_plugins = discover_plugins_by_type("SCRIMMAGE_PLUGIN_PATH", file_search);

    // Validate entity plugins: autonomy, controller, motion_model, sensor.
    // These are attached to specific entities via <entity> blocks in the mission file.
    for (const auto& [entity_desc_id, plugins] : mp->all_entity_plugins()) {
        for (const auto& [key, plugin] : plugins) {
            if (plugin.name.empty()) continue;

            const auto& available = result.available_plugins[plugin.type];
            bool found = available.count(plugin.name) > 0;

            PluginValidationInfo info;
            info.status = found ? PluginValidationInfo::Status::valid
                                : PluginValidationInfo::Status::config_not_found;

            if (!found) {
                info.suggestions = find_similar(plugin.name, available);

                ValidationError err;
                err.type = ValidationError::Type::PLUGIN_CONFIG_NOT_FOUND;
                err.plugin_name = plugin.name;
                err.plugin_type = plugin.type;
                err.context = "entity block " + std::to_string(entity_desc_id);
                err.message = "Plugin '" + plugin.name + "' not found";
                err.suggestions = info.suggestions;
                result.errors.push_back(err);
            }

            result.entity_plugin_diagnostics[entity_desc_id][key] = info;
        }
    }

    // Validate global (simulation-level) plugins: entity_interaction, network, metrics.
    // These are not attached to entities - they apply to the entire simulation.
    auto validate_global = [&](const std::list<std::string>& plugins, const std::string& type) {
        for (const std::string& instance_name : plugins) {
            auto attrs_it = mp->attributes().find(instance_name);
            // Validation must check the implementation name, not the instance alias from name=.
            const std::string plugin_name =
                (attrs_it != mp->attributes().end())
                    ? attrs_it->second["ORIGINAL_PLUGIN_NAME"]
                    : instance_name;

            const auto& available = result.available_plugins[type];
            bool found = available.count(plugin_name) > 0;

            if (!found) {
                ValidationError err;
                err.type = ValidationError::Type::PLUGIN_CONFIG_NOT_FOUND;
                err.plugin_name = plugin_name;
                err.plugin_type = type;
                err.context = "global";
                err.message = type + " plugin '" + plugin_name + "' not found";
                err.suggestions = find_similar(plugin_name, available);
                result.errors.push_back(err);
            }
        }
    };

    validate_global(mp->entity_interactions(), "entity_interaction");
    validate_global(mp->network_names(), "network");
    validate_global(mp->metrics(), "metrics");

    return result;
}

void MissionValidation::print_errors(
    const ValidationResult& result,
    const std::string& mission_filename) const {

    if (result.valid()) return;

    std::cerr << "\n";
    std::cerr << "========================================\n";
    std::cerr << "ERROR: Invalid mission file '" << mission_filename << "'\n";
    std::cerr << "========================================\n\n";

    for (const auto& err : result.errors) {
        std::cerr << "  [" << err.plugin_type << "] Plugin config not found\n";
        std::cerr << "    Plugin: '" << err.plugin_name << "'\n";

        if (!err.context.empty() && err.context != "global") {
            std::cerr << "    In: " << err.context << "\n";
        }

        if (!err.suggestions.empty()) {
            std::cerr << "    Did you mean: ";
            for (size_t i = 0; i < err.suggestions.size(); ++i) {
                if (i > 0) std::cerr << ", ";
                std::cerr << "'" << err.suggestions[i] << "'";
            }
            std::cerr << " ?\n";
        }
        std::cerr << "\n";
    }

    std::cerr << "Hint: Plugins are searched in SCRIMMAGE_PLUGIN_PATH.\n";
    const char* path = std::getenv("SCRIMMAGE_PLUGIN_PATH");
    if (path) {
        std::cerr << "  Current SCRIMMAGE_PLUGIN_PATH: " << path << "\n";
    } else {
        std::cerr << "  Warning: SCRIMMAGE_PLUGIN_PATH is not set!\n";
    }
    std::cerr << "\n";
}

// Discover plugins organized by type from directory structure.
// Path format: .../<anything>/plugins/<type>/<PluginName>/<PluginName>.xml
// Maps directory names to our type names (e.g., "motion" -> "motion_model")
//
// IMPORTANT: Path Structure Requirement for Submodules
// -----------------------------------------------------
// This function determines plugin type by parsing the directory structure,
// looking for a "plugins" directory followed by a type directory
// (autonomy, controller, motion, sensor, interaction, metrics, network).
//
// The "plugins" directory can be nested at any depth — we search for it anywhere
// in the path. These all work:
//
//   submodule/plugins/autonomy/MyAutonomy/MyAutonomy.xml           <-- WORKS
//   submodule/src/plugins/autonomy/MyAutonomy/MyAutonomy.xml       <-- WORKS
//   external/lib/plugins/motion/MyMotion/MyMotion.xml              <-- WORKS
//
// What matters is: plugins/<type>/<PluginName>/<PluginName>.xml
//
// This will NOT work:
//   submodule/MyAutonomy/MyAutonomy.xml                            <-- NO "plugins" dir
//   submodule/autonomy/MyAutonomy/MyAutonomy.xml                   <-- NO "plugins" dir
//
// NOTE: Plugin XML config files do NOT contain type information — they only have
// <library>PluginName_plugin</library> and plugin-specific params. The directory
// structure is the ONLY source of truth for plugin type.
std::map<std::string, std::set<std::string>> MissionValidation::discover_plugins_by_type(
    const std::string& env_var,
    FileSearch& file_search) {

    std::map<std::string, std::set<std::string>> plugins_by_type;
    std::unordered_map<std::string, std::list<std::string>> xml_files;
    file_search.find_files(env_var, ".xml", xml_files, verbose_);

    for (const auto& kv : xml_files) {
        const std::string& filename = kv.first;
        const std::list<std::string>& paths = kv.second;

        if (filename.size() <= 4 || filename.substr(filename.size() - 4) != ".xml") {
            continue;
        }
        std::string plugin_name = filename.substr(0, filename.size() - 4);

        // Extract type from path: .../plugins/<type>/<PluginName>/...
        if (paths.empty()) continue;
        fs::path p(paths.front());
        auto it = std::find(p.begin(), p.end(), "plugins");
        if (it != p.end() && ++it != p.end()) {
            auto type_it = kPluginDirToType.find(it->string());
            if (type_it != kPluginDirToType.end()) {
                plugins_by_type[type_it->second].insert(plugin_name);
            }
        }
    }
    return plugins_by_type;
}

// Find plugin names similar to the typo using Levenshtein (edit) distance.
// Edit distance = minimum insertions, deletions, or substitutions to transform
// one string into another. Low distance means likely typo.
// e.g., "Straigt" -> "Straight" has distance 1 (insert 'h')
std::vector<std::string> MissionValidation::find_similar(
    const std::string& name,
    const std::set<std::string>& available,
    size_t max_suggestions) const {

    std::vector<std::pair<size_t, std::string>> scored;

    std::string name_lower = name;
    std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

    for (const auto& candidate : available) {
        std::string cand_lower = candidate;
        std::transform(cand_lower.begin(), cand_lower.end(), cand_lower.begin(), ::tolower);

        size_t dist = levenshtein_distance(name_lower, cand_lower);
        if (dist <= (name.size() / 2 + 2)) {
            scored.push_back({dist, candidate});
        }
    }

    std::sort(scored.begin(), scored.end());

    std::vector<std::string> suggestions;
    for (size_t i = 0; i < std::min(max_suggestions, scored.size()); ++i) {
        suggestions.push_back(scored[i].second);
    }
    return suggestions;
}

size_t MissionValidation::levenshtein_distance(
    const std::string& s1,
    const std::string& s2) const {

    size_t m = s1.size();
    size_t n = s2.size();

    if (m == 0) return n;
    if (n == 0) return m;

    std::vector<std::vector<size_t>> dp(m + 1, std::vector<size_t>(n + 1));

    for (size_t i = 0; i <= m; ++i) dp[i][0] = i;
    for (size_t j = 0; j <= n; ++j) dp[0][j] = j;

    for (size_t i = 1; i <= m; ++i) {
        for (size_t j = 1; j <= n; ++j) {
            size_t cost = (s1[i - 1] == s2[j - 1]) ? 0 : 1;
            dp[i][j] = std::min({
                dp[i - 1][j] + 1,
                dp[i][j - 1] + 1,
                dp[i - 1][j - 1] + cost
            });
        }
    }
    return dp[m][n];
}

}  // namespace scrimmage
