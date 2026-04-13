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
 * @date 13 April 2026
 * @version 0.1.0
 * @brief Runtime plugin parameter override types for dynamic entity spawning.
 * @section DESCRIPTION
 * Type aliases and utility functions for passing per-spawn plugin parameter
 * overrides through the GenerateEntity mechanism. Used by SimControl and
 * Entity::init().
 *
 */

#ifndef INCLUDE_SCRIMMAGE_ENTITY_RUNTIMEPLUGINOVERRIDES_H_
#define INCLUDE_SCRIMMAGE_ENTITY_RUNTIMEPLUGINOVERRIDES_H_

#include <map>
#include <set>
#include <string>

#include "scrimmage/fwd_decl.h"

// Forward declarations for protobuf types
namespace scrimmage_msgs {
class GenerateEntity;
}

namespace scrimmage {

struct EntityPluginInfo;

// Runtime plugin parameter overrides for dynamically spawned entities.
// Used by GenerateEntity to pass per-spawn parameter changes to Entity::init().
//
// PluginParamMap: key-value pairs for a single plugin instance
//   Example: {"speed": "25", "show_camera_images": "true"}
//
// PluginInstanceParamOverrides: map from plugin index to its param overrides
//   Example: {0: {"speed": "25"}, 1: {"aggressive": "true"}}
//   (autonomy[0] gets speed=25, autonomy[1] gets aggressive=true)
//
// RuntimePluginOverrides: map from plugin type to instance overrides
//   Example: {
//     "autonomy":      {0: {"speed": "25"}},
//     "sensor":        {0: {"max_range": "100"}, 1: {"fov": "90"}},
//     "motion_model":  {0: {"max_speed": "50"}}
//   }
using PluginParamMap = std::map<std::string, std::string>;
using PluginInstanceParamOverrides = std::map<int, PluginParamMap>;
using RuntimePluginOverrides = std::map<std::string, PluginInstanceParamOverrides>;

// Check if a key is trying to override which plugin is used (e.g., "autonomy=Foo").
// Overriding plugin parameters is allowed (autonomy0.speed=25), but swapping out
// the plugin itself is not — the entity_tag defines plugin composition.
bool is_plugin_name_override_key(const std::string& key);

// Parse runtime plugin overrides from a GenerateEntity protobuf message.
// Resolves plugin_name to plugin_index when specified. If plugin_name matches
// multiple plugins of the same type, an error is logged. Populates
// runtime_plugin_overrides keyed by (plugin_type, plugin_index). Repeated
// entries for the same plugin instance are merged in message order, with later
// values overwriting earlier ones for the same parameter key.
// Returns true if parsing succeeded, false if errors occurred.
bool parse_runtime_plugin_overrides(
    const scrimmage_msgs::GenerateEntity& data,
    MissionParsePtr mp,
    int ent_desc_id,
    RuntimePluginOverrides& runtime_plugin_overrides);

// Build the set of parameter names valid for runtime overrides.
// Combines params from the plugin's XML config and mission inline attributes.
// Populates allowed_param_names. Returns true if successful.
bool get_allowed_runtime_plugin_param_names(
    const EntityPluginInfo& plugin_info,
    FileSearch& file_search,
    std::set<std::string>& allowed_param_names);

// Validate runtime plugin overrides against the mission and plugin configs.
// Checks that plugin indices exist and parameter names are declared.
// Returns true if valid, false if validation errors occurred.
bool validate_runtime_plugin_overrides(
    MissionParsePtr mp,
    FileSearchPtr file_search,
    int ent_desc_id,
    const RuntimePluginOverrides& runtime_plugin_overrides);

// Merge runtime overrides into a plugin's base params.
// Starts with plugin_info.params and overlays any matching runtime overrides.
// Returns the merged parameter map ready for plugin initialization.
std::map<std::string, std::string> resolve_plugin_params(
    const EntityPluginInfo& plugin_info,
    const RuntimePluginOverrides& runtime_plugin_overrides);

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_ENTITY_RUNTIMEPLUGINOVERRIDES_H_
