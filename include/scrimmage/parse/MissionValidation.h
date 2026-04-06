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

#ifndef INCLUDE_SCRIMMAGE_PARSE_MISSIONVALIDATION_H_
#define INCLUDE_SCRIMMAGE_PARSE_MISSIONVALIDATION_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace scrimmage {

// Maps plugin directory names to internal type names.
// Update here if plugin types are added or renamed.
inline const std::map<std::string, std::string> kPluginDirToType = {
    {"autonomy", "autonomy"},
    {"controller", "controller"},
    {"motion", "motion_model"},
    {"sensor", "sensor"},
    {"interaction", "entity_interaction"},
    {"metrics", "metrics"},
    {"network", "network"}
};

// Entity-level plugin types (attached to specific entities, not global).
inline const std::set<std::string> kEntityPluginTypes = {
    "autonomy", "controller", "motion_model", "sensor"
};

class FileSearch;
class MissionParse;

struct ValidationError {
    enum class Type {
        PLUGIN_CONFIG_NOT_FOUND,
        PLUGIN_LIBRARY_NOT_FOUND,
        PLUGIN_LIBRARY_INVALID,
        UNKNOWN_PLUGIN_TYPE
    };

    Type type;
    std::string plugin_name;
    std::string plugin_type;  // autonomy, controller, motion_model, etc.
    std::string context;      // location in mission file (entity block index or "global")
    std::string message;
    std::vector<std::string> suggestions;
};

struct PluginValidationInfo {
    enum class Status {
        unknown,
        valid,
        config_not_found,
    };

    Status status = Status::unknown;
    std::string config_xml_path;
    std::vector<std::string> suggestions;
};

struct ValidationResult {
    bool valid() const { return errors.empty(); }
    std::vector<ValidationError> errors;
    std::map<std::string, std::set<std::string>> available_plugins;
    std::map<int, std::map<std::string, PluginValidationInfo>> entity_plugin_diagnostics;
};

class MissionValidation {
 public:
    ValidationResult validate(
        const std::shared_ptr<MissionParse>& mp,
        FileSearch& file_search);

    void print_errors(
        const ValidationResult& result,
        const std::string& mission_filename) const;

    void set_verbose(bool verbose) { verbose_ = verbose; }

 protected:
    std::map<std::string, std::set<std::string>> discover_plugins_by_type(
        const std::string& env_var,
        FileSearch& file_search);

    std::vector<std::string> find_similar(
        const std::string& name,
        const std::set<std::string>& available,
        size_t max_suggestions = 3) const;

    size_t levenshtein_distance(
        const std::string& s1,
        const std::string& s2) const;

 private:
    bool verbose_ = false;
};

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PARSE_MISSIONVALIDATION_H_
