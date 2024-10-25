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

#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/XMLParser/RapidXMLParser.h>
#include <scrimmage/parse/XMLParser/XMLParser.h>
#include <scrimmage/proto/ProtoConversions.h>

#if ENABLE_LIBXML2_PARSER
#include <scrimmage/parse/XMLParser/LibXML2Parser.h>
#endif

#include <fstream>
#include <iostream>
#include <regex>  //NOLINT
#include <string>
#include <typeinfo>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <boost/algorithm/string.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

using std::cout;
using std::endl;

namespace fs = boost::filesystem;
namespace scrimmage {

void MissionParse::set_overrides(const std::string &overrides) {
    std::string overrides_no_space = remove_whitespace(overrides);

    // Iterate over comma-separated overrides.
    std::vector<std::string> overrides_tokens;
    split(overrides_tokens, overrides_no_space, ", ");
    for (auto &overrides_str : overrides_tokens) {
        // Parse each key=value pair
        std::vector<std::string> kv_tokens;
        split(kv_tokens, overrides_str, ":= ");

        // Only add to the map if the key:value was parsed correctly
        if (kv_tokens.size() == 2) {
            overrides_map_[kv_tokens[0]] = kv_tokens[1];
        }
    }
}

bool MissionParse::parse(const std::string &filename) {
    read_file_content(filename);
    mission_file_content_ = replace_overrides(mission_file_content_);
#if ENABLE_LIBXML2_PARSER
    std::size_t xIncludePos = mission_file_content_.find("http://www.w3.org/2001/XInclude");
    bool useLibXMLParser = xIncludePos != std::string::npos;
    if (useLibXMLParser) {
        return parse_mission<LibXML2Parser>();
    } else {
        return parse_mission<RapidXMLParser>();
    }
#else
    return parse_mission<RapidXMLParser>();
#endif
}

bool MissionParse::read_file_content(const std::string &filename) {
    mission_filename_ = expand_user(filename);
    // First, explicitly search for the mission file.
    if (!fs::exists(mission_filename_)) {
        // If the file doesn't exist, search for the mission file under the
        // SCRIMMAGE_MISSION_PATH.
        FileSearch file_search;
        std::string result = "";
        bool status = file_search.find_file(
            mission_filename_, "xml", "SCRIMMAGE_MISSION_PATH", result, false);
        if (!status) {
            // The mission file wasn't found. Exit.
            cout << "SCRIMMAGE mission file not found: " << mission_filename_ << endl;
            return false;
        }
        // The mission file was found, save its path.
        mission_filename_ = result;
    }

    std::ifstream file(mission_filename_.c_str());
    if (!file.is_open()) {
        std::cout << "Failed to open mission file: " << mission_filename_ << endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    mission_file_content_ = buffer.str();
    return true;
}

std::string MissionParse::replace_overrides(std::string str) {
    // Search and replace any overrides of the form ${key=value} in the mission
    // file
    for (auto &kv : overrides_map_) {
        std::regex reg("\\$\\{" + kv.first + "=(.+?)\\}");
        str = std::regex_replace(str, reg, kv.second);
    }

    // Replace our xml variables of the form ${var=default} with the default
    // value
    std::string fmt{"$1"};
    std::regex reg("\\$\\{.+?=(.+?)\\}");
    str = std::regex_replace(str, reg, fmt);
    return str;
}

template <class Parser>
bool MissionParse::parse_filecontents(Parser& doc) {
    doc.set_filename(mission_filename_);
    // doc.parse requires a null terminated string that it can modify.
    std::vector<char> mission_file_content_vec(mission_file_content_.size() + 1);
    mission_file_content_vec.assign(mission_file_content_.begin(),
                                    mission_file_content_.end());  // copy
    mission_file_content_vec.push_back('\0');                      // shouldn't reallocate
    return doc.parse(mission_file_content_vec);
}

#if ENABLE_LIBXML2_PARSER
template <>
bool MissionParse::parse_filecontents<LibXML2Parser>(LibXML2Parser& doc) {
    // LibXMLParser is used when other xml files are included via xinclude to create a composite 
    // xml mission file. We need to parse this contents twice to ensure that proper override substitution
    // occurs
    doc.set_filename(mission_filename_);
    std::vector<char> mission_file_content_vec(mission_file_content_.size());
    mission_file_content_vec.assign(mission_file_content_.begin(),
                                    mission_file_content_.end());  // copy
    doc.parse(mission_file_content_vec); 
    mission_file_content_vec = doc.get_filecontents();

    mission_file_content_.assign(mission_file_content_vec.cbegin(), mission_file_content_vec.cend());
    mission_file_content_ = replace_overrides(mission_file_content_);
    mission_file_content_vec.assign(mission_file_content_.begin(),
                                    mission_file_content_.end());
    
    return doc.parse(mission_file_content_vec);
}
#endif

template <class Parser>
bool MissionParse::parse_mission() {
    // std::cout << "Parsing with " << typeid(Parser).name() << "\n";
    //  Parse the xml tree.

    // Helpers to avoid passing bad strings into stoi and stod
    auto parse_int = [&](std::string str) { return std::stoi(replace_overrides(str)); };

    auto parse_double = [&](std::string str) { return std::stod(replace_overrides(str)); };

    Parser doc;
    //doc.set_filename(mission_filename_);
    //// doc.parse requires a null terminated string that it can modify.
    //std::vector<char> mission_file_content_vec(mission_file_content_.size() + 1);
    //mission_file_content_vec.assign(mission_file_content_.begin(),
    //                                mission_file_content_.end());  // copy
    //mission_file_content_vec.push_back('\0');                      // shouldn't reallocate
    //doc.parse(mission_file_content_vec);

    parse_filecontents(doc);
    auto runscript_node = doc.first_node("runscript");
    if (!runscript_node.is_valid()) {
        cout << "Missing runscript tag." << endl;
        return false;
    }

    auto run_node = runscript_node.first_node("run");
    if (!run_node.is_valid()) {
        cout << "Missing run node" << endl;
        return false;
    }

    motion_multiplier_ = 1;
    for (auto attr = run_node.first_attribute(); attr.is_valid(); attr = attr.next()) {
        std::string attr_str(attr.name());
        if (attr_str == "start") {
            t0_ = parse_double(attr.value());
        } else if (attr_str == "end") {
            tend_ = parse_double(attr.value());
        } else if (attr_str == "dt") {
            dt_ = parse_double(attr.value());
        } else if (attr_str == "motion_multiplier") {
            motion_multiplier_ = parse_double(attr.value());
        } else if (attr_str == "time_warp") {
            time_warp_ = parse_double(attr.value());
        } else if (attr_str == "enable_gui") {
            enable_gui_ = str2bool(attr.value());
        } else if (attr_str == "network_gui") {
            network_gui_ = str2bool(attr.value());
        } else if (attr_str == "start_paused") {
            start_paused_ = str2bool(attr.value());
        } else if (attr_str == "full_screen") {
            full_screen_ = str2bool(attr.value());
        } else if (attr_str == "window_width") {
            window_width_ = parse_double(attr.value());
        } else if (attr_str == "window_height") {
            window_height_ = parse_double(attr.value());
        }
    }

    auto parse_tags = [&](const std::string &tagname, std::list<std::string> &tag_list) {
        for (auto node = runscript_node.first_node(tagname); node.is_valid();
             node = node.next_sibling(tagname)) {
            // If a "name" is specified, use this name
            auto attr = node.first_attribute("name");
            std::string name = (!attr.is_valid()) ? node.value() : attr.value();
            tag_list.push_back(name);
        }
    };

    parse_tags("entity_interaction", entity_interactions_);
    parse_tags("network", network_names_);
    parse_tags("metrics", metrics_);

    // param_common name: tag: value
    std::map<std::string, std::map<std::string, std::string>> param_common;
    for (auto script_node = runscript_node.first_node("param_common"); script_node.is_valid();
         script_node = script_node.next_sibling("param_common")) {
        auto nm_attr = script_node.first_attribute("name");
        if (!nm_attr.is_valid()) {
            std::cout << "warning: found param_common block without a name, skipping" << std::endl;
            continue;
        }

        std::string nm = nm_attr.value();

        for (auto node = script_node.first_node(); node.is_valid(); node = node.next_sibling()) {
            param_common[nm][node.name()] = node.value();
        }
    }

    // Loop through each node under "runscript" that isn't an entity or base
    attributes_.clear();
    for (auto node = runscript_node.first_node(); node.is_valid(); node = node.next_sibling()) {
        std::string nm = node.name();
        if (nm != "entity" && nm != "base" && nm != "entity_common" && nm != "param_common") {
            params_[nm] = node.value();

            auto attr = node.first_attribute("name");
            std::string name = (!attr.is_valid()) ? node.value() : attr.value();

            std::string nm2 = nm == "entity_interaction" ? name : nm;
            std::string nm3 = nm == "metrics" ? name : nm;
            std::string nm4 = nm == "network" ? name : nm;

            attributes_[nm2]["ORIGINAL_PLUGIN_NAME"] = node.value();
            attributes_[nm3]["ORIGINAL_PLUGIN_NAME"] = node.value();
            attributes_[nm4]["ORIGINAL_PLUGIN_NAME"] = node.value();

            // Loop through each node's attributes:
            for (auto attr = node.first_attribute(); attr.is_valid(); attr = attr.next()) {
                std::string attr_name = attr.name();
                if (attr_name == "param_common") {
                    for (auto &kv : param_common[attr.value()]) {
                        attributes_[nm2][kv.first] = kv.second;
                        attributes_[nm3][kv.first] = kv.second;
                        attributes_[nm4][kv.first] = kv.second;
                    }
                } else {
                    attributes_[nm2][attr.name()] = attr.value();
                    attributes_[nm3][attr.name()] = attr.value();
                    attributes_[nm4][attr.name()] = attr.value();
                }
            }
        }
    }

    // Save background color:
    bool bg_color_result = false;
    std::vector<int> temp_color;
    if (params_.count("background_color") > 0) {
        bg_color_result = str2container(params_["background_color"], " ", temp_color, 3);
    }
    if (bg_color_result) {
        background_color_.set_r(temp_color[0]);
        background_color_.set_g(temp_color[1]);
        background_color_.set_b(temp_color[2]);
    }

    // Is latitude_origin defined?
    if (params_.count("latitude_origin") > 0) {
        latitude_origin_ = parse_double(params_["latitude_origin"]);
    }

    // Is longitude_origin defined?
    if (params_.count("longitude_origin") > 0) {
        longitude_origin_ = parse_double(params_["longitude_origin"]);
    }

    // Is altitude_origin defined?
    if (params_.count("altitude_origin") > 0) {
        altitude_origin_ = parse_double(params_["altitude_origin"]);
    }

    set_lat_lon_alt_origin(latitude_origin_, longitude_origin_, altitude_origin_);

    // Handle log directory
    root_log_dir_ = expand_user("~/.scrimmage/logs");
    if (params_.count("log_dir") > 0) {
        // Get the dir attribute of the log node
        root_log_dir_ = expand_user(params_["log_dir"]);
    }

    // Is output_dir_trailer defined?
    if (params_.count("output_dir_trailer") > 0) {
        output_dir_trailer_ = "_" + params_["output_dir_trailer"].substr(0, 100);
        std::replace(output_dir_trailer_.begin(), output_dir_trailer_.end(), '/', '_');
    } else {
        output_dir_trailer_ = "";
    }

    // Create a directory to hold the log data
    // Use the current time for the directory's name
    time_t rawtime;
    struct tm *timeinfo;
    char time_buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);
    std::string name(time_buffer);

    log_dir_ = root_log_dir_ + "/" + name;

    if (job_number_ != -1) {
        log_dir_ += "_job_" + std::to_string(job_number_);
    }
    if (task_number_ != -1) {
        log_dir_ += "_task_" + std::to_string(task_number_);
    }
    log_dir_ += output_dir_trailer_;

    int ent_desc_id = 0;
    int team_id_err = 0;  // only used when team_id is not set "warn condition"

    // common block name -> <node_name, value>
    std::map<std::string, AttributeMap> entity_common_attributes;
    std::map<std::string, std::map<std::string, std::string>> entity_common;
    std::map<std::string, std::map<std::string, int>> orders;
    for (auto script_node = runscript_node.first_node("entity_common"); script_node.is_valid();
         script_node = script_node.next_sibling("entity_common")) {
        std::map<std::string, std::string> script_info;

        auto nm_attr = script_node.first_attribute("name");

        if (!nm_attr.is_valid()) {
            cout << "warning: found entity_common block without a name, skipping" << endl;
            continue;
        }

        std::string nm = nm_attr.value();

        for (auto node = script_node.first_node(); node.is_valid(); node = node.next_sibling()) {
            std::string node_name = node.name();

            if (node_name == "name") {
                continue;
            }

            if (node_name == "autonomy") {
                node_name += std::to_string(orders[nm]["autonomy"]++);
            } else if (node_name == "controller") {
                node_name += std::to_string(orders[nm]["controller"]++);
            } else if (node_name == "sensor") {
                node_name += std::to_string(orders[nm]["sensor"]++);
            }

            // Loop through each node's attributes:
            for (auto attr = node.first_attribute(); attr.is_valid(); attr = attr.next()) {
                const std::string attr_name = attr.name();
                if (attr_name == "param_common") {
                    for (auto &kv : param_common[attr.value()]) {
                        entity_common_attributes[nm][node_name][kv.first] = kv.second;
                    }
                } else {
                    entity_common_attributes[nm][node_name][attr_name] = attr.value();
                }
            }

            if (script_info.count(node_name) > 0 && node_name.compare("team_id")) {
                cout << "Warning: entity contains multiple tags for \"" << node_name << "\""
                     << endl;
            }
            script_info[node_name] = node.value();
        }

        entity_common[nm] = script_info;
    }

    // Loop through each "entity" node
    for (auto script_node = runscript_node.first_node("entity"); script_node.is_valid();
         script_node = script_node.next_sibling("entity")) {
        std::map<std::string, std::string> script_info;

        auto nm_attr = script_node.first_attribute("entity_common");

        int autonomy_order = 0, controller_order = 0, sensor_order = 0;
        if (nm_attr.is_valid()) {
            std::string nm = nm_attr.value();
            auto it = entity_common.find(nm);
            if (it == entity_common.end()) {
                cout << "warning: entity_common block referenced without definition" << endl;
            } else {
                script_info = it->second;
                entity_attributes_[ent_desc_id] = entity_common_attributes[nm];
                autonomy_order = orders[nm]["autonomy"];
                controller_order = orders[nm]["controller"];
                sensor_order = orders[nm]["sensor"];
            }
        }

        // Find the entity's team ID first, since it is required by later
        // nodes. Also, setup the TeamInfo structs / map.
        auto team_id_node = script_node.first_node("team_id");
        if (team_id_node.is_valid()) {
            if (team_id_node.next_sibling("team_id").is_valid()) {
                cout << "Warning: entity contains multiple tags for \"team_id\"" << endl;
            }
            script_info["team_id"] = team_id_node.value();
        } else if (script_info.count("team_id") == 0) {
            cout << "Warning: Team ID not set" << endl;
            script_info["team_id"] = std::to_string(team_id_err--);
        }

        int team_id = parse_int(script_info["team_id"]);

        // Since we can handle multiple bases per team ID, we need to see if
        // a team ID has already been set. If not, create a new entry into
        // the map.
        if (team_info_.count(team_id) == 0) {
            // Team ID doesn't exist yet, create it.
            TeamInfo info;
            info.team_id = team_id;
            info.color.set_r(0);
            info.color.set_g(0);
            info.color.set_b(0);
            team_info_[team_id] = info;
        }

        // Loop through each "base" node
        for (auto base_node = script_node.first_node("base"); base_node.is_valid();
             base_node = base_node.next_sibling("base")) {
            double radius = 25;
            auto radius_node = base_node.first_node("radius");
            if (radius_node.is_valid()) {
                radius = parse_double(radius_node.value());
            }

            double opacity = 1.0;
            auto opacity_node = base_node.first_node("opacity");
            if (opacity_node.is_valid()) {
                opacity = parse_double(opacity_node.value());
            }

            // Extract position of base
            Eigen::Vector3d pos_xyz(-1, -1, -1);

            // Search for xyz
            auto x_node = base_node.first_node("x");
            if (x_node.is_valid()) {
                pos_xyz(0) = parse_double(x_node.value());
            }

            auto y_node = base_node.first_node("y");
            if (y_node.is_valid()) {
                pos_xyz(0) = parse_double(y_node.value());
            }

            auto z_node = base_node.first_node("z");
            if (z_node.is_valid()) {
                pos_xyz(0) = parse_double(z_node.value());
            }

            // Search for lon lat alt.
            Eigen::Vector3d pos_LLA(-1, -1, -1);
            bool lon_valid = false, lat_valid = false, alt_valid = false;
            auto lon_node = base_node.first_node("longitude");
            if (lon_node.is_valid()) {
                pos_LLA(0) = parse_double(lon_node.value());
                lon_valid = true;
            }

            auto lat_node = base_node.first_node("latitude");
            if (lat_node.is_valid()) {
                pos_LLA(1) = parse_double(lat_node.value());
                lat_valid = true;
            }

            auto alt_node = base_node.first_node("altitude");
            if (alt_node.is_valid()) {
                pos_LLA(2) = parse_double(alt_node.value());
                alt_valid = true;
            }

            // If lon/lat/alt are defined, overwrite x/y/z
            if (lon_valid && lat_valid && alt_valid) {
                proj_->Forward(
                    pos_LLA(1), pos_LLA(0), pos_LLA(2), pos_xyz(0), pos_xyz(1), pos_xyz(2));
            }
            team_info_[team_id].bases.push_back(pos_xyz);
            team_info_[team_id].radii.push_back(radius);
            team_info_[team_id].opacities.push_back(opacity);
        }

        // Loop through every other element under the "entity" node
        for (auto node = script_node.first_node(); node.is_valid(); node = node.next_sibling()) {
            std::string nm = node.name();
            if (nm == "autonomy") {
                nm += std::to_string(autonomy_order++);
            } else if (nm == "controller") {
                nm += std::to_string(controller_order++);
            } else if (nm == "sensor") {
                nm += std::to_string(sensor_order++);
            }

            if (script_info.count(nm) > 0 && nm.compare("team_id")) {
                cout << "Warning: entity contains multiple tags for \"" << nm << "\"" << endl;
            }
            script_info[nm] = node.value();

            // Loop through each node's attributes:
            for (auto attr = node.first_attribute(); attr.is_valid(); attr = attr.next()) {
                const std::string attr_name = attr.name();
                if (attr_name == "param_common") {
                    for (auto &kv : param_common[attr.value()]) {
                        entity_attributes_[ent_desc_id][nm][kv.first] = kv.second;
                    }
                } else {
                    entity_attributes_[ent_desc_id][nm][attr_name] = attr.value();
                }
            }
        }

        // For each entity, if the lat/lon are defined, use these values to
        // overwrite the "x" and "y" values
        // Search for lon lat alt.
        Eigen::Vector3d pos_LLA(-1, -1, -1);
        Eigen::Vector3d pos_xyz(-1, -1, -1);
        bool lon_valid = false, lat_valid = false, alt_valid = false;
        if (script_info.count("longitude") > 0) {
            pos_LLA(0) = parse_double(script_info["longitude"]);
            lon_valid = true;
        }
        if (script_info.count("latitude") > 0) {
            pos_LLA(1) = parse_double(script_info["latitude"]);
            lat_valid = true;
        }
        if (script_info.count("altitude") > 0) {
            pos_LLA(2) = parse_double(script_info["altitude"]);
            alt_valid = true;
        }

        // If lon/lat/alt are defined, overwrite x/y/z
        if (lon_valid && lat_valid && alt_valid) {
            proj_->Forward(pos_LLA(1), pos_LLA(0), pos_LLA(2), pos_xyz(0), pos_xyz(1), pos_xyz(2));

            script_info["x"] = std::to_string(pos_xyz(0));
            script_info["y"] = std::to_string(pos_xyz(1));
            script_info["z"] = std::to_string(pos_xyz(2));
        }

        // Save the initial positions (x,y,z) since we modify them for each
        // entity during generation
        if (script_info.count("x") > 0) {
            script_info["x0"] = script_info["x"];
        } else {
            cout << "Entity missing 'x' tag." << endl;
        }

        if (script_info.count("y") > 0) {
            script_info["y0"] = script_info["y"];
        } else {
            cout << "Entity missing 'y' tag." << endl;
        }

        if (script_info.count("z") > 0) {
            script_info["z0"] = script_info["z"];
        } else {
            cout << "Entity missing 'z' tag." << endl;
        }

        bool color_status = false;
        std::vector<int> temp_color;
        if (script_info.count("color") > 0) {
            color_status = str2container(script_info["color"], " ", temp_color, 3);
        }

        if (color_status) {
            team_info_[team_id].color.set_r(temp_color[0]);
            team_info_[team_id].color.set_g(temp_color[1]);
            team_info_[team_id].color.set_b(temp_color[2]);
        }

        // Save the count
        GenerateInfo gen_info;
        gen_info.total_count = 1;
        gen_info.gen_count = 1;
        gen_info.first_in_group = true;
        gen_info.start_time = t0_ - 1;
        gen_info.rate = -1;
        gen_info.time_variance = 0;
        if (script_info.count("count") > 0) {
            // If there are overrides in count, catch them and substitue
            gen_info.total_count = parse_int(script_info["count"]);
            gen_info.gen_count = gen_info.total_count;
        } else {
            script_info["count"] = std::to_string(gen_info.total_count);
        }

        // Is this entity using generate_rate?
        if (script_info.count("generate_rate") > 0 && script_info.count("generate_count") > 0) {
            // Tokenize rate based on "/"
            std::vector<double> values;
            bool status = str2container(script_info["generate_rate"], "/", values, 2);
            double rate = -1;
            if (status) {
                rate = values[0] / values[1];
            } else {
                rate = parse_double(script_info["generate_rate"]);
            }

            double start_time = 0;
            if (script_info.count("generate_start_time") > 0) {
                start_time = parse_double(script_info["generate_start_time"]);
            }

            if (script_info.count("generate_time_variance") > 0) {
                gen_info.time_variance = parse_double(script_info["generate_time_variance"]);
            }

            std::string gen_count_str = replace_overrides(script_info["generate_count"]);
            int gen_count = parse_int(gen_count_str);
            if (rate > 0 && gen_count > 0) {
                gen_info.start_time = start_time;
                gen_info.gen_count = gen_count;
                gen_info.rate = rate;

            } else if (rate > 0 && gen_count <= 0) {
                cout << "WARNING: Not using entity generator."
                     << "generate_rate defined, but generate_count is "
                     << "less than or equal to zero" << endl;
            }
        }

        std::vector<double> start_times;
        for (int i = 0; i < gen_info.gen_count; i++) {
            start_times.push_back(gen_info.start_time);
        }
        next_gen_times_[ent_desc_id] = start_times;
        gen_info_[ent_desc_id] = gen_info;

        // If the entity block has a "tag" attribute, save the mapping from the
        // tag to the entity block ID
        auto tag_attr = script_node.first_attribute("tag");
        if (tag_attr.is_valid()) {
            entity_tag_to_id_[tag_attr.value()] = ent_desc_id;
        }

        entity_descs_[ent_desc_id++] = script_info;
    }

    parse_terrain();

    // Parse the output_type
    auto it_output_type = params_.find("output_type");
    if (it_output_type != params_.end()) {
        output_types_ = str2container<std::set<std::string>>(it_output_type->second, ", ");
    }
    // If "all" is a possible output_type, include all output_types
    if (output_types_.find("all") != output_types_.end()) {
        output_types_ = possible_output_types_;
    }

    // Log the bins by default. Don't log the bins if the tag is defined in
    // the mission file and it is set to true.
    no_bin_logging_ =
        (params_.count("no_bin_logging") > 0 && str2bool(params_["no_bin_logging"]) == true);

    return true;
}

bool MissionParse::create_log_dir() {
    // Create the root_log_dir_ if it doesn't exist:
    if (not fs::exists(fs::path(root_log_dir_)) &&
        not fs::create_directories(fs::path(root_log_dir_))) {
        cout << "Failed to create the root_log_dir: " << root_log_dir_ << endl;
        return false;
    }

    bool log_dir_created = false;
    std::string log_dir_original = log_dir_;
    int attempts = 0;
    while (!log_dir_created && attempts < 1e4) {
        log_dir_ = log_dir_original;
        // If the log directory already exists, append a number to it:
        if (fs::exists(fs::path(log_dir_))) {
            int ct = 1;
            std::string log_dir_tmp;
            do {
                log_dir_tmp = log_dir_ + "_" + std::to_string(ct++);
            } while (fs::exists(fs::path(log_dir_tmp)));
            log_dir_ = log_dir_tmp;
        }

        if (fs::create_directories(fs::path(log_dir_))) {
            log_dir_created = true;
        }
        attempts++;
    }

    if (!log_dir_created) {
        cout << "Unable to create log directory: " << log_dir_ << endl;
        return false;
    }

    // Copy the input scenario xml file to the output directory
    if (fs::exists(mission_filename_)) {
        fs::copy_file(fs::path(mission_filename_), fs::path(log_dir_ + "/mission.orig.xml"));
    }

    // Write the parsed content to the output directory
    std::ofstream content_out(log_dir_ + "/mission.xml");
    content_out << mission_file_content_;
    content_out.close();

    // Create the latest log directory by default. Don't create the latest
    // directory if the tag is defined in the mission file and it is set to
    // false.
    bool create_latest_dir = not(params_.count("create_latest_dir") > 0 &&
                                 str2bool(params_["create_latest_dir"]) == false);

    if (create_latest_dir) {
        boost::system::error_code ec;
        auto print_error = [&]() {
            cout << "Error code value: " << ec.value() << endl;
            cout << "Error code name: " << ec.category().name() << endl;
            cout << "Error message: " << ec.message() << endl;
        };
        auto fs_err = [&]() { return ec != boost::system::errc::success; };

        // Create a "latest" symlink to the directory
        // First, remove the latest symlink if it exists
        fs::path latest_sym(root_log_dir_ + std::string("/latest"));
        if (fs::is_symlink(latest_sym)) {
            fs::remove(latest_sym, ec);
            if (fs_err()) {
                cout << "WARNING: could not remove symlink to latest directory" << endl;
                print_error();
            }
        }

        // Create the symlink
        fs::create_directory_symlink(fs::path(log_dir_), latest_sym, ec);
        if (fs_err()) {
            cout << "WARNING: Unable to create latest log file symlink" << endl;
            cout << "Couldn't create symlink log directory: " << latest_sym.string() << " -> "
                 << fs::path(log_dir_).string() << endl;
            print_error();
        }
    }
    return true;
}

bool MissionParse::write(const std::string &file) { return true; }

double MissionParse::t0() { return t0_; }

double MissionParse::tend() { return tend_; }

double MissionParse::dt() { return dt_; }

void MissionParse::set_dt(const double &dt) { dt_ = dt; }

double MissionParse::motion_multiplier() { return motion_multiplier_; }

double MissionParse::time_warp() { return time_warp_; }

bool MissionParse::start_paused() { return start_paused_; }

const bool &MissionParse::full_screen() { return full_screen_; }
const unsigned &MissionParse::window_width() { return window_width_; }
const unsigned &MissionParse::window_height() { return window_height_; }

bool MissionParse::parse_terrain() {
    ConfigParse terrain_parse;
    utm_terrain_ = std::make_shared<scrimmage_proto::UTMTerrain>();

    // Check for grid settings
    if (params_.count("grid_spacing") > 0) {
        utm_terrain_->set_grid_spacing(std::stod(params_["grid_spacing"]));
    } else {
        utm_terrain_->set_grid_spacing(100);
    }

    if (params_.count("grid_size") > 0) {
        utm_terrain_->set_grid_size(std::stod(params_["grid_size"]));
    } else {
        utm_terrain_->set_grid_size(10000);
    }

    // Check for origin settings
    if (params_.count("origin_length") > 0) {
        utm_terrain_->set_origin_length(std::stod(params_["origin_length"]));
    } else {
        utm_terrain_->set_origin_length(1);
    }

    if (params_.count("show_origin") > 0) {
        utm_terrain_->set_show_origin(str2bool(params_["show_origin"]));
    } else {
        utm_terrain_->set_show_origin(false);
    }

    set(utm_terrain_->mutable_background(), background_color_);

    utm_terrain_->set_enable_grid(true);
    utm_terrain_->set_enable_terrain(false);
    utm_terrain_->set_enable_extrusion(false);

    if (params_.count("terrain") > 0 &&
        find_terrain_files(params_["terrain"], terrain_parse, utm_terrain_)) {
        double x_easting;
        double y_northing;
        int zone;
        bool northp;
        double gamma, k_scale;
        GeographicLib::UTMUPS::Forward(latitude_origin(),
                                       longitude_origin(),
                                       zone,
                                       northp,
                                       x_easting,
                                       y_northing,
                                       gamma,
                                       k_scale);

        // Make sure the projected zone and hemisphere match the input xml
        if (((northp && boost::to_upper_copy(terrain_parse.params()["hemisphere"]) == "NORTH") ||
             (!northp && boost::to_upper_copy(terrain_parse.params()["hemisphere"]) == "SOUTH")) &&
            zone == get("zone", terrain_parse.params(), -2)) {
            double origin_offset_x = 0;
            double origin_offset_y = 0;
            double origin_offset_z = 0;

            if (terrain_parse.params().count("origin_offset_x") == 1 &&
                terrain_parse.params().count("origin_offset_y") == 1 &&
                terrain_parse.params().count("origin_offset_z") == 1) {
                origin_offset_x = stod(terrain_parse.params()["origin_offset_x"]);
                origin_offset_y = stod(terrain_parse.params()["origin_offset_y"]);
                origin_offset_z = stod(terrain_parse.params()["origin_offset_z"]);
            }

            utm_terrain_->set_x_translate(x_easting - origin_offset_x);
            utm_terrain_->set_y_translate(y_northing - origin_offset_y);
            utm_terrain_->set_z_translate(altitude_origin() - origin_offset_z);
            utm_terrain_->set_z_translate(altitude_origin());
            utm_terrain_->set_system(terrain_parse.params()["system"]);
            utm_terrain_->set_zone(zone);
            utm_terrain_->set_hemisphere(terrain_parse.params()["hemisphere"]);
            utm_terrain_->set_enable_grid(
                get<bool>("enable_grid", terrain_parse.params(), "false"));
            utm_terrain_->set_enable_terrain(true);
            return true;

        } else {
            cout << "============================================" << endl;
            cout << "Invalid XML Terrain settings: " << endl;
            cout << "Hemisphere: " << terrain_parse.params()["hemisphere"] << endl;
            cout << "Zone: " << get("zone", terrain_parse.params(), -2) << endl;
            cout << "--------------------------------------------" << endl;
            cout << "Geographic lib output: " << endl;
            cout << "x_easting: " << x_easting << endl;
            cout << "y_northing: " << y_northing << endl;
            cout << "Zone: " << zone << endl;
            cout << "Northern hemisphere?: " << northp << endl;
        }
    }
    return false;
}

scrimmage_proto::Color &MissionParse::background_color() { return background_color_; }

std::string MissionParse::log_dir() { return log_dir_; }
std::string MissionParse::root_log_dir() { return root_log_dir_; }

void MissionParse::set_log_dir(const std::string &log_dir) { log_dir_ = log_dir; }

std::map<int, AttributeMap> &MissionParse::entity_attributes() { return entity_attributes_; }

std::map<int, std::map<std::string, std::string>> &MissionParse::entity_params() {
    return entity_params_;
}

std::map<int, int> &MissionParse::ent_id_to_block_id() { return ent_id_to_block_id_; }

EntityDesc_t &MissionParse::entity_descriptions() { return entity_descs_; }

std::map<std::string, int> &MissionParse::entity_tag_to_id() { return entity_tag_to_id_; }

bool MissionParse::enable_gui() { return enable_gui_; }

bool MissionParse::network_gui() { return network_gui_; }

AttributeMap &MissionParse::attributes() { return attributes_; }

std::map<std::string, std::string> &MissionParse::params() { return params_; }

double MissionParse::longitude_origin() { return longitude_origin_; }

double MissionParse::latitude_origin() { return latitude_origin_; }

double MissionParse::altitude_origin() { return altitude_origin_; }

void MissionParse::set_lat_lon_alt_origin(const double &latitude_origin,
                                          const double &longitude_origin,
                                          const double &altitude_origin) {
    latitude_origin_ = latitude_origin;
    longitude_origin_ = longitude_origin;
    altitude_origin_ = altitude_origin;

    proj_ = std::make_shared<GeographicLib::LocalCartesian>(
        latitude_origin_, longitude_origin_, altitude_origin_, GeographicLib::Geocentric::WGS84());
}

std::map<int, TeamInfo> &MissionParse::team_info() { return team_info_; }

void MissionParse::set_task_number(int task_num) { task_number_ = task_num; }

void MissionParse::set_job_number(int job_num) { job_number_ = job_num; }

std::list<std::string> MissionParse::entity_interactions() { return entity_interactions_; }

std::list<std::string> &MissionParse::network_names() { return network_names_; }

std::list<std::string> MissionParse::metrics() { return metrics_; }

std::map<int, GenerateInfo> &MissionParse::gen_info() { return gen_info_; }

std::map<int, std::vector<double>> &MissionParse::next_gen_times() { return next_gen_times_; }

std::shared_ptr<GeographicLib::LocalCartesian> MissionParse::projection() { return proj_; }

std::shared_ptr<scrimmage_proto::UTMTerrain> &MissionParse::utm_terrain() { return utm_terrain_; }

std::string MissionParse::get_mission_filename() { return mission_filename_; }

bool MissionParse::get_no_bin_logging() { return no_bin_logging_; }

void MissionParse::set_enable_gui(bool enable) { enable_gui_ = enable; }

void MissionParse::set_time_warp(double warp) { time_warp_ = warp; }

void MissionParse::set_network_gui(bool enable) { network_gui_ = enable; }

void MissionParse::set_start_paused(bool paused) { start_paused_ = paused; }

bool MissionParse::output_required() {
    // If the output_types set is empty, no output is required
    return output_types_.size() != static_cast<unsigned int>(0);
}

bool MissionParse::output_type_required(const std::string &output_type) {
    return output_types_.find(output_type) != output_types_.end();
}

}  // namespace scrimmage
