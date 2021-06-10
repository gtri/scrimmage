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
#include <scrimmage/common/PID.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/proto/Visual.pb.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace fs = boost::filesystem;
using std::cout;
using std::endl;

namespace sc = scrimmage;

namespace scrimmage {

bool find_terrain_files(std::string terrain_name,
                        ConfigParse &terrain_parse,
                        std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain) {
    if (utm_terrain == nullptr) {
        cout << "utm_terrain is null." << endl;
        return false;
    }

    utm_terrain->set_terrain_name(terrain_name);

    // Parse the motion model xml file
    terrain_parse.set_required("polydata");
    terrain_parse.set_required("texture");
    terrain_parse.set_required("system");
    terrain_parse.set_required("hemisphere");
    terrain_parse.set_required("zone");

    FileSearch file_search;
    std::map<std::string, std::string> overrides; // empty, no overrides
    if (terrain_parse.parse(overrides, terrain_name, "SCRIMMAGE_DATA_PATH", file_search)) {
        std::string polydata_file = terrain_parse.directory() + "/" + terrain_parse.params()["polydata"];
        std::string texture_file = terrain_parse.directory() + "/" + terrain_parse.params()["texture"];
        std::string extrusion_file = terrain_parse.directory() + "/" + terrain_parse.params()["extrusion"];
        std::string extrusion_property = terrain_parse.params()["extrusion_property"];
        if (fs::exists(extrusion_file) && fs::is_regular_file(extrusion_file) &&
                !extrusion_property.empty()) {
            utm_terrain->set_extrusion_file(extrusion_file);
            utm_terrain->set_extrusion_property(extrusion_property);
            utm_terrain->set_enable_extrusion(true);
        }
        if (fs::exists(polydata_file) && fs::exists(texture_file) &&
            fs::is_regular_file(polydata_file) && fs::is_regular_file(texture_file)) {
            utm_terrain->set_texture_file(texture_file);
            utm_terrain->set_poly_data_file(polydata_file);
            return true;
        }
    }
    return false;
}

bool find_model_properties(std::string model_name,
                           ConfigParse &cv_parse,
                           FileSearch &file_search,
                           std::map<std::string, std::string> &overrides,
                           std::shared_ptr<scrimmage_proto::ContactVisual> &cv,
                           bool &mesh_found, bool &texture_found) {
    if (cv == nullptr) {
        cout << "Contact Visual is null." << endl;
        return false;
    }

    mesh_found = texture_found = false;

    cv->set_name(model_name);

    cv_parse.set_required("model");

    if (cv_parse.parse(overrides, model_name, "SCRIMMAGE_DATA_PATH", file_search)) {
        std::string model_file = cv_parse.directory() + "/" + cv_parse.params()["model"];
        std::string texture_file = cv_parse.directory() + "/" + cv_parse.params()["texture"];

        if (fs::exists(model_file) && fs::is_regular_file(model_file)) {
            cv->set_model_file(model_file);
            mesh_found = true;
        }
        if (fs::exists(texture_file) && fs::is_regular_file(texture_file)) {
            cv->set_texture_file(texture_file);
            texture_found = true;
        }
    }

    auto it_info = cv_parse.params().find("visual_scale");
    if (it_info != cv_parse.params().end()) {
        cv->set_scale(std::stod(it_info->second));
    } else {
        cv->set_scale(1.0);
    }

    bool valid_rotate = false;
    it_info = cv_parse.params().find("visual_rpy");
    if (it_info != cv_parse.params().end()) {
        std::vector<double> temp_rotate;
        if (str2container(it_info->second, " ", temp_rotate, 3)) {
            valid_rotate = true;
            for (auto it = temp_rotate.begin(); it != temp_rotate.end(); it++) {
                cv->add_rotate(*it);
            }
        } else {
            cout << "Error parsing: visual_rpy" << endl;
        }
    }
    if (!valid_rotate) {
        for (int i = 0; i < 3; i++) {
            cv->add_rotate(0);
        }
    }
    return (mesh_found && texture_found);
}

bool parse_autonomy_data(std::map<std::string, std::string> &params,
                         std::map<std::string, std::string> &data_params) {
    if (params.count("data") == 0) {
        cout << "Missing data tag in params" << endl;
        return false;
    }

    ConfigParse data;
    FileSearch file_search;
    std::map<std::string, std::string> overrides; // empty, no overrides
    if (data.parse(overrides, params["data"], "SCRIMMAGE_DATA_PATH",
                   file_search)) {
        data_params = data.params();
    } else {
        cout << "Failed to find data file: " << params["data"] << endl;
        return false;
    }
    return true;
}

std::string expand_user(std::string path) {
    if (not path.empty() and path[0] == '~') {
        assert(path.size() == 1 or path[1] == '/');  // or other error handling
        char const* home = std::getenv("HOME");
        if (home or
            (home = std::getenv("USERPROFILE"))) {
            path.replace(0, 1, home);
        } else {
            char const *hdrive = std::getenv("HOMEDRIVE"),
                *hpath = std::getenv("HOMEPATH");
            assert(hdrive);  // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}

bool str2bool(std::string str) {
    // Remove spaces
    str.erase(std::remove_if(str.begin(), str.end(), ::isspace),
              str.end());

    if (boost::to_upper_copy(str) == "TRUE") {
        return true;
    }

    // Maybe it's a "1" or "0"
    int num;
    if (!(std::istringstream(str) >> num)) {
        num = 0;
    }
    return (num == 1);
}

bool get_vec(const std::string &str,
             std::map<std::string, std::string> & params,
             std::vector<std::string> &vec) {
    vec.clear();

    // Determine if the string name is a list
    int count = get(str+":size", params, 0);
    if (count <= 0) return false;

    for (int i = 0; i < count; i++) {
        std::string name = str;
        if (i > 0) {
            name += "_" + std::to_string(i);
        }
        if (params.count(name) == 0) {
            cout << "Invalid name in params: " << str << endl;
            return false;
        }
        vec.push_back(params[name]);
    }
    return true;
}

Eigen::Vector3d vec2eigen(std::vector<double> &vec) {
    Eigen::Vector3d v(0, 0, 0);
    if (vec.size() <= 3) {
        for (unsigned int i = 0; i < vec.size(); i++) {
            v(i) = vec[i];
        }
    }
    return v;
}

std::string remove_whitespace(const std::string &str) {
    std::string result = str;
    result.erase(
        std::remove_if(result.begin(),
                       result.end(),
                       [](unsigned char x){return std::isspace(x);}),
        result.end());
    return result;
}

bool get_vec_of_vecs(const std::string &str,
                     std::vector<std::vector<std::string>> &out,
                     const std::string &delims) {
    // Of the form...
    // [a b c d]
    // [e f g h]
    // [h i j k]
    out.clear();

    std::string str_copy(str);
    remove_leading_spaces(str_copy);
    remove_trailing_spaces(str_copy);

    // First split based on user defined delimiters (typically ", ")
    std::vector<std::string> tokens;
    boost::algorithm::split(tokens, str_copy, boost::is_any_of(delims),
                            boost::token_compress_on);

    // The tokens vector consists of the delimited items. Break up each vector
    // based on the start "[" and stop "]" delimiters. However, we will allow
    // one-level of nested start and stop delimiters.
    std::vector<std::string> items;
    int open_bracket_count = 0;
    std::string item = "";
    for (std::string token : tokens) {

        // Remove whitespace
        token.erase(
            std::remove_if(token.begin(),
                           token.end(),
                           [](unsigned char x){return std::isspace(x);}),
            token.end());

        // cout << token << endl;

        for (unsigned int i = 0; i < token.size(); i++) {
            if (token[i] == '[') {
                if (open_bracket_count > 0) {
                    item += "[";
                }
                ++open_bracket_count;
            } else if (token[i] == ']') {
                if (open_bracket_count > 1) {
                    item += "]";
                }
                --open_bracket_count;

                if (open_bracket_count == 0) {
                    items.push_back(item);
                    out.push_back(items);
                    items.clear();
                    item = "";
                }
            } else if (open_bracket_count == 2) {
                item += std::string(1, token[i]);
            } else if (open_bracket_count > 0) {
                item += std::string(1, token[i]);
            }
        }

        if (open_bracket_count == 1) {
            items.push_back(item);
            item.clear();
        } else if (open_bracket_count == 0) {
            out.push_back(items);
            item.clear();
            items.clear();
        }
    }

    // Remove any empty strings from the individual vectors
    for (auto &vec : out) {
        vec.erase(std::remove_if(vec.begin(), vec.end(), [] (auto &str) {
            return str.empty();
        }), vec.end());
    }

    // Remove any empty vectors from the out vector of vectors
    out.erase(std::remove_if(out.begin(), out.end(), [] (auto &vec) {
        return vec.empty();
    }), out.end());

    return true;
}

void split(std::vector<std::string> &tokens, const std::string &str,
           const std::string &delims) {
    boost::split(tokens, str, boost::is_any_of(delims));
}

bool set_pid_gains(sc::PID &pid, std::string str, bool is_angle) {
    std::vector<std::string> str_vals;
    boost::split(str_vals, str, boost::is_any_of(","));

    if (str_vals.size() != 4) {
        return false;
    } else {
        double p = std::stod(str_vals[0]);
        double i = std::stod(str_vals[1]);
        double d = std::stod(str_vals[2]);
        pid.set_parameters(p, i, d);

        if (is_angle) {
            double i_lim = sc::Angles::deg2rad(std::stod(str_vals[3]));
            pid.set_integral_band(i_lim);
            pid.set_is_angle(true);
        } else {
            double i_lim = std::stod(str_vals[3]);
            pid.set_integral_band(i_lim);
        }
    }
    return true;
}

unsigned int parse_plugin_vector(const std::string& key,
                                 std::map<std::string, std::string> &params,
                                 std::list<PluginOverrides> &plugin_overrides_list) {
    // Parse the behavior plugins
    std::string plugins_str = sc::get<std::string>(key, params, "");
    std::vector<std::vector<std::string>> vecs_of_vecs;
    if (not sc::get_vec_of_vecs(plugins_str, vecs_of_vecs, " ")) {
        cout << "Failed to parse vector of vectors" << endl;
        return 0;
    }
    for (std::vector<std::string> vecs : vecs_of_vecs) {
        if (vecs.size() < 1) {
            std::cout << "Plugin name missing." << std::endl;
            continue;
        }

        std::string plugin_name = "";
        std::map<std::string, std::string> plugin_overrides;
        int i = 0;
        for (std::string str : vecs) {
            if (i == 0) {
                plugin_name = str;
            } else {
                // Parse the plugin parameters (e.g., param_name="value")
                // Split the param_name and value, with equals sign in betweenn
                std::vector<std::string> tokens;
                boost::split(tokens, str, boost::is_any_of("="));
                if (tokens.size() == 2) {
                    // Remove the quotes from the value
                    tokens[1].erase(
                        std::remove_if(tokens[1].begin(),
                                       tokens[1].end(),
                                       [](unsigned char x){
                                           return (x == '\"') || (x == '\'');
                                       }),
                        tokens[1].end());
                    plugin_overrides[tokens[0]] = tokens[1];
                }
            }
            ++i;
        }
        plugin_overrides_list.push_back(PluginOverrides(plugin_name, plugin_overrides));
    }

    // for (auto &plugin_override : plugin_overrides_list) {
    //     cout << "Plugin name: " << plugin_override.name << endl;
    //     for (auto &kv : plugin_override.overrides) {
    //         cout << "\t" << kv.first << "=" << kv.second << endl;
    //     }
    // }

    return plugin_overrides_list.size();
}

void remove_leading_spaces(std::string &s) {
    auto it = std::find_if(s.begin(), s.end(),
                           [](char c) {
                               return !std::isspace<char>(c, std::locale::classic());
                           });
	s.erase(s.begin(), it);
}

void remove_trailing_spaces(std::string &s) {
    auto it = std::find_if(s.rbegin(), s.rend(),
                           [](char c) {
                               return !std::isspace<char>(c, std::locale::classic());
                           });
	s.erase(it.base(), s.end());
}

} // namespace scrimmage
