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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <boost/algorithm/string.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <rapidxml/rapidxml.hpp>

using std::cout;
using std::endl;

namespace fs = boost::filesystem;

namespace scrimmage {

bool compare_order(const std::pair<int, std::string> & first,
                   const std::pair<int, std::string> & second) {
    if (first.first < second.first) return true;
    return false;
}

bool MissionParse::parse(std::string filename) {
    mission_filename_ = filename;

    rapidxml::xml_document<> doc;
    std::ifstream file(mission_filename_.c_str());

    if (!file.is_open()) {
        cout << filename << " not found" << endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    std::string content(buffer.str());
    doc.parse<0>(&content[0]);

    rapidxml::xml_node<> *runscript_node = doc.first_node("runscript");
    if (runscript_node == 0) {
        cout << "Missing runscript tag." << endl;
        return false;
    }

    rapidxml::xml_node<> *run_node = runscript_node->first_node("run");
    if (run_node == 0) {
        cout << "Missing run node" << endl;
        return false;
    }

    motion_multiplier_ = 1;
    for (rapidxml::xml_attribute<> *attr = run_node->first_attribute();
         attr; attr = attr->next_attribute()) {

        std::string attr_str(attr->name());
        if (attr_str == "start") {
            t0_ = std::stod(attr->value());
        } else if (attr_str == "end") {
            tend_ = std::stod(attr->value());
        } else if (attr_str == "dt") {
            dt_ = std::stod(attr->value());
        } else if (attr_str ==  "motion_multiplier") {
            motion_multiplier_ = std::stod(attr->value());
        } else if (attr_str == "time_warp") {
            time_warp_ = std::stod(attr->value());
        } else if (attr_str == "enable_gui") {
            enable_gui_ = str2bool(attr->value());
        } else if (attr_str == "network_gui") {
            network_gui_ = str2bool(attr->value());
        } else if (attr_str == "start_paused") {
            start_paused_ = str2bool(attr->value());
        }
    }

    // Loop through each "entity_interaction" node
    // Put in an ordered list based on the "order" attribute
    std::list<std::pair<int, std::string> > ent_inters;
    for (rapidxml::xml_node<> *node = runscript_node->first_node("entity_interaction");
         node != 0; node = node->next_sibling("entity_interaction")) {

        std::string name(node->value());

        int order = std::numeric_limits<int>::max();
        rapidxml::xml_attribute<> *attr = node->first_attribute("order");
        if (attr != NULL) {
            order = std::stoi(attr->value());
        } else {
            cout << "entity_interaction missing 'order' attribute: "
                 << node->value() << endl;
        }
        ent_inters.push_back(std::pair<int, std::string>(order, name));
    }
    ent_inters.sort(compare_order); // sort based on "order"

    // Copy the sorted "entity_interaction" plugin names into a list
    for (std::pair<int, std::string> p : ent_inters) {
        entity_interactions_.push_back(p.second);
    }

    // Loop through each "metrics" node
    // Put in an ordered list based on the "order" attribute
    std::list<std::pair<int, std::string> > metrics;
    for (rapidxml::xml_node<> *node = runscript_node->first_node("metrics");
         node != 0; node = node->next_sibling("metrics")) {

        std::string name(node->value());

        int order = std::numeric_limits<int>::max();
        rapidxml::xml_attribute<> *attr = node->first_attribute("order");
        if (attr != NULL) {
            order = std::stoi(attr->value());
        } else {
            cout << "metrics missing 'order' attribute: "
                 << node->value() << endl;
        }
        metrics.push_back(std::pair<int, std::string>(order, name));
    }
    metrics.sort(compare_order); // sort based on "order"

    // Copy the sorted "metrics" plugin names into a list
    for (std::pair<int, std::string> p : metrics) {
        metrics_.push_back(p.second);
    }

    // Loop through each node under "runscript" that isn't an entity or base
    attributes_.clear();
    for (rapidxml::xml_node<> *node = runscript_node->first_node(); node != 0;
         node = node->next_sibling()) {

        std::string nm = node->name();
        if (nm != "entity" && nm != "base"  && nm != "entity_common") {
            params_[nm] = node->value();

            // Loop through each node's attributes:
            for (rapidxml::xml_attribute<> *attr = node->first_attribute();
                 attr; attr = attr->next_attribute()) {

                std::string nm2 = nm == "entity_interaction" ? node->value() : nm;
                attributes_[nm2][attr->name()] = attr->value();
            }
        }
    }

    // Save background color:
    bool bg_color_result = false;
    std::vector<int> temp_color;
    if (params_.count("background_color") > 0) {
        bg_color_result = str2vec<int>(params_["background_color"], " ", temp_color, 3);
    }
    if (bg_color_result) {
        background_color_.set_r(temp_color[0]);
        background_color_.set_g(temp_color[1]);
        background_color_.set_b(temp_color[2]);
    }

    // Is latitude_origin defined?
    if (params_.count("latitude_origin") > 0) {
        latitude_origin_ = std::stod(params_["latitude_origin"]);
    }

    // Is longitude_origin defined?
    if (params_.count("longitude_origin") > 0) {
        longitude_origin_ = std::stod(params_["longitude_origin"]);
    }

    // Is altitude_origin defined?
    if (params_.count("altitude_origin") > 0) {
        altitude_origin_ = std::stod(params_["altitude_origin"]);
    }

    proj_ =
      std::make_shared<GeographicLib::LocalCartesian>(latitude_origin_,
        longitude_origin_, altitude_origin_, GeographicLib::Geocentric::WGS84());

    // Handle log directory
    log_dir_ = expand_user("~/.scrimmage/logs");
    if (params_.count("log_dir") > 0) {
        // Get the dir attribute of the log node
        log_dir_ = expand_user(params_["log_dir"]);
    }

    // Create a directory to hold the log data
    // Use the current time for the directory's name
    time_t rawtime;
    struct tm * timeinfo;
    char time_buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);
    std::string name(time_buffer);
    int ct = 1;
    while (fs::exists(fs::path(log_dir_ + "/" + name))) {
        name = std::string(time_buffer) + "_" + std::to_string(ct++);
    }
    log_dir_ += "/" + name;

    if (job_number_ != -1) {
        log_dir_ += "_job_" + std::to_string(job_number_);
    }
    if (task_number_ != -1) {
        log_dir_ += "_task_" + std::to_string(task_number_);
    }

    int ent_desc_id = 0;
    int team_id_err = 0; // only used when team_id is not set "warn condition"

    // common block name -> <node_name, value>
    std::map<std::string, AttributeMap> entity_common_attributes;
    std::map<std::string, std::map<std::string, std::string>> entity_common;
    for (rapidxml::xml_node<> *script_node = runscript_node->first_node("entity_common");
         script_node != 0;
         script_node = script_node->next_sibling("entity_common")) {

        std::map<std::string, std::string> script_info;

        rapidxml::xml_attribute<> *nm_attr = script_node->first_attribute("name");

        if (nm_attr == 0) {
            cout << "warning: found entity_common block without a name, skipping" << endl;
            continue;
        }

        std::string nm = nm_attr->value();

        for (rapidxml::xml_node<> *node = script_node->first_node(); node != 0;
             node = node->next_sibling()) {
            std::string node_name = node->name();

            if (node_name == "name") {
                continue;
            }

            rapidxml::xml_attribute<> *order_attr = node->first_attribute("order");
            if (order_attr) {
                node_name += order_attr->value();
            } else if (node_name == "controller" || node_name == "sensor" ||
                       node_name == "sensable" || node_name == "autonomy") {
                node_name += "0";
            }

            // Loop through each node's attributes:
            for (rapidxml::xml_attribute<> *attr = node->first_attribute();
                 attr; attr = attr->next_attribute()) {

                entity_common_attributes[nm][node_name][attr->name()] = attr->value();
            }

            script_info[node_name] = node->value();
        }

        entity_common[nm] = script_info;
    }

    // Loop through each "entity" node
    for (rapidxml::xml_node<> *script_node = runscript_node->first_node("entity");
         script_node != 0;
         script_node = script_node->next_sibling("entity")) {

        std::map<std::string, std::string> script_info;

        rapidxml::xml_attribute<> *nm_attr = script_node->first_attribute("entity_common");

        if (nm_attr != 0) {
            std::string nm = nm_attr->value();
            auto it = entity_common.find(nm);
            if (it == entity_common.end()) {
                cout << "warning: entity_common block referenced without definition" << endl;
            } else {
                script_info = it->second;
                entity_attributes_[ent_desc_id] = entity_common_attributes[nm];
            }
        }

        script_info["log_dir"] = log_dir_;


        // Find the entity's team ID first, since it is required by later
        // nodes. Also, setup the TeamInfo structs / map.
        rapidxml::xml_node<> *team_id_node = script_node->first_node("team_id");
        if (team_id_node != 0) {
            script_info["team_id"] = team_id_node->value();
        } else if (script_info.count("team_id") == 0) {
            cout << "Warning: Team ID not set" << endl;
            script_info["team_id"] = std::to_string(team_id_err--);
        }

        int team_id = std::stoi(script_info["team_id"]);

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
        for (rapidxml::xml_node<> *base_node = script_node->first_node("base");
             base_node != 0;
             base_node = base_node->next_sibling("base")) {

            double radius = 25;
            rapidxml::xml_node<> *radius_node = base_node->first_node("radius");
            if (radius_node != 0) {
                radius = std::stod(radius_node->value());
            }

            double opacity = 1.0;
            rapidxml::xml_node<> *opacity_node = base_node->first_node("opacity");
            if (opacity_node != 0) {
                opacity = std::stod(opacity_node->value());
            }

            // Extract position of base
            Eigen::Vector3d pos_xyz(-1, -1, -1);

            // Search for xyz
            rapidxml::xml_node<> *x_node = base_node->first_node("x");
            if (x_node != 0) {
                pos_xyz(0) = std::stod(x_node->value());
            }
            rapidxml::xml_node<> *y_node = base_node->first_node("y");
            if (y_node != 0) {
                pos_xyz(1) = std::stod(y_node->value());
            }
            rapidxml::xml_node<> *z_node = base_node->first_node("z");
            if (z_node != 0) {
                pos_xyz(2) = std::stod(z_node->value());
            }

            // Search for lon lat alt.
            Eigen::Vector3d pos_LLA(-1, -1, -1);
            bool lon_valid = false, lat_valid = false, alt_valid = false;
            rapidxml::xml_node<> *lon_node = base_node->first_node("longitude");
            if (lon_node != 0) {
                pos_LLA(0) = std::stod(lon_node->value());
                lon_valid = true;
            }
            rapidxml::xml_node<> *lat_node = base_node->first_node("latitude");
            if (lat_node != 0) {
                pos_LLA(1) = std::stod(lat_node->value());
                lat_valid = true;
            }
            rapidxml::xml_node<> *alt_node = base_node->first_node("altitude");
            if (alt_node != 0) {
                pos_LLA(2) = std::stod(alt_node->value());
                alt_valid = true;
            }

            // If lon/lat/alt are defined, overwrite x/y/z
            if (lon_valid && lat_valid && alt_valid) {
                proj_->Forward(pos_LLA(1), pos_LLA(0), pos_LLA(2),
                               pos_xyz(0), pos_xyz(1), pos_xyz(2));
            }
            team_info_[team_id].bases.push_back(pos_xyz);
            team_info_[team_id].radii.push_back(radius);
            team_info_[team_id].opacities.push_back(opacity);
        }

        // Loop through every other element under the "entity" node
        for (rapidxml::xml_node<> *node = script_node->first_node(); node != 0;
             node = node->next_sibling()) {

            std::string nm = node->name();
            rapidxml::xml_attribute<> *order_attr = node->first_attribute("order");
            if (order_attr) {
                nm += order_attr->value();
            } else if (nm == "controller" || nm == "sensor" || nm == "sensable"
                       || nm == "autonomy") {
                nm += "0";
            }

            script_info[nm] = node->value();

            // Loop through each node's attributes:
            for (rapidxml::xml_attribute<> *attr = node->first_attribute();
                 attr; attr = attr->next_attribute()) {

                entity_attributes_[ent_desc_id][nm][attr->name()] = attr->value();
            }
        }

        // For each entity, if the lat/lon are defined, use these values to
        // overwrite the "x" and "y" values
        // Search for lon lat alt.
        Eigen::Vector3d pos_LLA(-1, -1, -1);
        Eigen::Vector3d pos_xyz(-1, -1, -1);
        bool lon_valid = false, lat_valid = false, alt_valid = false;
        if (script_info.count("longitude") > 0) {
            pos_LLA(0) = std::stod(script_info["longitude"]);
            lon_valid = true;
        }
        if (script_info.count("latitude") > 0) {
            pos_LLA(1) = std::stod(script_info["latitude"]);
            lat_valid = true;
        }
        if (script_info.count("altitude") > 0) {
            pos_LLA(2) = std::stod(script_info["altitude"]);
            alt_valid = true;
        }

        // If lon/lat/alt are defined, overwrite x/y/z
        if (lon_valid && lat_valid && alt_valid) {
            proj_->Forward(pos_LLA(1), pos_LLA(0), pos_LLA(2),
                           pos_xyz(0), pos_xyz(1), pos_xyz(2));

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
            color_status = str2vec(script_info["color"], " ", temp_color, 3);
        }

        if (!color_status) {
            cout << "Error parsing entity color:" <<  script_info["color"] << endl;
        } else {
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
            gen_info.total_count = std::stoi(script_info["count"]);
            gen_info.gen_count = gen_info.total_count;
        } else {
            script_info["count"] = std::to_string(gen_info.total_count);
        }

        // Is this entity using generate_rate?
        if (script_info.count("generate_rate") > 0 &&
            script_info.count("generate_count") > 0) {

            // Tokenize rate based on "/"
            std::vector<double> values;
            bool status = str2vec(script_info["generate_rate"], "/",
                                  values, 2);
            double rate = -1;
            if (status) {
                rate = values[0] / values[1];
            } else {
                rate = std::stod(script_info["generate_rate"]);
            }

            double start_time = 0;
            if (script_info.count("generate_start_time") > 0) {
                start_time = std::stod(script_info["generate_start_time"]);
            }

            if (script_info.count("generate_time_variance") > 0) {
                gen_info.time_variance = std::stod(script_info["generate_time_variance"]);
            }

            int gen_count = std::stoi(script_info["generate_count"]);
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

        entity_descs_[ent_desc_id++] = script_info;
    }

    parse_terrain();

    return true;
}

bool MissionParse::create_log_dir() {
    // Create the log directory
    if (!fs::exists(log_dir_)) {
        if (!fs::create_directories(fs::path(log_dir_))) {
            cout << "ERROR: Unable to create output directory: "
                 << log_dir_ << endl;
            return false;
        }
    }

    // Copy the input scenario xml file to the output directory
    fs::copy_file(fs::path(mission_filename_), fs::path(log_dir_+"/mission.xml"));

    return true;
}

bool MissionParse::write(std::string file) {
    return true;
}

double MissionParse::t0() { return t0_; }

double MissionParse::tend() { return tend_; }

double MissionParse::dt() { return dt_; }

double MissionParse::motion_multiplier() { return motion_multiplier_; }

double MissionParse::time_warp() { return time_warp_; }

bool MissionParse::start_paused() { return start_paused_; }

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

    if (params_.count("terrain") > 0 &&
        find_terrain_files(params_["terrain"], terrain_parse, utm_terrain_)) {

        double x_easting;
        double y_northing;
        int zone;
        bool northp;
        double gamma, k_scale;
        GeographicLib::UTMUPS::Forward(latitude_origin(),
                                       longitude_origin(),
                                       zone, northp, x_easting, y_northing,
                                       gamma, k_scale);

        // Make sure the projected zone and hemisphere match the input xml
        if (((northp && boost::to_upper_copy(terrain_parse.params()["hemisphere"]) == "NORTH") ||
             (!northp && boost::to_upper_copy(terrain_parse.params()["hemisphere"]) == "SOUTH")) &&
            zone == get("zone", terrain_parse.params(), -2)) {

            utm_terrain_->set_x_translate(x_easting);
            utm_terrain_->set_y_translate(y_northing);
            utm_terrain_->set_z_translate(altitude_origin());
            utm_terrain_->set_system(terrain_parse.params()["system"]);
            utm_terrain_->set_zone(zone);
            utm_terrain_->set_hemisphere(terrain_parse.params()["hemisphere"]);
            utm_terrain_->set_enable_grid(get<bool>("enable_grid", terrain_parse.params(), "false"));
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

scrimmage_proto::Color &MissionParse::background_color()
{ return background_color_; }

std::string MissionParse::log_dir() { return log_dir_; }

void MissionParse::set_log_dir(const std::string &log_dir) {log_dir_ = log_dir;}

std::map<int, AttributeMap> &MissionParse::entity_attributes() { return entity_attributes_; }

EntityDesc_t &MissionParse::entity_descriptions() { return entity_descs_; }

bool MissionParse::enable_gui() { return enable_gui_; }

bool MissionParse::network_gui() { return network_gui_; }

AttributeMap &MissionParse::attributes() { return attributes_; }

std::map<std::string, std::string> &MissionParse::params() { return params_; }

double MissionParse::longitude_origin() { return longitude_origin_; }

double MissionParse::latitude_origin() { return latitude_origin_; }

double MissionParse::altitude_origin() { return altitude_origin_; }

std::map<int, TeamInfo> &MissionParse::team_info() { return team_info_; }

void MissionParse::set_task_number(int task_num) { task_number_ = task_num; }

void MissionParse::set_job_number(int job_num) { job_number_ = job_num; }

std::list<std::string> MissionParse::entity_interactions()
{ return entity_interactions_; }

std::list<std::string> MissionParse::metrics()
{ return metrics_; }

std::map<int, GenerateInfo> &MissionParse::gen_info()
{ return gen_info_; }

std::map<int, std::vector<double> > &MissionParse::next_gen_times()
{ return next_gen_times_; }

std::shared_ptr<GeographicLib::LocalCartesian> MissionParse::projection()
{ return proj_; }

std::shared_ptr<scrimmage_proto::UTMTerrain> &MissionParse::utm_terrain()
{ return utm_terrain_; }

} // namespace scrimmage
