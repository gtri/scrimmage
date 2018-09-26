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

#ifndef INCLUDE_SCRIMMAGE_PARSE_MISSIONPARSE_H_
#define INCLUDE_SCRIMMAGE_PARSE_MISSIONPARSE_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/proto/Visual.pb.h>

#include <scrimmage/proto/Color.pb.h>

#include <list>
#include <vector>
#include <map>
#include <memory>
#include <string>

namespace sp = scrimmage_proto;

namespace scrimmage {

// Key 1: Entity Description XML ID
// Value 1: Map of entity information
// Key 2: entity param string
// Value 2: string value for entity param
typedef std::map<int, std::map<std::string, std::string>> EntityDesc_t;

// key1 = xml node name
// key2 = xml node attribute name
// value = xml node attribute value
typedef std::map<std::string, std::map<std::string, std::string>> AttributeMap;

struct TeamInfo {
    int team_id = 0;
    scrimmage_proto::Color color;
    std::list<Eigen::Vector3d> bases;
    std::vector<double> radii;
    std::vector<double> opacities;
};

struct GenerateInfo {
    double start_time;
    int total_count;
    int gen_count;
    double rate;
    bool first_in_group;
    double time_variance;
};

class MissionParse {
 public:
    bool create_log_dir();
    bool parse(const std::string &filename);
    bool write(const std::string &filename);

    double t0();
    double tend();
    double dt();
    double motion_multiplier();
    double time_warp();
    void set_time_warp(double warp);
    bool start_paused();

    bool parse_terrain();

    scrimmage_proto::Color & background_color();

    void set_log_dir(const std::string &log_dir);
    std::string log_dir();
    std::string root_log_dir();

    std::map<int, AttributeMap> &entity_attributes();

    std::map<int, std::map<std::string, std::string>> &entity_params();

    std::map<int, int> &ent_id_to_block_id();

    EntityDesc_t & entity_descriptions();

    std::map<std::string, int> & entity_tag_to_id();

    bool enable_gui();
    void set_enable_gui(bool enable);
    bool network_gui();
    void set_network_gui(bool enable);
    void set_start_paused(bool paused);

    AttributeMap &attributes();
    std::map<std::string, std::string> & params();

    double longitude_origin();
    double latitude_origin();
    double altitude_origin();

    std::map<int, TeamInfo> & team_info();

    void set_task_number(int task_num);
    void set_job_number(int job_num);

    std::list<std::string> entity_interactions();
    std::list<std::string> & network_names();

    std::list<std::string> metrics();

    std::map<int, GenerateInfo > & gen_info();

    std::map<int, std::vector<double>> &next_gen_times();

    std::shared_ptr<GeographicLib::LocalCartesian> projection();

    std::shared_ptr<scrimmage_proto::UTMTerrain> & utm_terrain();

    std::string get_mission_filename();

 protected:
    std::string mission_filename_ = "";

    double t0_ = 0;
    double tend_ = 50;
    double dt_ = 0.00833333;
    int motion_multiplier_ = 1;
    double time_warp_ = 0;

    bool enable_gui_ = true;
    bool network_gui_ = false;
    bool start_paused_ = false;

    AttributeMap attributes_;
    std::map<std::string, std::string> params_;

    std::map<int, TeamInfo > team_info_;

    std::map<int, AttributeMap> entity_attributes_;
    std::map<int, std::map<std::string, std::string>> entity_params_;

    // Key: Entity ID
    // Value: XML "entity" block used to create entity
    // This can be used to find the key for which entity_attributes_ maps to
    // the entity XML block.
    std::map<int, int> ent_id_to_block_id_;

    EntityDesc_t entity_descs_;

    std::string root_log_dir_;
    std::string log_dir_;

    double longitude_origin_ = 29.0;
    double latitude_origin_ = -95.0;
    double altitude_origin_ = 0;

    scrimmage_proto::Color background_color_;

    int task_number_ = -1;
    int job_number_ = -1;

    std::list<std::string> entity_interactions_;
    std::list<std::string> metrics_;
    std::list<std::string> network_names_;

    // Key: entity_description ID in EntityDesc_t map
    // Value: generation
    std::map<int, GenerateInfo> gen_info_;

    // Key: entity_description ID in EntityDesc_t vector
    std::map<int, std::vector<double> > next_gen_times_;

    std::map<std::string, int> entity_tag_to_id_;

    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

    std::shared_ptr<scrimmage_proto::UTMTerrain> utm_terrain_;
};
using MissionParsePtr = std::shared_ptr<MissionParse>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PARSE_MISSIONPARSE_H_
