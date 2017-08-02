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

#ifndef MISSIONPARSE_H_
#define MISSIONPARSE_H_
#include <list>
#include <vector>
#include <map>
#include <memory>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/proto/Visual.pb.h>

#include <Eigen/Dense>
#include <scrimmage/proto/Color.pb.h>

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
    int team_id;
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
    MissionParse();
    bool create_log_dir();
    bool parse(std::string filename);
    bool write(std::string filename);

    double t0();
    double tend();
    double dt();
    double motion_multiplier();
    double time_warp();
    bool start_paused();

    bool parse_terrain();

    scrimmage_proto::Color & background_color();

    void set_log_dir(const std::string &log_dir);
    std::string log_dir();

    std::map<int, AttributeMap> &entity_attributes();
    EntityDesc_t & entity_descriptions();

    bool enable_gui();
    bool network_gui();

    AttributeMap &attributes();
    std::map<std::string,std::string> & params();

    double longitude_origin();
    double latitude_origin();
    double altitude_origin();

    std::map<int,TeamInfo> & team_info();

    void set_task_number(int task_num);
    void set_job_number(int job_num);

    std::list<std::string> entity_interactions();

    std::list<std::string> metrics();

    std::map<int, GenerateInfo > & gen_info();

    std::map<int, std::vector<double> > & next_gen_times();

    std::shared_ptr<GeographicLib::LocalCartesian> projection();
    
    std::shared_ptr<scrimmage_proto::UTMTerrain> & utm_terrain();
    
protected:
    std::string mission_filename_;

    double t0_;
    double tend_;
    double dt_;
    int motion_multiplier_;
    double time_warp_;

    bool enable_gui_;
    bool network_gui_;
    bool start_paused_;

    AttributeMap attributes_;
    std::map<std::string,std::string> params_;

    std::map<int, TeamInfo > team_info_;

    std::map<int, AttributeMap> entity_attributes_;
    EntityDesc_t entity_descs_;

    std::string log_dir_;

    double longitude_origin_;
    double latitude_origin_;
    double altitude_origin_;

    scrimmage_proto::Color background_color_;
        
    int task_number_;
    int job_number_;

    std::list<std::string> entity_interactions_;
    std::list<std::string> metrics_;

    // Key: entity_description ID in EntityDesc_t map
    // Value: generation
    std::map<int, GenerateInfo> gen_info_;

    // Key: entity_description ID in EntityDesc_t vector
    std::map<int, std::vector<double> > next_gen_times_;

    std::shared_ptr<GeographicLib::LocalCartesian> proj_;            

    std::shared_ptr<scrimmage_proto::UTMTerrain> utm_terrain_;
    
private:
};
using MissionParsePtr = std::shared_ptr<MissionParse>;
}

#endif
