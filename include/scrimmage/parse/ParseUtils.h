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

#ifndef PARSE_UTILS_H_
#define PARSE_UTILS_H_
#include <map>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>

#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/proto/Visual.pb.h>

namespace scrimmage {

std::string expand_user(std::string path);

bool str2bool(std::string str);

template <class T> T convert(std::string str) {
    T num;
    if (!(std::istringstream(str) >> num)) {
        num = 0;
    }
    return num;
}

template <> inline bool convert<bool>(std::string str) {return str2bool(str);}
template <> inline const char* convert<const char *>(std::string str) {return str.c_str();}
template <> inline std::string convert<std::string>(std::string str) {return str;}

template <class T1, class T2 = T1>
    T2 get(const std::string &key, std::map<std::string, std::string> &map, T1 default_val) {
    auto it = map.find(key);
    return it == map.end() ? default_val : convert<T2>(it->second);
}

template <typename T>
bool str2vec(std::string &str, std::string delims,
             std::vector<T> &vec, unsigned int size) {
    vec.clear();

    std::vector<std::string> tokens;
    boost::split(tokens, str, boost::is_any_of(delims));

    for (std::string &t : tokens) {
        if (t.length() > 0) {
            vec.push_back(convert<T>(t));
        }
    }

    if (size != vec.size()) return false;
    return true;
}

template <typename T>
bool get_vec(std::string str,
             std::map<std::string, std::string> &params,
             std::string delims,
             std::vector<T> &vec,
             unsigned int size) {
    if (params.count(str) == 0)  return false;
    return str2vec<T>(params[str], delims, vec, size);
}

bool get_vec(std::string str,
             std::map<std::string, std::string> & params,
             std::vector<std::string> &vec);

Eigen::Vector3d vec2eigen(std::vector<double> &vec);


bool find_terrain_files(std::string terrain_name,
                        ConfigParse &terrain_parse,
                        std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain);

bool find_model_properties(std::string model_name, ConfigParse &cv_parse,
                           FileSearch &file_search,
                           std::shared_ptr<scrimmage_proto::ContactVisual> &cv,
                           bool &mesh_found, bool &texture_found);

bool parse_autonomy_data(std::map<std::string, std::string> &params,
                         std::map<std::string, std::string> &data_params);
} // namespace scrimmage
#endif
