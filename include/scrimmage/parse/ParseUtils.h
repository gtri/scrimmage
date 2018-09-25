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

#ifndef INCLUDE_SCRIMMAGE_PARSE_PARSEUTILS_H_
#define INCLUDE_SCRIMMAGE_PARSE_PARSEUTILS_H_
#include <Eigen/Dense>

#include <map>
#include <set>
#include <vector>
#include <string>
#include <memory>

namespace scrimmage_proto {
class UTMTerrain;
class ContactVisual;
}

namespace scrimmage {

class PID;
class FileSearch;
class ConfigParse;

std::string expand_user(std::string path);

bool str2bool(std::string str);

template <class T>
std::enable_if_t<!std::is_pointer<T>::value, T>
convert(const std::string &str) {
    T num;
    if (!(std::istringstream(str) >> num)) {
        num = 0;
    }
    return num;
}

template <class T>
std::enable_if_t<std::is_pointer<T>::value, T>
convert(const std::string &str) {
    return nullptr;
}

template <> inline bool convert<bool>(const std::string &str) {return str2bool(str);}
template <> inline const char* convert<const char *>(const std::string &str) {return str.c_str();}
template <> inline std::string convert<std::string>(const std::string &str) {return str;}

template <class T1, class T2 = T1>
    T2 get(const std::string &key, std::map<std::string, std::string> &map, T1 default_val) {
    auto it = map.find(key);
    return it == map.end() ? default_val : convert<T2>(it->second);
}

// a wrapper around boost so that boost does not have to be included
void split(std::vector<std::string> &tokens, const std::string &str,
           const std::string &delims);

template <typename T>
T str2container(const std::string &str, const std::string &delims) {
    T out;
    std::vector<std::string> tokens;
    split(tokens, str, delims);

    for (std::string &t : tokens) {
        if (t.length() > 0) {
            out.insert(out.end(), convert<typename T::value_type>(t));
        }
    }
    return out;
}

template <typename T>
bool str2container(const std::string &str, const std::string &delims,
                   T &result, int size = -1) {
    T tmp = str2container<T>(str, delims);
    if (size >= 0 && (unsigned int)size != tmp.size()) {
        return false;
    } else {
        // Only modify vec if it was a valid conversion
        result = tmp;
        return true;
    }
}

template <typename T>
[[deprecated("str2vec is deprecated, use str2container instead")]]
std::vector<T> str2vec(const std::string &str, const std::string &delims) {
    std::vector<T> out;
    std::vector<std::string> tokens;
    split(tokens, str, delims);

    for (std::string &t : tokens) {
        if (t.length() > 0) {
            out.push_back(convert<T>(t));
        }
    }
    return out;
}

template <typename T>
[[deprecated("str2vec is deprecated, use str2container instead")]]
bool str2vec(const std::string &str, const std::string &delims,
             std::vector<T> &vec, int size = -1) {
    std::vector<T> tmp_vec;
    std::vector<std::string> tokens;
    split(tokens, str, delims);

    for (std::string &t : tokens) {
        if (t.length() > 0) {
            tmp_vec.push_back(convert<T>(t));
        }
    }

    if (size >= 0 && (unsigned int)size != tmp_vec.size()) {
        return false;
    } else {
        // Only modify vec if it was a valid conversion
        vec = tmp_vec;
        return true;
    }
}

template <typename T>
bool get_vec(const std::string &str,
             const std::map<std::string, std::string> &params,
             const std::string &delims,
             std::vector<T> &vec,
             int size = -1) {
    auto it = params.find(str);
    return it == params.end() ? false : str2container(it->second, delims, vec, size);
}

bool get_vec(const std::string &str,
             std::map<std::string, std::string> & params,
             std::vector<std::string> &vec);

Eigen::Vector3d vec2eigen(std::vector<double> &vec);


bool find_terrain_files(std::string terrain_name,
                        ConfigParse &terrain_parse,
                        std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain);

bool find_model_properties(std::string model_name, ConfigParse &cv_parse,
                           FileSearch &file_search,
                           std::map<std::string, std::string> &overrides,
                           std::shared_ptr<scrimmage_proto::ContactVisual> &cv,
                           bool &mesh_found, bool &texture_found);

bool parse_autonomy_data(std::map<std::string, std::string> &params,
                         std::map<std::string, std::string> &data_params);

bool get_vec_of_vecs(std::string &str,
                     std::vector<std::vector<std::string>> &out,
                     const std::string &delims = " ,");

bool set_pid_gains(PID &pid, std::string str,
                   bool is_angle = false);

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PARSE_PARSEUTILS_H_
