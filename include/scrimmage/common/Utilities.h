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

#ifndef INCLUDE_SCRIMMAGE_COMMON_UTILITIES_H_
#define INCLUDE_SCRIMMAGE_COMMON_UTILITIES_H_

#include <Eigen/Dense>

#include <map>
#include <vector>
#include <string>

namespace scrimmage {

void display_progress(float progress);

int next_available_id(std::string name,
                      std::map<std::string, std::string> &info,
                      std::map<int, int> &id_map);

std::string get_sha(std::string &path);

std::string get_version();

void filter_line(int downsampling_factor,
    int num_points,
    std::vector<Eigen::Vector3d> &path,
    std::vector<Eigen::Vector3d> &filtered_path);

std::string generate_chars(std::string symbol, int num);

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_UTILITIES_H_
