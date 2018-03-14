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
 *   along with SCRIMMAGE.  If nogt, see <http://www.gnu.org/licenses/>.
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

std::string generate_chars(const std::string &symbol, int num);

std::string eigen_str(const Eigen::VectorXd &vec, uint8_t precision = 2);

std::vector<double> linspace(double low, double high, uint32_t n);

template <class T>
T interp(const T &low, const T &high, double pct) {
    return low + pct * (high - low);
}

template <class T>
T scale(const T &input, const T &in_min, const T &in_max,
        const T &out_min, const T &out_max) {
    T result = input;
    // Check for input/output min/max bounds
    if (in_min > in_max || out_min > out_max) {
        // cout << "Error: in_min > in_max || out_min > out_max" << endl;
        return 0;
    }

    // Saturate input
    if (result < in_min) {
        result = in_min;
    } else if (result > in_max) {
        result = in_max;
    }

    double scale_factor = (out_max - out_min) / (in_max - in_min);
    return (result - in_min) * scale_factor + out_min;
}

template <class T>
Eigen::VectorXd scale(const Eigen::VectorXd &input, const T &in_min,
                      const T &in_max, const T &out_min, const T &out_max) {
    Eigen::VectorXd result = input;
    for (int i = 0; i < input.size(); i++) {
        result(i) = scale(input(i), in_min, in_max, out_min, out_max);
    }
    return result;
}

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_UTILITIES_H_
