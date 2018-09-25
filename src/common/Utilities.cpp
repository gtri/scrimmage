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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cassert>
#include <vector>
#include <memory>

#include <boost/algorithm/string.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
namespace fs = boost::filesystem;

namespace scrimmage {

int next_available_id(std::string name,
                      std::map<std::string, std::string> &info,
                      std::map<int, int> &id_map) {
    int id;
    if (info.count(name) > 0) {
        id = std::stoi(info[name]);
    } else {
        id = 1;
    }

    // Find the next available ID:
    while (id_map.count(id) > 0) id++;

    // Save the id in the map:
    id_map[id] = id;

    return id;
}

void display_progress(float progress) {
    int bar_width = 70;
    std::cout << "[";
    int pos = bar_width * progress;
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) {
            std::cout << "=";
        } else if (i == pos) {
            std::cout << ">";
        } else {
            std::cout << " ";
        }
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

std::string get_sha(std::string &path) {
    std::string cd_cmd = "cd " + path + " && ";
    std::string sha_cmd = cd_cmd + "git rev-parse HEAD | tr -d '\n'";
    std::string status_cmd = cd_cmd + "git status --porcelain | wc -l";

    FILE *sha_file = popen(sha_cmd.c_str(), "r");
    FILE *status_file = popen(status_cmd.c_str(), "r");

    char sha[41], status[3];
    bool success =
        fgets(sha, 40, sha_file) != NULL &&
        fgets(status, 2, status_file) != NULL;

    pclose(sha_file);
    pclose(status_file);

    if (!success) {
        return "";
    } else if (status[0] == '0') {
        return std::string(sha);
    } else {
        return std::string(sha) + "-dirty";
    }
}

std::string get_version() {
    const char *env = std::getenv("SCRIMMAGE_ROOT");
    if (env == NULL) {
        return std::string("");
    } else {
        std::string env_path(env);
        return get_sha(env_path);
    }
}

void filter_line(int downsampling_factor,
    int num_points,
    std::vector<Eigen::Vector3d> &path,
    std::vector<Eigen::Vector3d> &filtered_path) {

    int curvature_sz = path.size() / downsampling_factor;
    std::list<std::pair<int, double>> curvature;

    auto idx = [=](int i) {return downsampling_factor * i;};
    for (int i = 1; i < curvature_sz - 1; i++) {
        Eigen::Vector3d &pt_prev = path[idx(i - 1)];
        Eigen::Vector3d &pt = path[idx(i)];
        Eigen::Vector3d &pt_next = path[idx(i + 1)];

        double curv = (pt_next - 2 * pt + pt_prev).squaredNorm();
        curvature.push_back(std::make_pair(idx(i), curv));
    }

    using Pair = std::pair<int, double>;
    curvature.sort([](Pair &a, Pair &b) {return a.second > b.second;});
    curvature.erase(std::next(curvature.begin(), num_points), curvature.end());
    curvature.sort([](Pair &a, Pair &b) {return a.first < b.first;});

    filtered_path.clear();
    filtered_path.reserve(curvature.size() + 2);
    filtered_path.push_back(path[0]);

    for (Pair &p : curvature) {
        filtered_path.push_back(path[p.first]);
    }

    filtered_path.push_back(path.back());
}

std::string generate_chars(const std::string &symbol, int num) {
    std::string out = "";
    for (int i = 0; i < num; i++) {
        out += symbol;
    }
    return out;
}

std::string eigen_str(const Eigen::VectorXd &vec, uint8_t precision) {
    if (vec.size() == 0) {
        return "";
    } else {
        std::stringstream ss;
        for (int i = 0; i < vec.size() - 1; i++) {
            ss << std::setprecision(precision) << vec(i) << ", ";
        }
        ss << std::setprecision(precision) << vec(vec.size() - 1);
        return ss.str();
    }
}

std::vector<double> linspace(double low, double high, uint32_t n) {
    std::vector<double> out;
    if (n > 0) {
        out.reserve(n);
        out.push_back(low);
        if (n > 1) {
            const double step = (high - low) / (n - 1);
            for (uint32_t i = 1; i < n - 1; i++) {
                out.push_back(low + step * i);
            }
            out.push_back(high);
        }
    }
    return out;
}

} // namespace scrimmage
