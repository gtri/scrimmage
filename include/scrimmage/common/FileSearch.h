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

#ifndef INCLUDE_SCRIMMAGE_COMMON_FILESEARCH_H_
#define INCLUDE_SCRIMMAGE_COMMON_FILESEARCH_H_

#include <unordered_map>
#include <list>
#include <memory>
#include <string>

namespace boost {
template <class T> class optional;
}

namespace scrimmage {

class FileSearch {
 public:
    void clear();

    /*! \brief finds a mission file
     *
     * The search path is:
     *  * SCRIMMAGE_MISSION_PATH environment variable
     *  * /usr/share
     *  * /usr/local/share
     */
    boost::optional<std::string> find_mission(std::string mission, bool verbose = false);

    bool find_file(const std::string &filename, std::string ext,
        const std::string &env_var, std::string &result, bool verbose = false);
    void find_files(std::string env_var, const std::string &ext,
        std::unordered_map<std::string, std::list<std::string>> &out,
        bool verbose = false);

 protected:
    // cache_[env_var][ext][filename] = list of full paths to files with that filename
    std::unordered_map<std::string,
        std::unordered_map<std::string,
            std::unordered_map<std::string, std::list<std::string>>>> cache_;
};

using FileSearchPtr = std::shared_ptr<FileSearch>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_FILESEARCH_H_
