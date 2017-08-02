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

#ifndef EXTERNAL_H_
#define EXTERNAL_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/common/FileSearch.h>
#include <map>

namespace scrimmage {

class External {
 public:
    External();
    NetworkPtr &network();
    EntityPtr &entity();
    FileSearch &file_search();
    PluginManagerPtr &plugin_manager();
    int get_max_entities();
    void set_max_entities(int max_entities);
    bool create_entity(ID id,
            std::map<std::string, std::string> &info,
            const std::string &log_dir);

 protected:
    NetworkPtr network_;
    EntityPtr entity_;
    FileSearch file_search_;
    PluginManagerPtr plugin_manager_;
    int max_entities_;
};

} // namespace scrimmage

#endif // EXTERNAL_H_
