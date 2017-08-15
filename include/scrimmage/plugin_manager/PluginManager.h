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

#ifndef INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINMANAGER_H_
#define INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINMANAGER_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/ConfigParse.h>

#include <list>
#include <map>
#include <unordered_set>
#include <memory>
#include <string>

namespace scrimmage {

class PluginInfo {
 public:
    PluginInfo() { }
    std::string name;
    std::string type;
    std::string path;
    bool returned = false;
    void * handle;
};

class PluginManager {
 public:
    typedef const char * (*plugin_name_t)();
    typedef const char * (*plugin_type_t)();

    PluginManager();

    void print_plugins(std::string plugin_type, std::string title, FileSearch &file_search);
    void print_returned_plugins();
    PluginPtr make_plugin(std::string plugin_type, std::string &plugin_name_xml, FileSearch &file_search, ConfigParse &config_parse, std::map<std::string, std::string> &overrides);
    std::map<std::string, std::unordered_set<std::string>> get_commits();
    void set_reload(bool reload);
    bool get_reload();

 protected:
    // Key 1: Plugin Type
    // Key 2: Plugin Name
    // Value: PluginInfo
    std::map<std::string, std::map<std::string, PluginInfo>> plugins_;

    std::unordered_map<std::string, std::list<std::string>> so_files_;
    bool files_checked_ = false;

    int check_library(std::string lib_path);
    PluginPtr make_plugin_helper(std::string &plugin_type, std::string &plugin_name);
    bool reload_;
};

using PluginManagerPtr = std::shared_ptr<PluginManager>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINMANAGER_H_
