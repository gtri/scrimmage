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

#include <dlfcn.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/plugin_manager/Plugin.h>

#include <iostream>
#include <set>

#include <boost/filesystem.hpp>

namespace scrimmage {

int PluginManager::check_library(std::string lib_path) {
    void *lib_handle;
#ifdef __APPLE__
    lib_handle = dlopen(lib_path.c_str(), RTLD_LAZY | RTLD_GLOBAL);
#else
    lib_handle = dlopen(lib_path.c_str(), RTLD_LAZY | RTLD_LOCAL);
#endif
    if (!lib_handle) {
        fputs(dlerror(), stderr);
        std::cout << std::endl;
        return -1;
    }

    char * error;

    // Extract the plugin type
    plugin_type_t type_func;
    type_func = (plugin_type_t)dlsym(lib_handle, "plugin_type");
    if ((error = dlerror()) != NULL)  {
        // fputs(error, stderr);
        std::cout << lib_path << " doesn't contain 'plugin_type'" << std::endl;
        dlclose(lib_handle);
        return 1;
    }
    std::string plugin_type((*type_func)());

    // Extract the plugin name
    plugin_name_t name_func;
    name_func = (plugin_name_t)dlsym(lib_handle, "plugin_name");
    if ((error = dlerror()) != NULL)  {
        // fputs(error, stderr);
        std::cout << lib_path << " doesn't contain 'plugin_name'" << std::endl;
        dlclose(lib_handle);
        return 3;
    }
    std::string plugin_name((*name_func)());

    // Ensure the maker function exists, but we don't need to call it
    dlsym(lib_handle, "maker");
    if ((error = dlerror()) != NULL)  {
        fputs(error, stderr);
        std::cout << lib_path << " doesn't contain 'maker'" << std::endl;
        dlclose(lib_handle);
        return 4;
    }

    PluginInfo info;
    info.name = plugin_name;
    info.type = plugin_type;
    info.path = lib_path;
    info.handle = lib_handle;

    plugins_info_[plugin_type][plugin_name] = info;

    // dlclose(lib_handle);

    return 0;
}

PluginManager::PluginManager() : reload_(false) {}

void PluginManager::print_plugins(const std::string &plugin_type, const std::string &title, FileSearch &file_search) {
    // make sure all files are loaded
    if (!files_checked_) {
        file_search.find_files("SCRIMMAGE_PLUGIN_PATH", LIB_EXT, so_files_);
        files_checked_ = true;
    }

    for (auto &kv : so_files_) {
        for (std::string &full_path : kv.second) {
            check_library(full_path);
        }
    }

    std::cout << "------------------------------" << std::endl;
    std::cout << title << ": " << std::endl;
    std::cout << "------------------------------" << std::endl;
    if (plugins_info_.count(plugin_type) > 0) {
        for (auto &kv : plugins_info_[plugin_type]) {
            std::cout << kv.first << std::endl;
        }
    } else {
        std::cout << "Plugin type doesn't exist: " << plugin_type << std::endl;
    }
    std::cout << "------------------------------" << std::endl;
}

void PluginManager::print_returned_plugins() {
    std::cout << "using the following plugins:" << std::endl;
    for (auto &kv : plugins_info_) {
        for (auto &kv2 : kv.second) {
            if (kv2.second.returned) {
                std::cout << kv.first << "::" << kv2.first << std::endl;
            }
        }
    }
}

PluginPtr PluginManager::make_plugin_helper(std::string &plugin_type, std::string &plugin_name) {
    auto it = plugins_info_.find(plugin_type);
    if (it != plugins_info_.end()) {
        auto it2 = it->second.find(plugin_name);
        if (it2 != it->second.end()) {
            it2->second.returned = true;

            if (reload_ && it2->second.handle) {
                dlclose(it2->second.handle);
            }

            PluginPtr (*maker_func)(void);
            // cppcheck-suppress cstyleCast
            maker_func = (PluginPtr (*)(void))dlsym(it2->second.handle, "maker");
            char * error;
            if ((error = dlerror()) != NULL)  {
                fputs(error, stderr);
                return nullptr;
            } else {
                return (*maker_func)();
            }
        }
    }
    return nullptr;
}

std::map<std::string, std::unordered_set<std::string>> PluginManager::get_commits() {
    std::map<std::string, std::unordered_set<std::string>> commits;
    std::string sha;
    for (auto &kv : plugins_info_) {
        for (auto &kv2 : kv.second) {
            scrimmage::PluginInfo &plugin_info = kv2.second;
            if (!plugin_info.returned) continue;

            std::string path =
                boost::filesystem::path(plugin_info.path).parent_path().string();

            if ((sha = scrimmage::get_sha(path)) != "") {
                commits[sha].insert(plugin_info.name);
            }
        }
    }
    return commits;
}

void PluginManager::set_reload(bool reload) {reload_ = reload;}

bool PluginManager::get_reload() {return reload_;}

} // namespace scrimmage
