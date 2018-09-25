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
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/FileSearch.h>

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <memory>
#include <string>

#ifdef __APPLE__
#define LIB_EXT ".dylib"
#else
#define LIB_EXT ".so"
#endif

namespace scrimmage {

class FileSearch;
class ConfigParse;

class PluginInfo {
 public:
    std::string name;
    std::string type;
    std::string path;
    bool returned = false;
    void * handle = nullptr;
};

template <class T>
class PluginStatus {
 public:
    enum Status {parse_failed, invalid_tag, missing, loaded, cast_failed};
    Status status = missing;
    std::shared_ptr<T> plugin = nullptr;
};

class PluginManager {
 public:
    typedef const char * (*plugin_name_t)();
    typedef const char * (*plugin_type_t)();

    PluginManager();

    void print_plugins(const std::string &plugin_type,
                       const std::string &title,
                       FileSearch &file_search);
    void print_returned_plugins();

    template <class T>
    PluginStatus<T> make_plugin(std::string plugin_type,
                                const std::string &plugin_name_xml,
                                FileSearch &file_search,
                                ConfigParse &config_parse,
                                std::map<std::string, std::string> &overrides,
                                const std::set<std::string> &plugin_tags) {
        PluginStatus<T> status;
        std::string plugin_name = plugin_name_xml;
        auto it_orig_plugin_name = overrides.find("ORIGINAL_PLUGIN_NAME");
        if (it_orig_plugin_name != overrides.end()) {
            plugin_name = it_orig_plugin_name->second;
        }

        config_parse.set_required("library");
        if (!config_parse.parse(overrides, plugin_name, "SCRIMMAGE_PLUGIN_PATH", file_search)) {
            std::cout << "Failed to parse: " << plugin_name << ".xml for type " << plugin_type << std::endl;
            std::cout << "Check that you have sourced ~/.scrimmage/setup.bash "
                      << "(this sets the environment variable SCRIMMAGE_PLUGIN_PATH to a directory including "
                      << plugin_name << ".xml)" << std::endl;
            status.status = PluginStatus<T>::parse_failed;
            return status;
        }

        // If the plugin_tags aren't empty, then we filter based on plugin_tags
        if (not plugin_tags.empty()) {
            // Does this plugin have a plugin "tags" attribute
            bool valid_tag = false;
            auto it_tags = overrides.find("tags");
            if (it_tags != overrides.end()) {
                // Break up the plugin's tags attribute into a set of strings
                auto tags = str2container<std::set<std::string>>(it_tags->second, ", ");
                // If the plugin_tags contains any tag in tags, this plugin
                // should be loaded
                valid_tag = std::any_of(tags.begin(), tags.end(),
                                        [&](const std::string &tag) {
                                            return (plugin_tags.find(tag) !=
                                                    plugin_tags.end());
                                        });
            }
            if (not valid_tag) {
                status.status = PluginStatus<T>::invalid_tag;
                return status;
            }
        }

        std::string plugin_name_so = config_parse.params()["library"];

        // first, if this has already been processed, return it
        PluginPtr plugin = make_plugin_helper(plugin_type, plugin_name_so);
        if (plugin != nullptr) {
            status.plugin = std::dynamic_pointer_cast<T>(plugin);
            if (status.plugin == nullptr) {
                status.status = PluginStatus<T>::cast_failed;
            } else {
                status.status = PluginStatus<T>::loaded;
            }
            return status;
        }

        if (!files_checked_ && so_files_.empty()) {
            file_search.find_files("SCRIMMAGE_PLUGIN_PATH", LIB_EXT, so_files_);
            files_checked_ = true;
        }

        // try the most obvious case, that the lib is named the same as the
        // plugin_name
        auto it = so_files_.find(std::string("lib") + plugin_name_so + LIB_EXT);
        if (it != so_files_.end()) {
            for (std::string &full_fname : it->second) {
                if (check_library(full_fname) == 0) {
                    // don't need to check again that it is in the map since the helper
                    // will do this already
                    so_files_.erase(it);
                    PluginPtr plugin = make_plugin_helper(plugin_type, plugin_name_so);
                    status.plugin = std::dynamic_pointer_cast<T>(plugin);
                    if (status.plugin == nullptr) {
                        status.status = PluginStatus<T>::cast_failed;
                    } else {
                        status.status = PluginStatus<T>::loaded;
                    }
                    return status;
                }
            }
        }

        // last, loop through the files
        it = so_files_.begin();
        while (it != so_files_.end()) {
            for (std::string &full_fname : it->second) {
                if (check_library(full_fname) == 0) {
                    // don't need to check again that it is in the map since the helper
                    // will do this already
                    so_files_.erase(it);
                    PluginPtr plugin = make_plugin_helper(plugin_type, plugin_name_so);
                    status.plugin = std::dynamic_pointer_cast<T>(plugin);
                    if (status.plugin == nullptr) {
                        status.status = PluginStatus<T>::cast_failed;
                    } else {
                        status.status = PluginStatus<T>::loaded;
                    }
                    return status;
                }
            }
            it++;
        }

        std::cout << "could not locate " << plugin_type << "::" << plugin_name_so << std::endl;
        status.status = PluginStatus<T>::missing;
        return status;
    }

    std::map<std::string, std::unordered_set<std::string>> get_commits();
    void set_reload(bool reload);
    bool get_reload();

    // std::list<PluginPtr> &plugins() { return plugins_; }

 protected:
    // Key 1: Plugin Type
    // Key 2: Plugin Name
    // Value: PluginInfo
    std::map<std::string, std::map<std::string, PluginInfo>> plugins_info_;

    std::unordered_map<std::string, std::list<std::string>> so_files_;
    bool files_checked_ = false;

    int check_library(std::string lib_path);
    PluginPtr make_plugin_helper(std::string &plugin_type, std::string &plugin_name);
    bool reload_;

    // std::list<PluginPtr> plugins_;
};

using PluginManagerPtr = std::shared_ptr<PluginManager>;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINMANAGER_H_
