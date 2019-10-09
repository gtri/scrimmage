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

#ifndef INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINHELPER_H_
#define INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINHELPER_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include <map>
#include <set>
#include <string>
#include <memory>
#include <iostream>
#include <list>

using std::cout;
using std::endl;

namespace scrimmage {

template <class T>
struct PluginAndConfig {
    std::shared_ptr<T> plugin;
    ConfigParse config_parse;
    PluginAndConfig() : plugin(nullptr) {}
     PluginAndConfig(std::shared_ptr<T> p, ConfigParse &cp)
             : plugin(p), config_parse(cp) { }
};

template <class T>
bool load_plugins(const std::list<PluginOverrides>& plugin_names_overrides,
                  const std::string& parent_class_name,
                  std::map<std::string, PluginAndConfig<T>> &plugins_and_configs,
                  const std::set<std::string>& plugin_tags = {},
                  PluginManagerPtr mgr = std::make_shared<PluginManager>(),
                  FileSearchPtr file_search = std::make_shared<FileSearch>()
                  ) {
    bool all_loaded = true;
    for (const auto &plugin : plugin_names_overrides) {
        ConfigParse config_parse;

        PluginStatus<T> status =
                mgr->make_plugin<T>(parent_class_name,
                                    plugin.name, *file_search, config_parse,
                                    plugin.overrides, plugin_tags);

        if (status.status == PluginStatus<T>::cast_failed) {
            cout << "Failed to open: " << parent_class_name  << ": " << plugin.name << endl;
            all_loaded = false;
        } else if (status.status == PluginStatus<T>::parse_failed) {
            cout << "Failed to parse: " << parent_class_name  << ": " << plugin.name << endl;
            all_loaded = false;
        } else if (status.status == PluginStatus<T>::loaded) {
            auto it_name = config_parse.params().find("name");
            std::string plugin_name = it_name == config_parse.params().end() ?
                    plugin.name : it_name->second;

            plugins_and_configs[plugin_name] = PluginAndConfig<T>(status.plugin, config_parse);
        }
        all_loaded &= (status.status == PluginStatus<T>::loaded);
    }
    return all_loaded;
}
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_PLUGINHELPER_H_
