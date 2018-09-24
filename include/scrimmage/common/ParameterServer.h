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

#ifndef INCLUDE_SCRIMMAGE_COMMON_PARAMETERSERVER_H_
#define INCLUDE_SCRIMMAGE_COMMON_PARAMETERSERVER_H_

#include <scrimmage/common/Parameter.h>

#include <string>
#include <map>
#include <list>
#include <memory>
#include <iostream>
#include <algorithm>

class Plugin;
using PluginPtr = std::shared_ptr<Plugin>;

namespace scrimmage {
class ParameterServer {
 public:
    void unregister_params(PluginPtr owner);

    template <class T>
    bool register_param(const std::string &name, T &variable,
                        std::function<void(const T &value)> callback,
                        PluginPtr owner) {
        // Create the Parameter
        auto param = std::make_shared<Parameter<T>>(variable, callback, owner);

        // Search for the parameter name in the server
        auto it_name = params_.find(name);
        if (it_name == params_.end()) {
            // The parameter name doesn't exist it, create it.
            params_[name][typeid(T).name()] = std::list<ParameterBasePtr> {param};
        } else {
            // Does this type exist for this parameter name?
            auto it_type = it_name->second.find(typeid(T).name());
            if (it_type != it_name->second.end()) {
                // This param name / type combo exists
                // Has this specific plugin already registered this parameter?
                for (const ParameterBasePtr &ptr : it_type->second) {
                    if (ptr->owner() == owner) {
                        return false;
                    }
                }
            }
            // Add the parameter to the params_ map
            (it_name->second)[typeid(T).name()].push_back(param);
        }
        return true;
    }

    template <class T>
    bool set_param(const std::string &name, const T &value) {
        // Search for the parameter name in the server
        auto it_name = params_.find(name);
        if (it_name != params_.end()) {
            // The param name exists, does the type exist?
            auto it_type = it_name->second.find(typeid(T).name());
            if (it_type != it_name->second.end()) {
                // The param name/type exists in the list, iterate over the
                // list of parameters and call their set_value functions
                for (ParameterBasePtr param : it_type->second) {
                    auto param_cast = std::dynamic_pointer_cast<Parameter<T>>(param);
                    if (param_cast) {
                        param_cast->set_value(value);
                    }
                }
                return true;
            }
        }
        return false;
    }

    template <class T>
    bool unregister_param(const std::string &name, PluginPtr owner) {
        // Search for the parameter name in the server
        auto it_name = params_.find(name);
        if (it_name != params_.end()) {
            auto it_type = it_name->second.find(typeid(T).name());
            if (it_type != it_name->second.end()) {
                // The parameter name/type exists, let's find the specific
                // plugin
                auto it_param = std::remove_if(it_type->second.begin(),
                                               it_type->second.end(),
                                               [&](ParameterBasePtr ptr) {
                                                   return ptr->owner() == owner; });
                if (it_param != it_type->second.end()) {
                    // The plugin created this param, erase it.
                    it_type->second.erase(it_param, it_type->second.end());
                    return true;
                }
            }
        }
        // Could not find the appropriate param name/type/plugin-owner
        return false;
    }

 protected:
    // Key 1: parameter name (string)
    // Key 2: parameter type (as a string)
    // Value: ParameterBase shared_ptr
    std::map<std::string, std::map<std::string, std::list<ParameterBasePtr>>> params_;
};
typedef std::shared_ptr<ParameterServer> ParameterServerPtr;
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_PARAMETERSERVER_H_
