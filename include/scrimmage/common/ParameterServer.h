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
#include <unordered_map>
#include <set>
#include <tuple>
#include <memory>
#include <iostream>
#include <algorithm>
#include <functional>

#include <boost/optional.hpp>

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
        auto it = params_[name][typeid(T).name()].emplace(
            std::make_shared<Parameter<T>>(variable, callback, owner));
        return it.second; // return false if the param already exists
    }

    template <class T>
    bool set_param(const std::string &name, const T &value) {
        auto param_set = find_name_type(name, typeid(T).name());
        if (param_set) {
            for (ParameterBasePtr param : *param_set) {
                auto param_cast = std::dynamic_pointer_cast<Parameter<T>>(param);
                param_cast->set_value(value);
            }
            return true;
        }
        return false;
    }

    template <class T>
    bool unregister_param(const std::string &name, PluginPtr owner) {
        auto param_set = find_name_type(name, typeid(T).name());
        if (param_set) {
            // Remove the parameter if this is the owner
            return remove_if_owner(*param_set, owner);
        }
        // Could not find the appropriate param name/type/plugin-owner
        return false;
    }

 protected:
    bool remove_if_owner(std::set<ParameterBasePtr> &param_set,
                         PluginPtr owner);

    inline boost::optional<std::set<ParameterBasePtr>&>
        find_name_type(const std::string &name, const std::string &type) {
        // Search for the parameter name
        auto it_name = params_.find(name);
        if (it_name != params_.end()) {
            // Search for the parameter type
            auto it_type = it_name->second.find(type);
            if (it_type != it_name->second.end()) {
                return boost::optional<std::set<ParameterBasePtr>&>
                    (it_type->second);
            }
        }
        return boost::none;
    }

    // Key 1: parameter name (string)
    // Key 2: parameter type (as a string)
    // Value: Set of ParameterBasePtr
    std::unordered_map<std::string,
        std::unordered_map<std::string, std::set<ParameterBasePtr>>> params_;
};
using ParameterServerPtr = std::shared_ptr<ParameterServer>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_PARAMETERSERVER_H_
