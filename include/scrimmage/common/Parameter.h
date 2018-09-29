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

#ifndef INCLUDE_SCRIMMAGE_COMMON_PARAMETER_H_
#define INCLUDE_SCRIMMAGE_COMMON_PARAMETER_H_

#include <string>
#include <memory>
#include <functional>

namespace scrimmage {

class Plugin;
using PluginPtr = std::shared_ptr<Plugin>;

class ParameterBase {
 public:
    explicit ParameterBase(PluginPtr &owner) : owner_(owner) {}
    virtual ~ParameterBase() {}
    const PluginPtr &owner() { return owner_; }
 protected:
    PluginPtr owner_ = nullptr;
};

template <class T>
class Parameter : public ParameterBase {
 public:
    Parameter(T &variable, std::function<void(const T &value)> callback,
              PluginPtr &owner) : ParameterBase(owner), value_(variable),
        callback_(callback) {}
    void set_value(const T &value) {
        value_ = value;
        callback_(value_);
    }
 protected:
    T &value_;
    std::function<void(const T &value)> callback_;
};

typedef std::shared_ptr<ParameterBase> ParameterBasePtr;

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_PARAMETER_H_
