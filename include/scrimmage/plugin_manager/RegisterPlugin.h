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
#ifndef INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_REGISTERPLUGIN_H_
#define INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_REGISTERPLUGIN_H_

#include <memory>

#define REGISTER_PLUGIN_HELPER(BaseClass, ChildClass, PluginName)   \
    extern "C" {                                                    \
        std::shared_ptr<BaseClass> maker() {                        \
            return std::make_shared<ChildClass>();                  \
        }                                                           \
        const char * plugin_name() {                                \
            return #PluginName;                                     \
        }                                                           \
        const char * plugin_type() {                                \
            return #BaseClass;                                      \
        }                                                           \
    }                                                               \

#ifdef __clang__
// http://clang.llvm.org/docs/UsersManual.html#diagnostics
// https://stackoverflow.com/a/28907242
#define REGISTER_PLUGIN(BaseClass, ChildClass, PluginName)          \
    _Pragma("clang diagnostic push")                                \
    _Pragma("clang diagnostic ignored \"-Wreturn-type-c-linkage\"") \
    REGISTER_PLUGIN_HELPER(BaseClass, ChildClass, PluginName)       \
    _Pragma("clang diagnostic pop")
#else
#define REGISTER_PLUGIN(BaseClass, ChildClass, PluginName)          \
    REGISTER_PLUGIN_HELPER(BaseClass, ChildClass, PluginName)
#endif

#endif // INCLUDE_SCRIMMAGE_PLUGIN_MANAGER_REGISTERPLUGIN_H_

