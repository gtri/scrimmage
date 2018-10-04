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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIACTIONS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIACTIONS_H_

#include <pybind11/pybind11.h>

#include <scrimmage/common/Visibility.h>

#include <vector>
#include <memory>

namespace scrimmage {
namespace autonomy {

class ScrimmageOpenAIAutonomy;

class DLL_PUBLIC OpenAIActions {
 public:
    OpenAIActions();
    std::vector<std::shared_ptr<ScrimmageOpenAIAutonomy>> &ext_ctrl_vec();
    void create_action_space(bool combine_actors);
    void distribute_action(pybind11::object action, bool combine_actors);

    pybind11::object action_space;

 protected:
    std::vector<std::shared_ptr<ScrimmageOpenAIAutonomy>> ext_ctrl_vec_;
    pybind11::object tuple_space_;
    pybind11::object box_space_;
    pybind11::object asarray_;
};

} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIACTIONS_H_
