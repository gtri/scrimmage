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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUTONOMYEXECUTOR_AUTONOMYEXECUTOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUTONOMYEXECUTOR_AUTONOMYEXECUTOR_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/parse/ConfigParse.h>

#include <string>
#include <map>
#include <list>

namespace sc = scrimmage;

namespace scrimmage {
namespace autonomy {
class AutonomyExecutor : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    bool call_init(std::string autonomy_name, sc::AutonomyPtr autonomy_s);

 protected:
    bool show_shapes_ = false;

    std::string current_state_ = "UNDEFINED_NO_STATE";

    std::map<std::string, std::list<scrimmage::AutonomyPtr>> autonomies_;
    std::list<scrimmage::AutonomyPtr> running_autonomies_;
    std::list<scrimmage::AutonomyPtr> default_autonomies_;

    std::map<std::string, sc::ConfigParse> autonomies_config_;

    // Key : Output variable index determined by controller
    // Value: Input variable index
    std::map<int, int> io_map_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUTONOMYEXECUTOR_AUTONOMYEXECUTOR_H_
