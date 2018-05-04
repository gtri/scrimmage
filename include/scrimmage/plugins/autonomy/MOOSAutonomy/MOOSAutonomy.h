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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSAUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSAUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/Angles.h>

#include <thread> // NOLINT
#include <string>
#include <map>

#include "MOOSNode.h"

namespace scrimmage {
namespace autonomy {
class MOOSAutonomy : public scrimmage::Autonomy {
 public:
    MOOSAutonomy();
    void init(std::map<std::string, std::string> &params) override;
    bool ready() override;
    bool step_autonomy(double t, double dt) override;

 protected:
    void run_moos_node();

    std::thread moos_node_thread_;
    std::thread ivp_thread_;
    MOOSNode moos_node_;

    std::string moos_app_name_;
    std::string moos_script_;
    std::string moos_mission_file_;

    scrimmage::Angles angles_to_moos_;
    scrimmage::Angles angles_from_moos_;

    int desired_heading_idx_ = 0;
    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSAUTONOMY_H_
