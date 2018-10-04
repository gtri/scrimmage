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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RLSIMPLE_RLSIMPLE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RLSIMPLE_RLSIMPLE_H_

#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

#include <map>
#include <string>
#include <utility>

namespace scrimmage {
namespace autonomy {

class RLSimple : public ScrimmageOpenAIAutonomy {
 public:
    void init_helper(std::map<std::string, std::string> &params) override;
    bool step_helper() override;

    void set_environment() override;
    std::pair<bool, double> calc_reward() override;

 protected:
    double radius_;

    bool x_discrete_ = true;

    bool ctrl_y_ = false;
    bool y_discrete_ = true;

    uint8_t output_vel_x_idx_ = 0;
    uint8_t output_vel_y_idx_ = 0;
};
} // namespace autonomy
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RLSIMPLE_RLSIMPLE_H_
