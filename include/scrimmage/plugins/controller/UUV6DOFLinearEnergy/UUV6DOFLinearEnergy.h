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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFLINEARENERGY_UUV6DOFLINEARENERGY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFLINEARENERGY_UUV6DOFLINEARENERGY_H_

#include <scrimmage/motion/Controller.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class UUV6DOFLinearEnergy : public scrimmage::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    // Inputs
    uint8_t in_throttle_idx_ = 0;
    uint8_t in_elevator_idx_ = 0;
    uint8_t in_rudder_idx_ = 0;

    // Outputs
    uint8_t out_throttle_idx_ = 0;
    uint8_t out_elevator_idx_ = 0;
    uint8_t out_rudder_idx_ = 0;

    double energy_ = 1000;
    double energy_max_ = 5000;
    double energy_min_ = 0;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_UUV6DOFLINEARENERGY_UUV6DOFLINEARENERGY_H_
