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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_H_

#include <scrimmage/motion/Controller.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class AircraftToSingleIntegratorController : public Controller {
 public:
    virtual void init(std::map<std::string,  std::string> &params) {}
    virtual bool step(double t, double dt) {
        double desired_heading = desired_state_->quat().yaw();
        double desired_alt = desired_state_->pos()(2);
        double desired_speed = desired_state_->vel()(0);

        // Convert desired speed, pitch, and heading into velocity
        u_[0] = desired_speed * cos(desired_heading);
        u_[1] = desired_speed * sin(desired_heading);
        u_[2] = desired_alt - state_->pos()(2);

        return true;
    }

    virtual Eigen::Vector3d &u() {return u_;}

 protected:
    Eigen::Vector3d u_;
};
} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_AIRCRAFTTOSINGLEINTEGRATORCONTROLLER_H_
