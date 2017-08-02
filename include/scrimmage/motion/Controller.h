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

#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include <memory>
#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>

namespace scrimmage {

class Controller : public Plugin {
 public:
    virtual void init(std::map<std::string, std::string> &params) = 0;
    virtual bool step(double t, double dt) = 0;
    inline void set_state(StatePtr &state) {state_ = state;}
    inline void set_desired_state(StatePtr &desired_state) {desired_state_ = desired_state;}
    
 protected:
    StatePtr state_;
    StatePtr desired_state_;
};

using ControllerPtr = std::shared_ptr<Controller>;

}
#endif // CONTROLLER_H_
