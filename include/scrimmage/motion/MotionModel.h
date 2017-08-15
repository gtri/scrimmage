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

#ifndef INCLUDE_SCRIMMAGE_MOTION_MOTIONMODEL_H_
#define INCLUDE_SCRIMMAGE_MOTION_MOTIONMODEL_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/plugin_manager/Plugin.h>

#include <map>
#include <vector>
#include <string>

namespace scrimmage {

class MotionModel : public Plugin {
 public:
    typedef std::vector<double> vector_t;

    MotionModel();
    std::string type();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);

    virtual bool step(double time, double dt);
    virtual bool posthumous(double t);
    virtual StatePtr &state();
    virtual void set_state(StatePtr &state);
    virtual void teleport(StatePtr &state);
    virtual void set_external_force(Eigen::Vector3d force);
    virtual void set_mass(double mass) { mass_ = mass; }
    virtual double mass() { return mass_; }

 protected:
    void ode_step(double dt);
    virtual void model(const vector_t &x , vector_t &dxdt , double t);

    StatePtr state_;
    vector_t x_;
    vector_t u_;

    Eigen::Vector3d ext_force_;
    double mass_;
};

using MotionModelPtr = std::shared_ptr<MotionModel>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_MOTION_MOTIONMODEL_H_
