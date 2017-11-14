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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE3D_UNICYCLE3D_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE3D_UNICYCLE3D_H_

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <string>

namespace scrimmage {
namespace motion {
class Unicycle3D : public scrimmage::MotionModel {
 public:
    Unicycle3D();
    ~Unicycle3D();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);

    virtual bool step(double t, double dt);

    virtual void model(const vector_t &x , vector_t &dxdt , double t);

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::Vector3d &u() = 0;
    };

 protected:
    Eigen::Vector3d ctrl_u_;
    double turn_rate_max_;
    double pitch_rate_max_;
    double vel_max_;
    bool enable_roll_;

    scrimmage::Quaternion quat_world_;
    scrimmage::Quaternion quat_local_;

    bool write_csv_;
    CSV csv_;
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_UNICYCLE3D_UNICYCLE3D_H_
