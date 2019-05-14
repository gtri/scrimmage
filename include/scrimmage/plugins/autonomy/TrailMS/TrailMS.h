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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAILMS_TRAILMS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAILMS_TRAILMS_H_
#include <scrimmage/plugins/autonomy/MotorSchemas/BehaviorBase.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class TrailMS : public scrimmage::autonomy::motor_schemas::BehaviorBase {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    int trail_id_ = -1;
    double trail_range_ = 5.0;
    bool show_track_point_ = false;

    Eigen::AngleAxisd aa_angle_az_;
    Eigen::AngleAxisd aa_angle_elev_;

    scrimmage_proto::ShapePtr sphere_shape_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAILMS_TRAILMS_H_
