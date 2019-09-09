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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_MOTORSCHEMAS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_MOTORSCHEMAS_H_

#include <scrimmage/plugins/autonomy/MotorSchemas/BehaviorBase.h>

#include <scrimmage/autonomy/Autonomy.h>

#include <map>
#include <string>
#include <list>
#include <memory>

namespace scrimmage {
namespace autonomy {
class MotorSchemas : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    bool show_shapes_;
    bool pub_vel_vec_;
    double max_speed_;

    bool add_lower_bound_to_vz_;
    double vz_lower_bound_;

    int desired_heading_idx_ = 0;
    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;

    int output_vel_x_idx_ = 0;
    int output_vel_y_idx_ = 0;
    int output_vel_z_idx_ = 0;

    std::string current_state_ = "UNDEFINED_NO_STATE";

    std::map<std::string, std::list<motor_schemas::BehaviorBasePtr>> behaviors_;
    std::list<motor_schemas::BehaviorBasePtr> default_behaviors_;
    std::list<motor_schemas::BehaviorBasePtr> current_behaviors_;

    scrimmage_proto::ShapePtr line_shape_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_MOTORSCHEMAS_H_
