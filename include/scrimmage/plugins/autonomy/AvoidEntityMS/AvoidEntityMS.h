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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AVOIDENTITYMS_AVOIDENTITYMS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AVOIDENTITYMS_AVOIDENTITYMS_H_

#include <scrimmage/plugins/autonomy/MotorSchemas/BehaviorBase.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {
class AvoidEntityMS : public scrimmage::autonomy::motor_schemas::BehaviorBase {
 public:
    AvoidEntityMS();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    double sphere_of_influence_;
    double minimum_range_;
    bool avoid_non_team_;

    scrimmage_proto::ShapePtr circle_shape_ = std::make_shared<scrimmage_proto::Shape>();
    scrimmage_proto::ShapePtr line_shape_ = std::make_shared<scrimmage_proto::Shape>();

    bool show_shapes_ = false;
};
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AVOIDENTITYMS_AVOIDENTITYMS_H_
