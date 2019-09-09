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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALFORCEFIELD_EXTERNALFORCEFIELD_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALFORCEFIELD_EXTERNALFORCEFIELD_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>

#include <map>
#include <list>
#include <string>
#include <vector>
#include <random>
#include <memory>

namespace scrimmage {
namespace interaction {

class ExternalForceField : public scrimmage::EntityInteraction {
 public:
    enum ForceType {
        Constant,
        Variable
    };
    ExternalForceField();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;

 protected:
    PublisherPtr pub_;
    ForceType force_type_ = Constant;
    Eigen::Vector3d force_ = Eigen::Vector3d::Zero();

    std::shared_ptr<std::normal_distribution<double>> force_change_period_noise_;
    std::vector<std::shared_ptr<std::normal_distribution<double>>> force_noise_;

    double next_sample_time_ = -1.0;
    void sample_force();

    double moment_enable_min_z_ = -10;
    double moment_enable_max_z_ = 0.5;
    double roll_period_ = 1.0;
    double pitch_period_ = 1.0;
    double yaw_period_ = 1.0;
    double roll_amp_ = 0.0;
    double pitch_amp_ = 0.0;
    double yaw_amp_ = 0.0;
    double mag_ = 0.0;
    double ang_ = 0.0;
    Eigen::Vector3d moment_ = Eigen::Vector3d::Zero();

 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_EXTERNALFORCEFIELD_EXTERNALFORCEFIELD_H_
