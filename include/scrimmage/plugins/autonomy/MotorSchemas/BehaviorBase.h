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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>

#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {
class BehaviorBase : public scrimmage::Autonomy {
 public:
    BehaviorBase();
    Eigen::Vector3d &desired_vector();
    void set_gain(const double &gain);
    const double &gain();
    void set_max_vector_length(const double &max_vector_length);
    void configure_contacts(std::map<std::string, std::string> &params);

 protected:
    Eigen::Vector3d desired_vector_;
    double gain_;
    double max_vector_length_;

    // Use truth contacts by default, unless 'contacts' parameter is set
    bool use_truth_contacts_ = true;
    bool use_noisy_contacts_ = false;

    ContactMap noisy_contacts_;
};
using BehaviorBasePtr = std::shared_ptr<BehaviorBase>;
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOTORSCHEMAS_BEHAVIORBASE_H_
