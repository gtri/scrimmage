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

#include <scrimmage/plugins/autonomy/MotorSchemas/BehaviorBase.h>

#include <scrimmage/parse/ParseUtils.h>

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {

BehaviorBase::BehaviorBase() : desired_vector_(Eigen::Vector3d(0, 0, 0)), gain_(1.0),
                               max_vector_length_(1.0) {
}

Eigen::Vector3d &BehaviorBase::desired_vector() {
    return desired_vector_;
}

void BehaviorBase::set_gain(const double &gain) {
    gain_ = gain;
}

const double &BehaviorBase::gain() {
    return gain_;
}

void BehaviorBase::set_max_vector_length(const double &max_vector_length) {
    max_vector_length_ = max_vector_length;
}

void BehaviorBase::configure_contacts(std::map<std::string, std::string> &params) {
    std::vector<std::string> use_contacts;
    if (get_vec("contacts", params, ", ", use_contacts)) {
        use_truth_contacts_ = false;
        use_noisy_contacts_ = false;
        for (auto &str : use_contacts) {
            if (str == "truth") {
                use_truth_contacts_ = true;
            } else if (str == "noisy") {
                use_noisy_contacts_ = true;
            }
        }
    }

    if (use_noisy_contacts_) {
        auto cnt_cb = [&] (scrimmage::MessagePtr<ContactMap> &msg) {
                          noisy_contacts_ = msg->data;
                      };
        subscribe<ContactMap>("LocalNetwork", "ContactsWithCovariances", cnt_cb);
    }
}

} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
