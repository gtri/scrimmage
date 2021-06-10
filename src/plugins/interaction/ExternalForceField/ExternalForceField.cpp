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

#include <scrimmage/plugins/interaction/ExternalForceField/ExternalForceField.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <math.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::ExternalForceField,
                ExternalForceField_plugin)

namespace scrimmage {
namespace interaction {

ExternalForceField::ExternalForceField() {
}

bool ExternalForceField::init(std::map<std::string, std::string> &mission_params,
                              std::map<std::string, std::string> &plugin_params) {

    pub_ = advertise("GlobalNetwork", "Force_Field");
    // Parse the force type
    std::string force_type_str = sc::get("force_type", plugin_params, "constant");
    if (force_type_str == "constant") {
        force_type_ = Constant;

        // Parse the constant force
        std::vector<double> force_vec;

        if (str2container(sc::get("constant_force", plugin_params, "0, 0, 0"),
                          ", ", force_vec, 3)) {
            if (force_vec[0] == 99) {
                mag_ = sc::get("constant_mag", plugin_params, mag_);
                ang_ = sc::get("constant_dir", plugin_params, ang_);
                ang_ *= M_PI / 180;
                // calc x y force, set force_v, publish global mag, angle
                force_vec[0] = mag_ * cos(ang_);
                force_vec[1] = mag_ * sin(ang_);
                force_vec[2] = 0;
            }
            // std::cout << "Force is [0]: " << force_vec[0] << ", [1]: " << force_vec[1] << std::endl;
            force_ << force_vec[0], force_vec[1], force_vec[2];
        }
    } else if (force_type_str == "variable") {
        force_type_ = Variable;

        // Parse the force change period
        std::vector<double> force_change_period_vec;
        if (str2container(sc::get("force_change_period", plugin_params, "0.0, 1.0"),
                          ", ", force_change_period_vec, 2)) {
            force_change_period_noise_ =
                parent_->random()->make_rng_normal(force_change_period_vec[0],
                                                   force_change_period_vec[1]);
        } else {
            cout << "Failed to parse force_change_period" << endl;
            return false;
        }

        // Parse the variable force magnitudes
        for (int i = 0; i < 3; i++) {
            std::vector<double> var_force_vec;
            if (str2container(sc::get("variable_force_" + std::to_string(i),
                                      plugin_params, "0.0, 1.0"),
                              ", ", var_force_vec, 2)) {
                auto force_noise =
                    parent_->random()->make_rng_normal(var_force_vec[0],
                                                       var_force_vec[1]);
                force_noise_.push_back(force_noise);
            } else {
                cout << "Failed to parse variable_force_" << i << endl;
                return false;
            }
        }

        // Sample the first variable force
        sample_force();

    } else {
        cout << "Invalid force type: " << force_type_str << endl;
        return false;
    }

    // moment periods
    roll_period_ = sc::get("moment_roll_period", plugin_params, roll_period_);
    pitch_period_ = sc::get("moment_pitch_period", plugin_params, pitch_period_);
    yaw_period_ = sc::get("moment_yaw_period", plugin_params, yaw_period_);

    // moment amplitudes
    roll_amp_ = sc::get("moment_roll_amplitude", plugin_params, roll_amp_);
    pitch_amp_ = sc::get("moment_pitch_amplitude", plugin_params, pitch_amp_);
    yaw_amp_ = sc::get("moment_yaw_amplitude", plugin_params, yaw_amp_);

    return true;
}

bool ExternalForceField::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (force_type_ == Variable) {
        if (time_->t() >= next_sample_time_) {
            sample_force();
        }
    }

    auto msg_force = std::make_shared<sc::Message<Eigen::Vector3d>>();
    msg_force->data = force_;
    pub_->publish(msg_force);

    moment_(0) = roll_amp_ * sin(time_->t() * 2 * M_PI * 1.0 / roll_period_);
    moment_(1) = pitch_amp_ * sin(time_->t() * 2 * M_PI * 1.0 / pitch_period_);
    moment_(2) = yaw_amp_ * sin(time_->t() * 2 * M_PI * 1.0 / yaw_period_);

    for (auto &ent : ents) {
        ent->motion()->set_external_force(force_);

        // Only apply moment force within z-boundary. Scale magnitude.
        if (ent->state_truth()->pos()(2) >= moment_enable_min_z_ &&
            ent->state_truth()->pos()(2) <= moment_enable_max_z_) {
            double scale = (ent->state_truth()->pos()(2) - moment_enable_min_z_) /
                (moment_enable_max_z_ - moment_enable_min_z_);
            ent->motion()->set_external_moment(moment_ * scale);
        }
    }
    return true;
}

void ExternalForceField::sample_force() {
    auto gener = parent_->random()->gener();

    // Sample the force
    for (int i = 0; i < 3; i++) {
        force_[i] = (*force_noise_[i])(*gener);
    }

    // Determine when the next sample time will take place
    next_sample_time_ = time_->t() + (*force_change_period_noise_)(*gener);
}
} // namespace interaction
} // namespace scrimmage
