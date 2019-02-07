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
 * @version 0.1.0:96
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/SimpleINS/SimpleINS.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/msgs/GPS.pb.h>

#include <vector>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;


REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::SimpleINS, SimpleINS_plugin)

namespace scrimmage {
namespace autonomy {

void SimpleINS::init(std::map<std::string, std::string> &params) {
    // Noise params information
    auto add_noise = [&](std::string prefix, auto &noise_vec) {
        for (int i = 0; i < 3; i++) {
            std::string tag_name = prefix + "_" + std::to_string(i);
            std::vector<double> vec;
            double mean, stdev;
            std::tie(mean, stdev) = get_vec(tag_name, params, " ", vec, 2) ?
                std::make_pair(vec[0], vec[1]) : std::make_pair(0.0, 1.0);
            noise_vec.push_back(parent_->random()->make_rng_normal(mean, stdev));
        }
    };

    add_noise("pos_noise", pos_noise_);
    add_noise("vel_noise", vel_noise_);
    add_noise("orient_noise", orient_noise_);

    pub_ = advertise("LocalNetwork", "StateWithCovariance");

    // GPS information
    auto gps_cb = [&](auto &msg) {
        gps_fix_ = msg->data.fixed();
    };
    subscribe<sm::GPSStatus>("GlobalNetwork", "GPSStatus", gps_cb);

    surface_timer_ = sc::get<double>("surface_timer", params, 10);

    // VariableIO information
    using Type = VariableIO::Type;
    using Dir = VariableIO::Direction;

    uint8_t output_vel_x_idx_ = vars_.declare(Type::velocity_x, Dir::Out);
    uint8_t output_vel_y_idx_ = vars_.declare(Type::velocity_y, Dir::Out);
    uint8_t output_vel_z_idx = vars_.declare(Type::velocity_z, Dir::Out);

    vars_.output(output_vel_x_idx_, 0);
    vars_.output(output_vel_y_idx_, 0);
    vars_.output(output_vel_z_idx, 0);

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
}


bool SimpleINS::step_autonomy(double t, double dt) {
    auto gener = parent_->random()->gener();

    // Make a copy of the current state
    StateWithCovariance ns(*(parent_->state()));

    auto msg = std::make_shared<Message<StateWithCovariance>>();

    if (init_m_) {
        m_ = Eigen::MatrixXd::Identity(ns.covariance().rows(), ns.covariance().cols());
        init_m_ = false;
    }

    // Create noise position to send back.
    for (int i = 0; i < 3; i++) {
        msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener);
        msg->data.vel()(i) = ns.vel()(i) + (*vel_noise_[i])(*gener);
    }

    msg->data.quat() = ns.quat()
    * Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener))
    * Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener))
    * Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener));

    // Use gen in order to create a growing covariance
    double pos_noise_0 = (*pos_noise_[0])(*gener);
    double pos_noise_1 = (*pos_noise_[1])(*gener);

    if (!gps_fix_) {
        m_(0, 0) += (pos_noise_0 < 0 ? -1*pos_noise_0 : pos_noise_0);
        m_(1, 1) += (pos_noise_1 < 0 ? -1*pos_noise_1 : pos_noise_1);

        prev_time_ = t;
    } else {
        // GPS fix on - wait on surface timer and snap back to an identity matrix
        if (t - prev_time_ > surface_timer_) {
           m_ = Eigen::MatrixXd::Identity(ns.covariance().rows(), ns.covariance().cols());
        }
    }

    msg->data.set_covariance(m_);

    // Publish StateWCovariance msg
    pub_->publish(msg);
    return true;
}

} // namespace autonomy
} // namespace scrimmage
