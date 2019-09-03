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
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/SimpleINS/SimpleINS.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/msgs/GPS.pb.h>

#include <vector>

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::SimpleINS, SimpleINS_plugin)

namespace scrimmage {
namespace sensor {

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
    pub_cep_ = advertise("LocalNetwork", "INS_CEP");
    pub_seastate_ = advertise("LocalNetwork", "INS_SS");

    // get xml info
    sea_state_gps_ = sc::get<double>("sea_state_gps", params, 99);

    // GPS information
    auto gps_cb = [&](auto &msg) {
        gps_fix_ = msg->data.fixed();
    };
    subscribe<sm::GPSStatus>("GlobalNetwork", "GPSStatus", gps_cb);


    surface_timer_ = sc::get<double>("surface_timer", params, 10);

    parent_->state() = std::make_shared<State>();
    *(parent_->state()) = *(parent_->state_truth());
}


bool SimpleINS::step() {
    auto gener = parent_->random()->gener();
    auto msg_SS = std::make_shared<sc::Message<double>>();
    msg_SS->data = sea_state_gps_;
    pub_seastate_->publish(msg_SS);

    // Make a copy of the current state
    StateWithCovariance ns(*(parent_->state_truth()));

    auto msg = std::make_shared<Message<StateWithCovariance>>();

    if (init_m_) {
        m_ = Eigen::MatrixXd::Identity(ns.covariance().rows(), ns.covariance().cols());
        init_m_ = false;
    }


      // Acceleration
        accel_ = ns.vel() - vel_Nminus1;
        vel_Nminus1 = ns.vel();
    for (int i = 0; i < 3; i++) {
        accel_(i) = abs(accel_(i) / .01);
        if (accel_(i) > 1) {
            accel_(i) = 1;
            }
    }


    // Use gen in order to create a growing covariance
    double pos_noise_0 = (*pos_noise_[0])(*gener) * (accel_(0));
    double pos_noise_1 = (*pos_noise_[1])(*gener) * (accel_(1));
    const int arrayNum[4] = {15, 30, 45, 60};

    if (!gps_fix_) {
        m_(0, 0) += (pos_noise_0 < 0 ? -1*pos_noise_0 : pos_noise_0);
        m_(1, 1) += (pos_noise_1 < 0 ? -1*pos_noise_1 : pos_noise_1);

        prev_time_ = time_->t();
        int randIdx = rand() % 4;
        surface_timer_ = arrayNum[randIdx];
        if (sea_state_gps_ > 1) {
            surface_timer_ += 60;
        }
    } else {
        // GPS fix on - wait on surface timer and snap back to an identity matrix
        m_(0, 0) += (pos_noise_0 < 0 ? -1*pos_noise_0 : pos_noise_0);
        m_(1, 1) += (pos_noise_1 < 0 ? -1*pos_noise_1 : pos_noise_1);
        // GPS fix on - wait on surface timer and snap back to an identity matrix
        if (time_->t() - prev_time_ > surface_timer_) {
           m_ = Eigen::MatrixXd::Identity(ns.covariance().rows(), ns.covariance().cols());
           // Reset the position error accumulator
           pos_error_accum_ = Eigen::Vector3d::Zero();
        }
    }



    // Create noisy position / velocity
    for (int i = 0; i < 3; i++) {
         // std::cout << "Accelration for " << i << ", is: " << (accel_(i)) << std::endl;
         pos_error_accum_(i) += (*pos_noise_[i])(*gener) * (accel_(i));
         // msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener);
        msg->data.vel()(i) = ns.vel()(i) + (*vel_noise_[i])(*gener);
    }
      msg->data.pos() = ns.pos() + pos_error_accum_;

    msg->data.quat() = ns.quat()
    * Quaternion(Eigen::Vector3d::UnitX(), (*orient_noise_[0])(*gener))
    * Quaternion(Eigen::Vector3d::UnitY(), (*orient_noise_[1])(*gener))
    * Quaternion(Eigen::Vector3d::UnitZ(), (*orient_noise_[2])(*gener));

    msg->data.set_covariance(m_);

    double sig_L;
    double sig_S;

    if (m_(0, 0) < m_(1, 1)) {
        sig_L = sqrt(m_(1, 1));
        sig_S = sqrt(m_(0, 0));
    } else {
        sig_L = sqrt(m_(0, 0));
        sig_S = sqrt(m_(1, 1));
    }
    double sig_w = sig_S / sig_L;
    double CEP;
    if (sig_w < 0.5) {
        CEP = sig_L * (0.67 + 0.8 * sig_w * sig_w);
    } else {
        CEP = 0.59 * sig_L * (1 + sig_w);
    }
    auto msg_cep = std::make_shared<sc::Message<double>>();
        msg_cep->data = CEP;
        pub_cep_->publish(msg_cep);
    // Publish StateWCovariance msg
    pub_->publish(msg);

    // Update the entity's state.
    *(parent_->state()) = static_cast<sc::State>(msg->data);

    return true;
}

} // namespace sensor
} // namespace scrimmage
