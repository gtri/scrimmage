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

#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/math/State.h>

#include <limits>

namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy, ExternalControl, ExternalControl_plugin)

bool ExternalControl::step_autonomy(double t, double dt) {
    if (!env_sent_) {
        external_control_client_ =
            ExternalControlClient(grpc::CreateChannel(
                server_address_, grpc::InsecureChannelCredentials()));
        env_sent_ = true;
        send_env();
    }

    auto action = send_action_result(t, 0, false);
    if (!action) {
        std::cout << "error receiving external action" << std::endl;
        return false;
    }

    return handle_action(t, dt, *action);
}

bool ExternalControl::handle_action(
        double t, double dt, scrimmage_proto::Action action) {
    if (action.continuous_size() != 9) {
        std::cout << "recieved external action "
            << " of length " << action.continuous_size()
            << "  (need length 9 to match state)" << std::endl;
        return false;
    }

    const auto c = action.continuous();
    desired_state_->pos() <<
        action.continuous(0), action.continuous(1), action.continuous(2);
    desired_state_->vel() <<
        action.continuous(3), action.continuous(4), action.continuous(5);
    desired_state_->quat().set(
        action.continuous(6), action.continuous(7), action.continuous(8));
    return true;
}

boost::optional<scrimmage_proto::Action>
ExternalControl::send_action_result(double t, double reward, bool done) {
    sp::ActionResult action_result;
    sp::SpaceSample *obs = action_result.mutable_observations();

    for (auto &kv : parent_->sensors()) {
        auto msg = kv.second->sensor_msg_flat(t);
        if (msg) {
            sp::SpaceSample &sample = (*msg)->data;
            for (int i = 0; i < sample.value_size(); i++) {
                obs->add_value(sample.value(i));
            }
        }
    }

    action_result.set_reward(reward);
    action_result.set_done(done);

    return external_control_client_.send_action_result(action_result);
}

bool ExternalControl::send_env() {
    sp::Environment env;

    *env.mutable_action_spaces() = action_space_params();

    sp::SpaceParams *obs_space = env.mutable_observation_spaces();

    for (auto &kv : parent_->sensors()) {
        auto obs_space_params = kv.second->observation_space_params();
        if (obs_space_params) {
            for (const sp::SingleSpaceParams params : obs_space_params->params()) {
                *obs_space->add_params() = params;
            }
        }
    }

    env.set_min_reward(min_reward_);
    env.set_max_reward(max_reward_);

    return external_control_client_.send_environment(env, desired_state_);
}

scrimmage_proto::SpaceParams ExternalControl::action_space_params() {
    scrimmage_proto::SpaceParams space_params;
    scrimmage_proto::SingleSpaceParams *single_space_params =
        space_params.add_params();
    single_space_params->set_num_dims(9);

    // we have to use non-infinite bounds for openai's tuple_space to give
    // non-nan values when the sample method on tuple_space is called
    const double inf = 1e9;
    for (int i = 0; i < 6; i++) {
        single_space_params->add_minimum(-inf);
        single_space_params->add_maximum(inf);
    }

    // roll
    single_space_params->add_minimum(-M_PI);
    single_space_params->add_maximum(M_PI);

    // pitch
    single_space_params->add_minimum(-M_PI / 2);
    single_space_params->add_maximum(M_PI / 2);

    // yaw
    single_space_params->add_minimum(-M_PI / 2);
    single_space_params->add_maximum(M_PI / 2);

    return space_params;
}
