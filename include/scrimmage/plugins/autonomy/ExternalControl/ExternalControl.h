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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <map>
#include <string>
#include <limits>
#include <utility>

namespace scrimmage_proto {
class Action;
class SpaceParams;
class ActionResult;
}

namespace boost {
template <class T> class optional;
}

namespace scrimmage {
class DelayedTask;

namespace autonomy {

class ExternalControlClient;

class ExternalControl : public scrimmage::Autonomy {
 public:
    ExternalControl();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    virtual bool handle_action(
        double t, double dt, const scrimmage_proto::Action &action);
    virtual scrimmage_proto::SpaceParams action_space_params();
    virtual std::pair<bool, double> calc_reward(double t);
    void close(double t) override;
    virtual scrimmage_proto::ActionResult get_observation(double t);

    bool check_action(
        const scrimmage_proto::Action &action,
        uint64_t discrete_action_size,
        uint64_t continuous_action_size);

    std::string server_address_ = "localhost:50051";
    double min_reward_ = -std::numeric_limits<double>::infinity();
    double max_reward_ = std::numeric_limits<double>::infinity();
    bool use_trained_model_ = false;

 private:
    boost::optional<scrimmage_proto::Action>
        send_action_result(scrimmage_proto::ActionResult &action_result,
            double reward, bool done);
    bool send_env();

    std::shared_ptr<ExternalControlClient> external_control_client_;
    std::shared_ptr<DelayedTask> delayed_task_;
    double curr_reward = 0;
    bool env_sent_ = false;

    int num_attempts_ = 0;
    std::string python_cmd_ = "external_control.py";
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_
