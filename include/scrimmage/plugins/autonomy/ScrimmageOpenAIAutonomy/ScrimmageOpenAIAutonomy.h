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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/Visibility.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIObservations.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIActions.h>

#include <pybind11/pybind11.h>

#include <map>
#include <vector>
#include <string>
#include <utility>

namespace scrimmage {

struct EnvParams {
    std::vector<double> discrete_count;
    std::vector<std::pair<double, double>> continuous_extrema;
};

struct EnvValues {
    std::vector<int> discrete;
    std::vector<double> continuous;
};

namespace autonomy {

class DLL_PUBLIC ScrimmageOpenAIAutonomy : public scrimmage::Autonomy {
 public:
    ScrimmageOpenAIAutonomy();

    // normal overrides
    void init(std::map<std::string, std::string> &params) final;
    bool step_autonomy(double t, double dt) final;

    // new overrides
    virtual void init_helper(std::map<std::string, std::string> &/*params*/) {}
    virtual bool step_helper() {return true;}

    virtual void set_environment() {}
    virtual std::pair<bool, double> calc_reward();
    std::pair<double, double> reward_range;
    EnvParams action_space;
    EnvValues action;

 private:
    // nonlearning mode variables
    bool first_step_ = true;
    PublisherPtr pub_reward_;
    pybind11::object actor_func_;
    bool nonlearning_mode_ = false;
    OpenAIObservations observations_;
    OpenAIActions actions_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
