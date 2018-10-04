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

#include <pybind11/pybind11.h>

#include <scrimmage/common/DelayedTask.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIObservations.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIActions.h>

#include <string>
#include <memory>
#include <vector>
#include <thread> // NOLINT
#include <tuple>

namespace scrimmage {
class MissionParse;
using MissionParsePtr = std::shared_ptr<MissionParse>;

class SimControl;
class Log;

namespace autonomy {
class ScrimmageOpenAIAutonomy;
} // namespace autonomy
} // namespace scrimmage

class ScrimmageOpenAIEnv {
 public:

    ScrimmageOpenAIEnv() {}
    ScrimmageOpenAIEnv(const std::string &mission_file,
                       bool enable_gui = false,
                       bool combine_actors = false,
                       bool global_sensor = false,
                       double timestep = -1);

    pybind11::tuple step(pybind11::object action);
    pybind11::object reset();

    void close();
    void render(const std::string &mode = "human");
    void seed(pybind11::object _seed = pybind11::none());
    ScrimmageOpenAIEnv *get_this() {return this;}

    pybind11::object get_observation_space() {return observations_.observation_space;}
    void set_observation_space(pybind11::object observation_space) {observations_.observation_space = observation_space;}

    pybind11::object get_action_space() {return actions_.action_space;}
    void set_action_space(pybind11::object action_space) {actions_.action_space = action_space;}

    pybind11::object get_observation() {return observations_.observation;}
    void set_observation(pybind11::object observation) {observations_.observation = observation;}

    bool get_global_sensor() {return observations_.get_global_sensor();}
    void set_global_sensor(bool global_sensor) {observations_.set_global_sensor(global_sensor);}

    bool get_combine_actors() {return observations_.get_combine_actors();}
    void set_combine_actors(bool combine_actors) {observations_.set_combine_actors(combine_actors);}

    pybind11::object spec;
    pybind11::object metadata;

    pybind11::object env;
    pybind11::tuple reward_range;

 protected:
    pybind11::object warning_function_;

    std::string mission_file_ = "";
    bool enable_gui_ = false;
    scrimmage::DelayedTask delayed_task_;

    std::thread thread_;

    std::shared_ptr<scrimmage::SimControl> simcontrol_;
    std::shared_ptr<scrimmage::Log> log_;
    scrimmage::MissionParsePtr mp_;
    bool seed_set_ = false;
    int seed_;

    using ExternalControlPtr = std::shared_ptr<scrimmage::autonomy::ScrimmageOpenAIAutonomy>;

    scrimmage::autonomy::OpenAIObservations observations_;
    scrimmage::autonomy::OpenAIActions actions_;

    void set_reward_range();
    void update_observation();
    std::tuple<pybind11::float_, pybind11::bool_, pybind11::dict> calc_reward();
    void reset_scrimmage(bool enable_gui);
    void scrimmage_memory_cleanup();
    void reset_learning_mode();
    int loop_number_ = 0;

    bool is_gym_instance(pybind11::object &obj, const std::string &type);

    pybind11::object tuple_space_;
    pybind11::object discrete_space_;
    pybind11::object multidiscrete_space_;
    pybind11::object box_space_;

 private:

};
