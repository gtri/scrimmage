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

#include "py_openai_env.h"

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>

// temporary includes for scrimmage_memory_cleanup
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/math/State.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/log/Log.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <signal.h>

#include <string>
#include <algorithm>
#include <limits>
#include <exception>
#include <chrono> // NOLINT
#include <signal.h>

#include <boost/optional.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <GeographicLib/LocalCartesian.hpp>

namespace py = pybind11;
namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;
namespace sc_auto = scrimmage::autonomy;

namespace {
// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

ScrimmageOpenAIEnv::ScrimmageOpenAIEnv(
        const std::string &mission_file,
        bool enable_gui,
        bool combine_actors,
        bool global_sensor,
        double timestep) :
    spec(py::none()),
    metadata(py::dict()),
    mission_file_(mission_file),
    enable_gui_(enable_gui) {

    observations_.set_global_sensor(global_sensor);
    observations_.set_combine_actors(combine_actors);

    delayed_task_.delay = timestep;

    py::module warnings_module = py::module::import("warnings");
    warning_function_ = warnings_module.attr("warn");

    mp_ = std::make_shared<sc::MissionParse>();
    reset_scrimmage(false);

    tuple_space_ = sc_auto::get_gym_space("Tuple");
    discrete_space_ = sc_auto::get_gym_space("Discrete");
    multidiscrete_space_ = sc_auto::get_gym_space("MultiDiscrete");
    box_space_ = sc_auto::get_gym_space("Box");
}

void ScrimmageOpenAIEnv::set_reward_range() {
    reward_range = pybind11::tuple(2);
    if (actions_.ext_ctrl_vec().size() == 1) {
        reward_range[0] = actions_.ext_ctrl_vec()[0]->reward_range.first;
        reward_range[1] = actions_.ext_ctrl_vec()[0]->reward_range.second;
    } else {

        auto get_min_reward = [&](auto &a) {return a->reward_range.first;};
        auto get_max_reward = [&](auto &a) {return a->reward_range.second;};
        auto min_reward = actions_.ext_ctrl_vec() | ba::transformed(get_min_reward);
        auto max_reward = actions_.ext_ctrl_vec() | ba::transformed(get_max_reward);

        if (observations_.get_combine_actors()) {
            reward_range[0] = boost::accumulate(min_reward, 0.0);
            reward_range[1] = boost::accumulate(max_reward, 0.0);
        } else {
            double mx = -std::numeric_limits<double>::infinity();
            double mn = std::numeric_limits<double>::infinity();

            for (auto r : min_reward) mn = std::min(mn, r);
            for (auto r : max_reward) mx = std::max(mx, r);

            reward_range[0] = mn;
            reward_range[1] = mx;
        }
    }
}

void ScrimmageOpenAIEnv::render(const std::string &/*mode*/) {
    warning_function_("render must be set in gym.make with enable_gui kwargs");
}

bool ScrimmageOpenAIEnv::is_gym_instance(pybind11::object &obj, const std::string &type) {
    py::object cls = sc_auto::get_gym_space(type);
    return PyObject_IsInstance(obj.ptr(), cls.ptr());
}

pybind11::object ScrimmageOpenAIEnv::reset() {

    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    shutdown_handler = [&](int /*s*/){
        std::cout << std::endl << "Exiting gracefully" << std::endl;
        simcontrol_->force_exit();
        if (enable_gui_ && system("pkill scrimmage-viz")) {
            // ignore error
        }
        throw std::exception();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    reset_scrimmage(enable_gui_);
    observations_.update_observation(actions_.ext_ctrl_vec().size());
    return observations_.observation;
}

void ScrimmageOpenAIEnv::scrimmage_memory_cleanup() {
    // temporary fix until scrimmage uses weak pointers
    actions_.ext_ctrl_vec().clear();
    observations_.ext_sensor_vec().clear();

    if (!simcontrol_) return;

    for (auto e : simcontrol_->ents()) {
        auto s = std::make_shared<sc::State>();
        if (e->motion()) {
            e->motion()->set_state(s);
            e->motion()->set_parent(nullptr);
            e->motion()->set_pubsub(nullptr);
            e->motion()->set_time(std::make_shared<sc::Time>());
        }

        for (auto &kv : e->sensors()) {
            kv.second->set_parent(nullptr);
            kv.second->set_pubsub(nullptr);
            kv.second->set_time(std::make_shared<sc::Time>());
        }

        for (auto c : e->controllers()) {
            c->set_parent(nullptr);
            c->set_time(std::make_shared<sc::Time>());
            c->set_pubsub(nullptr);
            auto ds = std::make_shared<sc::State>();
            c->set_desired_state(ds);
        }

        for (auto a : e->autonomies()) {
            auto r = std::make_shared<sc::RTree>();
            a->set_rtree(r);
            a->set_parent(nullptr);
            auto p = std::make_shared<GeographicLib::LocalCartesian>();
            a->set_projection(p);
            a->set_pubsub(nullptr);
            a->set_time(std::make_shared<sc::Time>());
            auto s = std::make_shared<sc::State>();
            a->set_state(s);
            auto ds = std::make_shared<sc::State>();
            a->set_desired_state(ds);
            auto c = std::make_shared<sc::ContactMap>();
            a->set_contacts(c);
        }

        e->autonomies().clear();
        e->motion() = nullptr;
        e->controllers().clear();
        e->set_mp(nullptr);
        e->state() = nullptr;
        e->contacts() = nullptr;
        e->plugin_manager() = nullptr;
        e->file_search() = nullptr;
        e->pubsub() = nullptr;
        e->rtree() = nullptr;
        e->set_time_ptr(nullptr);
        e->set_random(nullptr);
        e->contact_visual() = nullptr;
        e->sensors().clear();
        e->services().clear();
        e->properties().clear();
        e->set_projection(nullptr);
    }
}

void ScrimmageOpenAIEnv::reset_scrimmage(bool enable_gui) {
    scrimmage_memory_cleanup();
    mp_ = std::make_shared<sc::MissionParse>();
    simcontrol_ = std::make_shared<sc::SimControl>();
    if (!mp_->parse(mission_file_)) {
        std::cout << "Failed to parse file: " << mission_file_ << std::endl;
    }
    if (seed_set_) {
        mp_->params()["seed"] = std::to_string(seed_);
    }
    if (enable_gui) {
        mp_->set_network_gui(true);
        mp_->set_enable_gui(false);

        // TODO: fix this to get feedback that the gui is running
        if (system("scrimmage-viz &")) {
            std::cout << "could not start scrimmage-viz" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } else {
        mp_->set_time_warp(0);
    }
    // users may have forgotten to turn off nonlearning_mode.
    // If this code is being called then it should never be nonlearning_mode
    reset_learning_mode();

    log_ = sc::preprocess_scrimmage(mp_, *simcontrol_);
    simcontrol_->start_overall_timer();
    simcontrol_->set_time(mp_->t0());
    if (enable_gui) {
        simcontrol_->pause(mp_->start_paused());
    } else {
        simcontrol_->pause(false);
    }
    delayed_task_.last_updated_time = -std::numeric_limits<double>::infinity();
    if (log_ == nullptr) {
        py::print("scrimmage initialization unsuccessful");
    }

    loop_number_ = 0;
    if (!simcontrol_->generate_entities(0)) {
        py::print("scrimmage entity generation unsuccessful");
    }

    actions_.ext_ctrl_vec().clear();
    observations_.ext_sensor_vec().clear();
    for (auto &e : simcontrol_->ents()) {
        auto cast = [&](auto &p) {return std::dynamic_pointer_cast<sc_auto::ScrimmageOpenAIAutonomy>(p);};
        auto good_ptr = [&](auto &p) {return cast(p) != nullptr;};
        for (const auto &a : e->autonomies() | ba::filtered(good_ptr) | ba::transformed(cast)) {
            actions_.ext_ctrl_vec().push_back(a);
            observations_.add_sensors(a->parent()->sensors());
        }
    }
    observations_.create_observation_space(actions_.ext_ctrl_vec().size());
    actions_.create_action_space(observations_.get_combine_actors());
    set_reward_range();
}

void ScrimmageOpenAIEnv::close() {
    postprocess_scrimmage(mp_, *simcontrol_, log_);
    if (enable_gui_ && system("pkill scrimmage-viz")) {
        // ignore error
    }
}

void ScrimmageOpenAIEnv::seed(pybind11::object _seed) {
    seed_set_ = true;
    seed_ = _seed.cast<int>();
}

pybind11::tuple ScrimmageOpenAIEnv::step(pybind11::object action) {

    actions_.distribute_action(action, observations_.get_combine_actors());

    delayed_task_.update(simcontrol_->t());
    bool done = !simcontrol_->run_single_step(loop_number_++) ||
        simcontrol_->end_condition_reached();
    while (!done && !delayed_task_.update(simcontrol_->t()).first) {
        done = !simcontrol_->run_single_step(loop_number_++) ||
        simcontrol_->end_condition_reached();
    }

    observations_.update_observation(actions_.ext_ctrl_vec().size());

    py::float_ py_reward;
    py::bool_ py_done;
    py::dict py_info;
    std::tie(py_reward, py_done, py_info) = calc_reward();

    done |= py_done.cast<bool>();
    py_done = py::bool_(done);

    if (done && enable_gui_ && system("pkill scrimmage-viz")) {
        // ignore error
    }

    return py::make_tuple(observations_.observation, py_reward, py_done, py_info);
}

void ScrimmageOpenAIEnv::reset_learning_mode() {
    // users may have forgotten to turn off nonlearning_mode.
    // If this code is being called then it should never be nonlearning_mode
    for (auto &kv : mp_->entity_descriptions()) {
        int id = kv.first;
        auto &desc_map = kv.second;
        auto &attrs = mp_->entity_attributes()[id];

        size_t autonomy_ct = 0;
        auto autonomy_str = [&](int ct){return std::string("autonomy") + std::to_string(ct);};
        auto find_autonomy = [&](int ct) {return desc_map.find(autonomy_str(ct));};

        auto it = find_autonomy(autonomy_ct);
        while (it != desc_map.end()) {
            attrs[autonomy_str(autonomy_ct++)]["nonlearning_mode_openai_plugin"] = "false";
            it = find_autonomy(autonomy_ct);
        }
    }
}

std::tuple<pybind11::float_, pybind11::bool_, pybind11::dict> ScrimmageOpenAIEnv::calc_reward() {

    py::dict info;
    py::list info_done;
    py::list info_reward;

    double reward = 0;
    bool done = false;

    for (auto &a : actions_.ext_ctrl_vec()) {
        double temp_reward;
        bool temp_done;
        std::tie(temp_done, temp_reward) = a->calc_reward();

        reward += temp_reward;
        done |= temp_done;

        info_reward.append(temp_reward);
        info_done.append(temp_done);
    }

    info["reward"] = info_reward;
    info["done"] = info_done;

    return std::make_tuple(py::float_(reward), py::bool_(done), info);
}

void add_openai_env(pybind11::module &m) {
    py::class_<ScrimmageOpenAIEnv>(m, "ScrimmageOpenAIEnv")
        .def(py::init<std::string&, bool, bool, bool, double>(),
            R"(Scrimmage Open AI Environment Constructor.

Parameters
----------
mission_file : str
    the mission file to be run (does not need the full path, e.g. \"straight.xml\"

enable_gui : bool
    whether to start a gui. This will run scrimmage-viz in the
    background.  Make sure your mission file is set to output to
    localhost:50051, e.g.,

      <stream_port>50051</stream_port>
      <stream_ip>localhost</stream_ip>

combine_actors : bool
    whether to present the actions and observations of multiple
    learners as a single action/observation.
    This is only relevant when there are more than one learner in a
    scenario. See test_openai.py for use examples.

global_sensor : bool
    whether to only use the first single observation. This is only
    relevant when there are multiple learners and combine_actors is
    set to False. It is designed to be used when
    agents all have the same information.

timestep : float
    run scrimmage for multiple timesteps before outputting
    the observation and reward on env.step().
)",
            py::arg("mission_file"),
            py::arg("enable_gui") = false,
            py::arg("combine_actors") = false,
            py::arg("global_sensor") = false,
            py::arg("timestep") = -1)
        .def("step", &ScrimmageOpenAIEnv::step,
            R"(Run scrimmage for one step.

The return is dependent on combine_actors and global_sensor.

obs : varies
    * nump.array - observation_space is Discrete, MultiDiscrete, or Box
    * length 2 list of np arrays - observation space is Tuple

    When there are multiple learners, the output will be a list
    with one element for each learner. Each element of the list
    will correspond to the logic above for each agent.

reward : float

done : bool

info : dict
   info[\"reward\"] and info[\"done\"]
   is a list of the individual rewards and done status for each learner.
   When there is a single learner, these lists have length 1.
)")
        .def_readwrite("reward_range", &ScrimmageOpenAIEnv::reward_range)
        .def_property("action_space", &ScrimmageOpenAIEnv::get_action_space, &ScrimmageOpenAIEnv::set_action_space)
        .def_property("observation_space", &ScrimmageOpenAIEnv::get_observation_space, &ScrimmageOpenAIEnv::set_observation_space)
        .def_property("observation", &ScrimmageOpenAIEnv::get_observation, &ScrimmageOpenAIEnv::set_observation)
        .def_readwrite("spec", &ScrimmageOpenAIEnv::spec)
        .def_readwrite("metadata", &ScrimmageOpenAIEnv::metadata)
        .def("reset", &ScrimmageOpenAIEnv::reset, "restart scrimmage")
        .def("close", &ScrimmageOpenAIEnv::close, "closes scrimmage")
        .def("seed", &ScrimmageOpenAIEnv::seed, "Set the seed used in the mission file")
        .def("render", &ScrimmageOpenAIEnv::render,
            "no-op. Use \"enable_gui\" if you want to see a visual",
            py::arg("mode") = "human")
        .def_property_readonly("unwrapped", &ScrimmageOpenAIEnv::get_this);
}
