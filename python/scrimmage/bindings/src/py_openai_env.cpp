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
#include <scrimmage/viewer/Viewer.h>

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

#include <string>
#include <algorithm>
#include <limits>
#include <exception>
#include <chrono> // NOLINT

#include <boost/optional.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/min_element.hpp>

#include <GeographicLib/LocalCartesian.hpp>

namespace py = pybind11;
namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

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
    combine_actors_(combine_actors),
    global_sensor_(global_sensor),
    enable_gui_(enable_gui) {

    delayed_task_.delay = timestep;

    py::module warnings_module = py::module::import("warnings");
    warning_function_ = warnings_module.attr("warn");

    mp_ = std::make_shared<sc::MissionParse>();
    reset_scrimmage(false);

    tuple_space_ = get_gym_space("Tuple");
    discrete_space_ = get_gym_space("Discrete");
    multidiscrete_space_ = get_gym_space("MultiDiscrete");
    box_space_ = get_gym_space("Box");
    asarray_ = py::module::import("numpy").attr("asarray");
}

void ScrimmageOpenAIEnv::to_continuous(std::vector<std::pair<double, double>> &p,
                                       pybind11::list &minima,
                                       pybind11::list &maxima) {
    for (auto &value : p) {
        py::list min_max;
        minima.append(value.first);
        maxima.append(value.second);
    }
}

void ScrimmageOpenAIEnv::to_discrete(std::vector<double> &p, py::list &maxima) {
    for (auto &value : p) {
        maxima.append(value);
    }
}

void ScrimmageOpenAIEnv::create_action_space() {

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &a : ext_ctrl_vec_) {
            a->set_environment();
            to_discrete(a->action_space.discrete_count, discrete_count);
            to_continuous(a->action_space.continuous_extrema, continuous_minima, continuous_maxima);
        }

        action_space =
            create_space(discrete_count, continuous_minima, continuous_maxima);

    } else {

        py::list action_spaces;

        for (auto &a : ext_ctrl_vec_) {
            py::list discrete_count;
            py::list continuous_minima;
            py::list continuous_maxima;

            a->set_environment();
            to_discrete(a->action_space.discrete_count, discrete_count);
            to_continuous(a->action_space.continuous_extrema, continuous_minima, continuous_maxima);

            auto space =
                create_space(discrete_count, continuous_minima, continuous_maxima);
            action_spaces.append(space);
        }

        py::object tuple_space = get_gym_space("Tuple");
        action_space = tuple_space(action_spaces);
    }
}

void ScrimmageOpenAIEnv::create_observation_space() {

    py::object tuple_space = get_gym_space("Tuple");

    auto create_obs = [&](py::list &discrete_count, py::list &continuous_maxima) -> py::object {

        int len_discrete = py::len(discrete_count);
        int len_continuous = py::len(continuous_maxima);

        py::array_t<int> discrete_array(len_discrete);
        py::array_t<double> continuous_array(len_continuous);

        if (len_discrete > 0 && len_continuous > 0) {
            py::list obs;
            obs.append(discrete_array);
            obs.append(continuous_array);
            return obs;
        } else if (len_continuous > 0) {
            return continuous_array;
        } else {
            return discrete_array;
        }
    };

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        bool done = false;
        for (auto &v : ext_sensor_vec_) {
            if (done) break;
            for (auto &s : v) {
                s->set_observation_space();
                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                if (global_sensor_) {
                    done = true;
                    break;
                }
            }
        }

        observation_space =
            create_space(discrete_count, continuous_minima, continuous_maxima);

        observation = create_obs(discrete_count, continuous_maxima);

    } else {

        py::list observation_spaces;
        py::list obs;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                py::list discrete_count;
                py::list continuous_minima;
                py::list continuous_maxima;

                s->set_observation_space();
                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                auto space =
                    create_space(discrete_count, continuous_minima, continuous_maxima);
                observation_spaces.append(space);
                obs.append(create_obs(discrete_count, continuous_maxima));
            }
        }

        observation_space = tuple_space(observation_spaces);
        observation = obs;
    }
}

void ScrimmageOpenAIEnv::set_reward_range() {
    reward_range = pybind11::tuple(2);
    if (ext_ctrl_vec_.size() == 1) {
        reward_range[0] = ext_ctrl_vec_[0]->reward_range.first;
        reward_range[1] = ext_ctrl_vec_[0]->reward_range.second;
    } else {

        auto get_min_reward = [&](auto &a) {return a->reward_range.first;};
        auto get_max_reward = [&](auto &a) {return a->reward_range.second;};
        auto min_reward = ext_ctrl_vec_ | ba::transformed(get_min_reward);
        auto max_reward = ext_ctrl_vec_ | ba::transformed(get_max_reward);

        if (combine_actors_) {
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

pybind11::object ScrimmageOpenAIEnv::create_space(
        pybind11::list discrete_count,
        pybind11::list continuous_minima,
        pybind11::list continuous_maxima) {

    py::module np = py::module::import("numpy");
    py::object np_array = np.attr("array");
    py::object np_float32 = np.attr("float32");

    py::object gym_discrete_space = get_gym_space("Discrete");
    py::object gym_multidiscrete_space = get_gym_space("MultiDiscrete");
    py::object gym_box_space = get_gym_space("Box");
    py::object gym_tuple_space = get_gym_space("Tuple");

    py::object discrete_space = py::len(discrete_count) == 1 ?
        gym_discrete_space(discrete_count[0]) :
        gym_multidiscrete_space(discrete_count);

    py::object continuous_space = gym_box_space(
        np_array(continuous_minima),
        np_array(continuous_maxima),
        py::none(),
        np_float32);

    int len_discrete = py::len(discrete_count);
    int len_continuous = py::len(continuous_minima);
    if (len_discrete != 0 && len_continuous != 0) {
        return gym_tuple_space(discrete_space, continuous_space);
    } else if (len_discrete != 0) {
        return discrete_space;
    } else if (len_continuous != 0) {
        return continuous_space;
    } else {
        // TODO: error handling
        return pybind11::object();
    }
}

void ScrimmageOpenAIEnv::render(const std::string &/*mode*/) {
    warning_function_("render must be set in gym.make with enable_gui kwargs");
}

void ScrimmageOpenAIEnv::update_observation() {

    auto call_get_obs = [&](auto *data, uint32_t &beg_idx, auto sensor, int obs_size) {
        uint32_t end_idx = beg_idx + obs_size;
        if (end_idx != beg_idx) {
            sensor->get_observation(data, beg_idx, end_idx);
            beg_idx = end_idx;
        }
    };

    auto init_arrays = [&](py::object obs_space, py::object obs) {
        py::array_t<int> disc_obs;
        py::array_t<double> cont_obs;
        if (PyObject_IsInstance(obs_space.ptr(), tuple_space_.ptr())) {
            py::list obs_list = obs.cast<py::list>();
            disc_obs = obs_list[0].cast<py::array_t<int>>();
            cont_obs = obs_list[1].cast<py::array_t<double>>();
        } else if (PyObject_IsInstance(obs_space.ptr(), box_space_.ptr())) {
            cont_obs = obs.cast<py::array_t<double>>();
        } else {
            disc_obs = obs.cast<py::array_t<int>>();
        }

        return std::make_tuple(disc_obs, cont_obs);
    };

    py::array_t<int> disc_obs;
    py::array_t<double> cont_obs;
    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {

        std::tie(disc_obs, cont_obs) = init_arrays(observation_space, observation);
        uint32_t disc_beg_idx = 0;
        uint32_t cont_beg_idx = 0;
        int* r_disc = static_cast<int *>(disc_obs.request().ptr);
        double* r_cont = static_cast<double *>(cont_obs.request().ptr);

        bool done = false;
        for (auto &v : ext_sensor_vec_) {
            if (done) break;
            for (auto &s : v) {
                auto obs_space = s->observation_space;
                call_get_obs(r_disc, disc_beg_idx, s, obs_space.discrete_count.size());
                call_get_obs(r_cont, cont_beg_idx, s, obs_space.continuous_extrema.size());

                if (global_sensor_) {
                    done = true;
                    break;
                }
            }
        }

    } else {
        py::list observation_space_list =
            observation_space.attr("spaces").cast<py::list>();
        py::list observation_list = observation.cast<py::list>();

        for (size_t i = 0; i < ext_sensor_vec_.size(); i++) {
            py::object indiv_space = observation_space_list[i];
            py::object indiv_obs = observation_list[i];
            std::tie(disc_obs, cont_obs) = init_arrays(indiv_space, indiv_obs);
            uint32_t disc_beg_idx = 0;
            uint32_t cont_beg_idx = 0;
            int* r_disc = static_cast<int *>(disc_obs.request().ptr);
            double* r_cont = static_cast<double *>(cont_obs.request().ptr);

            for (auto &s : ext_sensor_vec_[i]) {
                auto obs_space = s->observation_space;
                call_get_obs(r_disc, disc_beg_idx, s, obs_space.discrete_count.size());
                call_get_obs(r_cont, cont_beg_idx, s, obs_space.continuous_extrema.size());
            }
        }
    }
}

py::object ScrimmageOpenAIEnv::get_gym_space(const std::string &type) {
    return py::module::import("gym").attr("spaces").attr(type.c_str());
}

bool ScrimmageOpenAIEnv::is_gym_instance(pybind11::object &obj, const std::string &type) {
    py::object cls = get_gym_space(type);
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
    update_observation();
    return observation;
}

void ScrimmageOpenAIEnv::scrimmage_memory_cleanup() {
    // temporary fix until scrimmage uses weak pointers
    ext_ctrl_vec_.clear();
    ext_sensor_vec_.clear();

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

        if (e->controller()) {
            e->controller()->set_parent(nullptr);
            e->controller()->set_time(std::make_shared<sc::Time>());
            e->controller()->set_pubsub(nullptr);
            auto ds = std::make_shared<sc::State>();
            e->controller()->set_desired_state(ds);
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
        e->controller() = nullptr;
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
    log_ = sc::preprocess_scrimmage(mp_, *simcontrol_);
    simcontrol_->start_overall_timer();
    simcontrol_->set_time(mp_->t0());
    if (enable_gui) {
        simcontrol_->pause(true);
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

    ext_ctrl_vec_.clear();
    ext_sensor_vec_.clear();
    for (auto &e : simcontrol_->ents()) {
        for (auto &a : e->autonomies()) {
            auto a_cast = std::dynamic_pointer_cast<sc::autonomy::ScrimmageOpenAIAutonomy>(a);
            if (a_cast) {
                ext_ctrl_vec_.push_back(a_cast);

                std::vector<ScrimmageOpenAISensor> vec;
                for (auto &s : a_cast->parent()->sensors()) {
                    auto s_cast =
                      std::dynamic_pointer_cast<sc::sensor::ScrimmageOpenAISensor>(s.second);
                    if (s_cast) {
                        vec.push_back(s_cast);
                    }
                }
                ext_sensor_vec_.push_back(vec);
            }
        }
    }
    create_observation_space();
    create_action_space();
    set_reward_range();
}

void ScrimmageOpenAIEnv::run_viewer() {
    py::print("running viewer");
    scrimmage::Viewer viewer;

    auto outgoing = simcontrol_->outgoing_interface();
    auto incoming = simcontrol_->incoming_interface();

    viewer.set_incoming_interface(outgoing);
    viewer.set_outgoing_interface(incoming);
    viewer.set_enable_network(false);
    viewer.init(mp_->attributes()["camera"], mp_->log_dir(), mp_->dt());
    viewer.run();
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

    distribute_action(action);

    delayed_task_.update(simcontrol_->t());
    bool done = !simcontrol_->run_single_step(loop_number_++) ||
        simcontrol_->end_condition_reached();
    while (!done && !delayed_task_.update(simcontrol_->t()).first) {
        done = !simcontrol_->run_single_step(loop_number_++) ||
        simcontrol_->end_condition_reached();
    }

    update_observation();

    py::float_ py_reward;
    py::bool_ py_done;
    py::dict py_info;
    std::tie(py_reward, py_done, py_info) = calc_reward();

    done |= py_done.cast<bool>();
    py_done = py::bool_(done);

    return py::make_tuple(observation, py_reward, py_done, py_info);
}

std::tuple<pybind11::float_, pybind11::bool_, pybind11::dict> ScrimmageOpenAIEnv::calc_reward() {

    py::dict info;
    py::list info_done;
    py::list info_reward;

    double t = simcontrol_->t();
    double dt = mp_->dt(); // TODO: rely on time pointer

    double reward = 0;
    bool done = false;

    for (auto &a : ext_ctrl_vec_) {
        double temp_reward;
        bool temp_done;
        std::tie(temp_done, temp_reward) = a->calc_reward(t, dt);

        reward += temp_reward;
        done |= temp_done;

        info_reward.append(temp_reward);
        info_done.append(temp_done);
    }

    info["reward"] = info_reward;
    info["done"] = info_done;

    return std::make_tuple(py::float_(reward), py::bool_(done), info);
}

void ScrimmageOpenAIEnv::distribute_action(pybind11::object action) {
    py::array_t<int> disc_actions;
    py::array_t<double> cont_actions;
    int* disc_action_data;
    double* cont_action_data;

    auto update_action_lists = [&](py::object space, py::object act) {
        disc_actions = py::list();
        cont_actions = py::list();
        if (PyObject_IsInstance(space.ptr(), tuple_space_.ptr())) {
            py::list action_list = space.cast<py::list>();
            disc_actions = asarray_(action_list[0]);
            cont_actions = asarray_(action_list[1]);
            disc_action_data = static_cast<int*>(disc_actions.request().ptr);
            cont_action_data = static_cast<double*>(cont_actions.request().ptr);
        } else if (PyObject_IsInstance(space.ptr(), box_space_.ptr())) {
            cont_actions = asarray_(act);
            cont_action_data = static_cast<double*>(cont_actions.request().ptr);
        } else {
            disc_actions = asarray_(act);
            disc_action_data = static_cast<int*>(disc_actions.request().ptr);
        }
    };

    auto put_action = [&](auto &a, int &disc_idx, int cont_idx) {
        if (a->action.discrete.size() != a->action_space.discrete_count.size()) {
            a->action.discrete.resize(a->action_space.discrete_count.size());
        }
        if (a->action.continuous.size() != a->action_space.continuous_extrema.size()) {
            a->action.continuous.resize(a->action_space.continuous_extrema.size());
        }
        for (size_t i = 0; i < a->action_space.discrete_count.size(); i++) {
            a->action.discrete[i] = disc_action_data[disc_idx++];
        }
        for (size_t i = 0; i < a->action_space.continuous_extrema.size(); i++) {
            a->action.continuous[i] = cont_action_data[cont_idx++];
        }
    };

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {

        update_action_lists(action_space, action);

        int disc_action_idx = 0;
        int cont_action_idx = 0;
        for (auto &a : ext_ctrl_vec_) {
            put_action(a, disc_action_idx, cont_action_idx);
        }

    } else {
        py::list action_space_list = action_space.attr("spaces").cast<py::list>();
        py::list action_list = action.cast<py::list>();
        for (size_t i = 0; i < ext_ctrl_vec_.size(); i++) {
            py::object indiv_action_space = action_space_list[i];
            py::object indiv_action = action_list[i];
            update_action_lists(indiv_action_space, indiv_action);
            int disc_action_idx = 0;
            int cont_action_idx = 0;
            put_action(ext_ctrl_vec_[i], disc_action_idx, cont_action_idx);
        }
    }
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
        .def_readwrite("action_space", &ScrimmageOpenAIEnv::action_space)
        .def_readwrite("observation_space", &ScrimmageOpenAIEnv::observation_space)
        .def_readwrite("observation", &ScrimmageOpenAIEnv::observation)
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
