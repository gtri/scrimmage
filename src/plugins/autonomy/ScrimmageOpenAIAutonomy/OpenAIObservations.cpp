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
#include <pybind11/numpy.h>

#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIUtils.h>
#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIObservations.h>

#include <boost/range/algorithm/transform.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/map.hpp>

namespace py = pybind11;
namespace br = boost::range;
namespace ba = boost::adaptors;

namespace scrimmage {
namespace autonomy {

OpenAIObservations::OpenAIObservations() :
    tuple_space_(get_gym_space("Tuple")),
    box_space_(get_gym_space("Box")) {}

pybind11::object OpenAIObservations::update_observation(size_t num_entities) {

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
    if (num_entities == 1 || combine_actors_) {

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

                if (num_entities > 1 && global_sensor_) {
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

    return observation;
}

void OpenAIObservations::create_observation_space(size_t num_entities) {

    auto create_obs = [&](py::list &discrete_count, py::list &continuous_maxima) -> py::object {

        int len_discrete = py::len(discrete_count);
        int len_continuous = py::len(continuous_maxima);

        py::array_t<int> discrete_array(len_discrete);
        py::array_t<double> continuous_array(len_continuous);

        // Return by move to fix this warning from clang: warning: local
        // variable 'obs' will be copied despite being returned by name
        // [-Wreturn-std-move]
        if (len_discrete > 0 && len_continuous > 0) {
            py::list obs;
            obs.append(discrete_array);
            obs.append(continuous_array);
            return std::move(obs);
        } else if (len_continuous > 0) {
            return std::move(continuous_array);
        } else {
            return std::move(discrete_array);
        }
    };

    if (num_entities == 1 || combine_actors_) {
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

                if (num_entities > 1 && global_sensor_) {
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

        py::object tuple_space = get_gym_space("Tuple");
        observation_space = tuple_space(observation_spaces);
        observation = obs;
    }
}

void OpenAIObservations::add_sensors(const std::unordered_map<std::string, SensorPtr> &sensors) {
    std::vector<std::shared_ptr<sensor::ScrimmageOpenAISensor>> vec;
    auto cast = [&](auto &p) {return std::dynamic_pointer_cast<sensor::ScrimmageOpenAISensor>(p);};
    auto good_ptr = [&](auto &p) {return cast(p) != nullptr;};
    br::transform(sensors | ba::map_values | ba::filtered(good_ptr), std::back_inserter(vec), cast);
    ext_sensor_vec_.push_back(vec);
}
} // namespace autonomy
} // namespace scrimmage
