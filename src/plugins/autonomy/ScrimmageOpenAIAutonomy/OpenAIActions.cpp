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

#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIActions.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/OpenAIUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>

#include <iostream>

namespace py = pybind11;

namespace scrimmage {
namespace autonomy {

OpenAIActions::OpenAIActions() :
    tuple_space_(get_gym_space("Tuple")),
    box_space_(get_gym_space("Box")),
    asarray_(py::module::import("numpy").attr("asarray")) {}

std::vector<std::shared_ptr<ScrimmageOpenAIAutonomy>> &OpenAIActions::ext_ctrl_vec() {
    return ext_ctrl_vec_;
}

void OpenAIActions::create_action_space(bool combine_actors) {

    if (ext_ctrl_vec_.empty()) {
        std::cout << "OpenAIActions::create_action_space called "
            << "with 0 actors" << std::endl;
    } else if (ext_ctrl_vec_.size() == 1 || combine_actors) {
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

void OpenAIActions::distribute_action(pybind11::object action, bool combine_actors) {
    py::array_t<int> disc_actions;
    py::array_t<double> cont_actions;
    int* disc_action_data = nullptr;
    double* cont_action_data = nullptr;

    auto update_action_lists = [&](py::object space, py::object act) {
        disc_actions = py::list();
        cont_actions = py::list();
        if (PyObject_IsInstance(space.ptr(), tuple_space_.ptr())) {
            py::tuple action_list = act.cast<py::list>();
            disc_actions = asarray_(action_list[0], py::str("int"));
            cont_actions = asarray_(action_list[1], py::str("float"));
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

    auto put_action = [&](int &idx, size_t from_sz, auto *from_data, auto &to_vec) {
        if (to_vec.size() != from_sz) {
            to_vec.resize(from_sz);
        }
        if (from_data == nullptr && from_sz > 0) {
            std::cout << "Error: disc_action_data not set correctly" << std::endl;
            return;
        } else {
            for (size_t i = 0; i < from_sz; i++) {
                to_vec[i] = from_data[idx++];
            }
        }
    };

    auto put_actions = [&](auto a, int &disc_action_idx, int &cont_action_idx) {
        put_action(disc_action_idx, a->action_space.discrete_count.size(),
                   disc_action_data, a->action.discrete);
        put_action(cont_action_idx, a->action_space.continuous_extrema.size(),
                   cont_action_data, a->action.continuous);
    };

    if (ext_ctrl_vec_.size() == 1 || combine_actors) {

        update_action_lists(action_space, action);

        int disc_action_idx = 0;
        int cont_action_idx = 0;
        for (auto &a : ext_ctrl_vec_) {
            put_actions(a, disc_action_idx, cont_action_idx);
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
            put_actions(ext_ctrl_vec_[i], disc_action_idx, cont_action_idx);
        }
    }
}


} // namespace autonomy
} // namespace scrimmage
