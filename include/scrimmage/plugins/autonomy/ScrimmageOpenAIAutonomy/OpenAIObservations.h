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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIOBSERVATIONS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIOBSERVATIONS_H_

#include <pybind11/pybind11.h>

#include <scrimmage/common/Visibility.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace scrimmage {

class Sensor;
using SensorPtr = std::shared_ptr<Sensor>;

namespace sensor {
class ScrimmageOpenAISensor;
}

namespace autonomy {

class DLL_PUBLIC OpenAIObservations {
 public:
    OpenAIObservations();

    void create_observation_space(size_t num_entities);
    pybind11::object update_observation(size_t num_entities);

    std::vector<std::vector<std::shared_ptr<sensor::ScrimmageOpenAISensor>>> &ext_sensor_vec() {return ext_sensor_vec_;}

    void add_sensors(const std::unordered_map<std::string, SensorPtr> &sensors);

    pybind11::object observation_space;
    pybind11::object observation;

    void set_global_sensor(bool global_sensor) {global_sensor_ = global_sensor;}
    bool get_global_sensor() const {return global_sensor_;}

    void set_combine_actors(bool combine_actors) {combine_actors_ = combine_actors;}
    bool get_combine_actors() const {return combine_actors_;}

 protected:
    std::vector<std::vector<std::shared_ptr<sensor::ScrimmageOpenAISensor>>> ext_sensor_vec_;

    pybind11::object tuple_space_;
    pybind11::object box_space_;

    bool combine_actors_ = false;
    bool global_sensor_ = false;
};
} // namespace autonomy
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_OPENAIOBSERVATIONS_H_
