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

#include <scrimmage/common/VariableIO.h>

#include <Eigen/Dense>

#include <map>
#include <string>

namespace scrimmage {

VariableIO::VariableIO() : input_(std::make_shared<Eigen::VectorXd>()),
                           output_(std::make_shared<Eigen::VectorXd>()) {
}

std::map<std::string, int> & VariableIO::output_variable_index() {
    return output_variable_index_;
}

std::map<std::string, int> & VariableIO::input_variable_index() {
    return input_variable_index_;
}

int VariableIO::add_input_variable(std::string &var) {
    // If the variable already exists, return its existing index
    declared_input_variables_.insert(var);
    auto it = input_variable_index_.find(var);
    if (it != input_variable_index_.end()) {
        return it->second;
    }

    // If it doesn't exist, add it to the map and resize input vector
    int idx = next_input_variable_index_;
    input_variable_index_[var] = idx;
    ++next_input_variable_index_;
    *input_ = Eigen::VectorXd::Zero(input_variable_index_.size());
    return idx;
}

int VariableIO::add_output_variable(std::string &var) {
    // If the variable already exists, return its existing index
    declared_output_variables_.insert(var);
    int idx;
    auto it = output_variable_index_.find(var);
    if (it != output_variable_index_.end()) {
        idx = it->second;
    } else {
        // If it doesn't exist, it will be ignored by the next vector
        idx = output_variable_index_.size();
        output_variable_index_[var] = idx;
    }
    return idx;
}

int VariableIO::declare(std::string var, Direction dir) {
    if (dir == VariableIO::Direction::In) {
        return add_input_variable(var);
    } else {
        return add_output_variable(var);
    }
}

double VariableIO::input(int i) {
    return (*input_)(i);
}

void VariableIO::output(int i, double x) {
    // The plugin writing to the output doesn't know if the index it is writing
    // to is within the bounds of the input of the next plugin. Connect is
    // called before output to assign the shared ptr output to point to the
    // shared pointer input of the next plugin.
    if (i < output_->size()) (*output_)(i) = x;
}

double VariableIO::output(int i) {
    return i < output_->size() ? (*output_)(i) : NAN;
}

void connect(VariableIO &output, VariableIO &input) {
    output.output_variable_index() = input.input_variable_index();
    output.output_ = input.input_;
}

bool VariableIO::exists(std::string var, Direction dir) {
    if (dir == VariableIO::Direction::In) {
        return (input_variable_index_.count(var) > 0);
    } else {
        return (output_variable_index_.count(var) > 0);
    }
}

std::set<std::string> VariableIO::declared_input_variables() {
    return declared_input_variables_;
}

std::set<std::string> VariableIO::declared_output_variables() {
    return declared_output_variables_;
}
} // namespace scrimmage
