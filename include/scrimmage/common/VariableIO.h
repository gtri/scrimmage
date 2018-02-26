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

#ifndef INCLUDE_SCRIMMAGE_COMMON_VARIABLEIO_H_
#define INCLUDE_SCRIMMAGE_COMMON_VARIABLEIO_H_

#include <Eigen/Dense>

#include <map>
#include <string>
#include <memory>

namespace scrimmage {
class VariableIO {
 public:
    enum class Direction {In = 0, Out};

    VariableIO();

    std::map<std::string, int> &output_variable_index();
    std::map<std::string, int> &input_variable_index();

    int add_input_variable(std::string &var);
    int add_output_variable(std::string &var);
    int declare(std::string var, Direction dir);

    double input(int i);
    void output(int i, double x);
    double output(int i);

    /*! \brief Connect two VariableIO objects where writing to the output of
     *  the first object will transfer the data to the input of the second
     *  object. */
    friend void connect(VariableIO &output, VariableIO &input);

 protected:
    int next_input_variable_index_ = 0;
    std::map<std::string, int> input_variable_index_;
    std::map<std::string, int> output_variable_index_;
    std::shared_ptr<Eigen::VectorXd> input_;
    std::shared_ptr<Eigen::VectorXd> output_;
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_VARIABLEIO_H_
