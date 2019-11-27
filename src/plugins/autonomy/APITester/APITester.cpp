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

#include <scrimmage/plugins/autonomy/APITester/APITester.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::APITester,
                APITester_plugin)

namespace scrimmage {
namespace autonomy {

void APITester::write_my_test_values() {
    csv_.append(CSV::Pairs{
            {"my_test_bool", my_test_bool_},
            {"my_test_int", my_test_int_},
            {"my_test_float", my_test_float_},
            {"my_test_double", my_test_double_}
        });
}

void APITester::init(std::map<std::string, std::string> &params) {
    // Setup variableIO
    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

    // Get default variable values
    my_test_bool_ = sc::get<bool>("my_test_bool", params, my_test_bool_);
    my_test_int_ = sc::get<int>("my_test_int", params, my_test_int_);
    my_test_float_ = sc::get<float>("my_test_float", params, my_test_float_);
    my_test_double_ = sc::get<double>("my_test_double", params, my_test_double_);

    std::string csv_file_name = sc::get<std::string>("csv_file_name", params, "api_tester.csv");

    // Create the csv for saving the variable values
    if (!csv_.open_output(parent_->mp()->log_dir() + "/" + csv_file_name)) {
        std::cout << "APITest: Couldn't create output file" << endl;
    }
    csv_.set_column_headers("my_test_bool, my_test_int, my_test_float, my_test_double");
    write_my_test_values(); // Write initial values to csv

    // Register the test variables with the parameter server
    auto bool_cb = [&](const bool &my_test_bool) {
        this->write_my_test_values();
    };
    register_param<bool>("my_test_bool", my_test_bool_, bool_cb);

    auto int_cb = [&](const int &my_test_int) {
        this->write_my_test_values();
    };
    register_param<int>("my_test_int", my_test_int_, int_cb);

    auto float_cb = [&](const float &my_test_float) {
        this->write_my_test_values();
    };
    register_param<float>("my_test_float", my_test_float_, float_cb);

    auto double_cb = [&](const double &my_test_double) {
        this->write_my_test_values();
    };
    register_param<double>("my_test_double", my_test_double_, double_cb);
}

bool APITester::step_autonomy(double t, double dt) {
    return true;
}
} // namespace autonomy
} // namespace scrimmage
