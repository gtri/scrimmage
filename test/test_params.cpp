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

#include <gtest/gtest.h>

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/common/ParameterServer.h>

#include <memory>
#include <cmath>
#include <iostream>

namespace sc = scrimmage;

TEST(test_params, single_set_get) {
    std::shared_ptr<sc::ParameterServer> param_server
        = std::make_shared<sc::ParameterServer>();

    std::shared_ptr<sc::Plugin> one = std::make_shared<sc::Plugin>();
    one->set_param_server(param_server);

    std::shared_ptr<sc::Plugin> two = std::make_shared<sc::Plugin>();
    two->set_param_server(param_server);

    std::string param_name = "one_param";

    // Create a local variable and register a parameter
    double one_param = 0;
    EXPECT_TRUE(one->register_param<double>(param_name, one_param));

    // Set the param in a different plugin
    const double param_value = 5;
    EXPECT_TRUE(two->set_param(param_name, param_value));

    // Did the local variable change to the correct value?
    EXPECT_NEAR(one_param, param_value, 1e-1);

    // Unregister the parameter
    EXPECT_TRUE(one->unregister_param<double>(param_name));

    // Set the parameter from a different plugin
    EXPECT_TRUE(two->set_param(param_name, param_value+1));

    // The local variable shouldn't have changed this time
    EXPECT_NEAR(one_param, param_value, 1e-1);

    // close_plugin calls unregister_params()
    one->close_plugin(0);
    two->close_plugin(0);
}

TEST(test_params, multiple_sets) {
    std::shared_ptr<sc::ParameterServer> param_server
        = std::make_shared<sc::ParameterServer>();

    int num_plugins = 10;
    std::vector<std::shared_ptr<sc::Plugin>> plugins(
        num_plugins, std::make_shared<sc::Plugin>());

    std::string param_name = "my_param";
    double value = 0;
    std::vector<double> variables(num_plugins, value);
    for (int i = 0; i < num_plugins; i++) {
        plugins[i]->set_param_server(param_server);
        EXPECT_NEAR(variables[i], value, 1e-1);
        plugins[i]->register_param<double>(param_name, variables[i]);
    }

    // Change a param
    value = 10.0;
    EXPECT_TRUE(plugins.front()->set_param(param_name, value));

    // Have all the params updated?
    for (int i = 0; i < num_plugins; i++) {
        EXPECT_NEAR(variables[i], value, 1e-1);
    }
}
