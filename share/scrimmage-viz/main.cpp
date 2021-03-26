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

#include <scrimmage/viewer/Viewer.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/parse/MissionParse.h>

#include <map>

#include <boost/program_options.hpp>

namespace sc = scrimmage;
namespace po = boost::program_options;

void set_param(po::variables_map &vm,
               std::map<std::string, std::string> &params, std::string str) {
    if (vm.count(str)) {
        params[str] = vm[str].as<std::string>();
    }
}

int main(int argc, char* argv[]) {

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("local_ip,i", po::value<std::string>(), "The local IP address where this viewer will run.")
        ("local_port,p", po::value<std::string>(), "The local port where this viewer will listen.")
        ("remote_ip,r", po::value<std::string>(), "The remote IP address where SCRIMMAGE is running.")
        ("remote_port,o", po::value<std::string>(), "The remote port where SCRIMMAGE is running.")
        ("pos", po::value<std::string>(), "camera position")
        ("focal_point", po::value<std::string>(), "camera focal point");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    sc::InterfacePtr incoming_interface = std::make_shared<sc::Interface>();
    sc::InterfacePtr outgoing_interface = std::make_shared<sc::Interface>();

    // Get the network parameters from the command line parser
    std::map<std::string, std::string> camera_params;
    set_param(vm, camera_params, "local_ip");
    set_param(vm, camera_params, "local_port");
    set_param(vm, camera_params, "remote_ip");
    set_param(vm, camera_params, "remote_port");
    set_param(vm, camera_params, "pos");
    set_param(vm, camera_params, "focal_point");

    if (camera_params.count("pos") > 0 || camera_params.count("focal_point") > 0) {
        camera_params["mode"] = "FREE";
    }

    auto mp = std::make_shared<sc::MissionParse>();
    mp->set_dt(1.0e-6);
    mp->set_log_dir("");

    sc::Viewer viewer;
    viewer.set_enable_network(true);
    viewer.set_incoming_interface(incoming_interface);
    viewer.set_outgoing_interface(outgoing_interface);
    viewer.init(mp, camera_params);
    viewer.run();

    return 0;
}
