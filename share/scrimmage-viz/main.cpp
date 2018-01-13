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

namespace sc = scrimmage;

int main(int, char* []) {
    sc::InterfacePtr incoming_interface = std::make_shared<sc::Interface>();
    sc::InterfacePtr outgoing_interface = std::make_shared<sc::Interface>();

    sc::Viewer viewer;
    viewer.set_enable_network(true);
    viewer.set_incoming_interface(incoming_interface);
    viewer.set_outgoing_interface(outgoing_interface);
    viewer.init({}, "", 1.0e-6);
    viewer.run();

    return 0;
}
