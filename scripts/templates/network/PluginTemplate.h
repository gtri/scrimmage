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

#ifndef (>>>HEADER_GUARD<<<)
#define (>>>HEADER_GUARD<<<)

#include <scrimmage/pubsub/Network.h>

#include <map>
#include <string>

namespace sc = scrimmage;

namespace scrimmage {
namespace network {

class (>>>PLUGIN_NAME<<<) : public scrimmage::Network {
 public:
    (>>>PLUGIN_NAME<<<)();

    virtual bool init(std::map<std::string, std::string> &mission_params,
                      std::map<std::string, std::string> &plugin_params);
 protected:
    virtual bool is_reachable(const scrimmage::PluginPtr &pub_plugin,
                              const scrimmage::PluginPtr &sub_plugin);

    virtual bool is_successful_transmission(const scrimmage::PluginPtr &pub_plugin,
                                            const scrimmage::PluginPtr &sub_plugin);
 protected:
 private:
};
} // namespace network
} // namespace scrimmage
#endif // (>>>HEADER_GUARD<<<)
