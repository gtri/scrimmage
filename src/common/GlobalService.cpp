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
 * @author Joel dunham <joel.dunham@gtri.gatech.edu>
 * @date 2019/08/16
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/GlobalService.h>

#include <string>
#include <algorithm>

namespace scrimmage {

GlobalService::GlobalService() {
    // No-op
}

GlobalService::~GlobalService() {
    // Clean up the services
    services_.clear();
}

std::unordered_map<std::string, Service> &GlobalService::services() {return services_;}

bool GlobalService::call_service(scrimmage::MessageBasePtr req,
        scrimmage::MessageBasePtr &res, const std::string &service_name) {
    auto it = services_.find(service_name);
    if (it == services_.end()) {
        std::cout << "request for global service ("
            << service_name
            << ") that does not exist" << std::endl;
        std::cout << "Global services are: ";
        for (auto &kv : services_) {
            std::cout << kv.first << ", ";
        }
        std::cout << std::endl;
        return false;
    }

    Service &service = it->second;
    bool success = service(req, res);

    if (!success) {
        std::cout << "call to " << service_name << " failed" << std::endl;
        return false;
    } else {
        return true;
    }
}
}  // namespace scrimmage
