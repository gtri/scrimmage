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

#ifndef INCLUDE_SCRIMMAGE_COMMON_GLOBALSERVICE_H_
#define INCLUDE_SCRIMMAGE_COMMON_GLOBALSERVICE_H_

#include <scrimmage/pubsub/Message.h>
#include <map>
#include <set>
#include <unordered_map>
#include <list>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <iostream>

namespace scrimmage {

using Service = std::function<bool (scrimmage::MessageBasePtr, scrimmage::MessageBasePtr&)>;

class GlobalService {
 public:
    GlobalService();
    ~GlobalService();

    /////////////////////////////
    // All service call functions
    std::unordered_map<std::string, Service> &services();

    bool call_service(MessageBasePtr req, MessageBasePtr &res, const std::string &service_name);

    bool call_service(MessageBasePtr &res, const std::string &service_name) {
        return call_service(std::make_shared<MessageBase>(), res, service_name);
    }

    template <class T = MessageBasePtr,
              class = typename std::enable_if<!std::is_same<T, MessageBasePtr>::value, void>::type>
    bool call_service(MessageBasePtr req, T &res, const std::string &service_name) {
        MessageBasePtr res_base;
        if (call_service(req, res_base, service_name)) {
            res = std::dynamic_pointer_cast<typename T::element_type>(res_base);
            if (res == nullptr) {
                std::cout << "could not cast for global service " << service_name.c_str() << std::endl;
                return false;
            } else {
                return true;
            }
        } else {
            return false;
        }
    }

    template <class T = MessageBasePtr,
              class = typename std::enable_if<!std::is_same<T, MessageBasePtr>::value, void>::type>
    bool call_service(T &res, const std::string &service_name) {
        return call_service(std::make_shared<MessageBase>(), res, service_name);
    }

 protected:
    std::unordered_map<std::string, Service> services_;
};
using GlobalServicePtr = std::shared_ptr<GlobalService>;
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_COMMON_GLOBALSERVICE_H_
