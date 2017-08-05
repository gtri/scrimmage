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
#ifndef INCLUDE_SCRIMMAGE_SENSOR_SENSOR_H_
#define INCLUDE_SCRIMMAGE_SENSOR_SENSOR_H_

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/fwd_decl.h>
#include <scrimmage/pubsub/Message.h>

#include <map>
#include <memory>
#include <string>

#include <boost/optional.hpp>

namespace scrimmage {

class Sensor : public Plugin {
 public:
    virtual inline void init(std::map<std::string, std::string> &params) {return;}

    virtual std::string name() { return std::string("Sensor"); }
    virtual std::string type() { return std::string("Sensor"); }

    virtual boost::optional<scrimmage::MessageBasePtr> sensor_msg(double t) = 0;

    template <class T = MessageBase,
              class = typename std::enable_if<std::is_same<T, MessageBase>::value, void>::type>
    boost::optional<MessageBasePtr> sense(double t) {
        return sensor_msg(t);
    }

    template <class T = MessageBase,
              class = typename std::enable_if<!std::is_same<T, MessageBase>::value, void>::type>
    boost::optional<std::shared_ptr<scrimmage::Message<T>>> sense(double t) {
        auto msg = sensor_msg(t);
        if (msg) {
            auto msg_cast = std::dynamic_pointer_cast<scrimmage::Message<T>>(*msg);
            return msg_cast ?
                boost::optional<std::shared_ptr<scrimmage::Message<T>>>(msg_cast) : boost::none;
        } else {
            return boost::none;
        }
    }
};

using SensorPtr = std::shared_ptr<Sensor>;
}  // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_SENSOR_SENSOR_H_
