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

#ifndef SENSOR_H_
#define SENSOR_H_

#include <map>
#include <memory>

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/fwd_decl.h>

namespace scrimmage {

class Sensor : public Plugin {
 public:
    virtual inline void init(std::map<std::string,std::string> &params)
    {return;}
    
    virtual std::string name() { return std::string("Sensor"); }
    virtual std::string type() { return std::string("Sensor"); }
    virtual scrimmage::MessageBasePtr sensor_msg(double t, bool &valid)
    { return nullptr; }
    
    template <class T=MessageBase>
        std::shared_ptr<T> sense(double t, bool &valid)
        {
            MessageBasePtr sensor_msg = this->sensor_msg(t, valid);
            if (!valid) return nullptr;
            auto msg_cast = std::dynamic_pointer_cast<T>(sensor_msg);
            if (msg_cast) {
                return msg_cast;
            } else {
                return nullptr;
            }
        }        
    
 protected: 
};

using SensorPtr = std::shared_ptr<Sensor>;
}  // namespace scrimmage

#endif // SENSOR
