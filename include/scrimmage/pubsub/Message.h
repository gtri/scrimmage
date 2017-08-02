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

#ifndef _SCRIMMAGE_MESSAGE_H_
#define _SCRIMMAGE_MESSAGE_H_

#include <scrimmage/pubsub/MessageBase.h>

namespace scrimmage {

class MessageBase; 

template <class T>
class Message : public MessageBase {
 public:
    Message() : MessageBase() {}
#if ENABLE_PYTHON_BINDINGS==0
    Message(T _data, int _sender=undefined_id, std::string _serialized_data="") :
        MessageBase(_sender, _serialized_data), data(_data) {}
#else 
    Message(T _data, int _sender=undefined_id, std::string _serialized_data="", pybind11::object _py_data=pybind11::none()) :
        MessageBase(_sender, _serialized_data, _py_data), data(_data) {}
#endif 
    T data;

};

} // namespace scrimmage
#endif
