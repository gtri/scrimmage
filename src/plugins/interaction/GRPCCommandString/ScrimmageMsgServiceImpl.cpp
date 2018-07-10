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

#include <scrimmage/plugins/interaction/GRPCCommandString/ScrimmageMsgServiceImpl.h>
#include <scrimmage/plugins/interaction/GRPCCommandString/GRPCCommandString.h>
#include <scrimmage/msgs/Command.grpc.pb.h>

#include <list>

namespace scrimmage {
ScrimmageMsgServiceImpl::ScrimmageMsgServiceImpl(
    std::shared_ptr<Plugin> plugin) :
    plugin_(std::dynamic_pointer_cast<interaction::GRPCCommandString>(plugin)) {
}

grpc::Status scrimmage::ScrimmageMsgServiceImpl::SendCommandString(
    grpc::ServerContext* context, const scrimmage_msgs::CommandString* cmd,
    scrimmage_msgs::CommandAck* reply) {
    plugin_->push_msg(*cmd);
    reply->set_success(1);
    return grpc::Status::OK;
}

} // namespace scrimmage
