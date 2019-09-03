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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_SCRIMMAGEMSGSERVICEIMPL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_SCRIMMAGEMSGSERVICEIMPL_H_

#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/plugins/interaction/GRPCCommandString/GRPCCommandString.h>
#include <scrimmage/msgs/Command.grpc.pb.h>

#include <queue>
#include <memory>

namespace scrimmage {
class ScrimmageMsgServiceImpl final : public scrimmage_msgs::ScrimmageMsgService::Service {
 public:
    explicit ScrimmageMsgServiceImpl(std::shared_ptr<Plugin> plugin);
    grpc::Status SendCommandString(grpc::ServerContext* context,
                                   const scrimmage_msgs::CommandString* cmd,
                                   scrimmage_msgs::CommandAck* reply) override;
 protected:
    std::shared_ptr<interaction::GRPCCommandString> plugin_;
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRPCCOMMANDSTRING_SCRIMMAGEMSGSERVICEIMPL_H_
