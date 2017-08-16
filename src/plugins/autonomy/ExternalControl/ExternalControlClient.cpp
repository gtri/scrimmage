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
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControlClient.h>
#include <scrimmage/math/State.h>
#include <google/protobuf/empty.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

bool ExternalControlClient::send_environment(
        scrimmage_proto::Environment &env, scrimmage::StatePtr state) {

    google::protobuf::Empty reply;
    grpc::ClientContext context;
    grpc::Status status = stub_->SendEnvironment(&context, env, &reply);
    return status.ok();
}

boost::optional<scrimmage_proto::Action>
ExternalControlClient::send_action_result(
        scrimmage_proto::ActionResult &action_result) {
    sp::Action action;
    grpc::ClientContext context;
    grpc::Status status = stub_->SendActionResult(&context, action_result, &action);
    return status.ok() && !action.done() ?
        boost::optional<sp::Action>(action) : boost::none;
}
