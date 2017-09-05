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

#ifndef INCLUDE_SCRIMMAGE_NETWORK_SCRIMMAGESERVICEIMPL_H_
#define INCLUDE_SCRIMMAGE_NETWORK_SCRIMMAGESERVICEIMPL_H_

#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/proto/Scrimmage.grpc.pb.h>
#include <scrimmage/network/Interface.h>

#include <list>
#include <mutex> // NOLINT

#if ENABLE_GRPC

namespace scrimmage {

class ScrimmageServiceImpl final : public scrimmage_proto::ScrimmageService::Service {
 public:
    explicit ScrimmageServiceImpl(Interface *interface);

    grpc::Status SendFrame(grpc::ServerContext* context, const scrimmage_proto::Frame* frame,
                           scrimmage_proto::BlankReply* reply) override;

    grpc::Status SendUTMTerrain(grpc::ServerContext* context, const scrimmage_proto::UTMTerrain* terrain,
                                scrimmage_proto::BlankReply* reply) override;

    grpc::Status SendSimInfo(grpc::ServerContext* context, const scrimmage_proto::SimInfo* sim_info,
                             scrimmage_proto::BlankReply* reply) override;

    grpc::Status SendGUIMsg(grpc::ServerContext* context, const scrimmage_proto::GUIMsg* gui_msg,
                            scrimmage_proto::BlankReply* reply) override;

    grpc::Status SendContactVisual(grpc::ServerContext* context,
                                   const scrimmage_proto::ContactVisual* contact_visual,
                                   scrimmage_proto::BlankReply* reply) override;

    grpc::Status SendShapes(grpc::ServerContext* context,
                            const scrimmage_proto::Shapes* shape,
                            scrimmage_proto::BlankReply* reply) override;

 protected:
    Interface * interface_;
};
} // namespace scrimmage

#endif // ENABLE_GRPC

#endif // INCLUDE_SCRIMMAGE_NETWORK_SCRIMMAGESERVICEIMPL_H_
