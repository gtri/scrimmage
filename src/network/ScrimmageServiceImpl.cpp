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

#if ENABLE_GRPC

#include <scrimmage/network/ScrimmageServiceImpl.h>
#include <scrimmage/network/Interface.h>

scrimmage::ScrimmageServiceImpl::ScrimmageServiceImpl(scrimmage::Interface *interface) : interface_(interface) {}

grpc::Status scrimmage::ScrimmageServiceImpl::SendFrame(grpc::ServerContext *context, const scrimmage_proto::Frame *frame, scrimmage_proto::BlankReply *reply) {
    std::shared_ptr<scrimmage_proto::Frame> f = std::make_shared<scrimmage_proto::Frame>(*frame);
    interface_->push_frame(f);
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::SendUTMTerrain(grpc::ServerContext *context, const scrimmage_proto::UTMTerrain *terrain, scrimmage_proto::BlankReply *reply) {
    std::shared_ptr<scrimmage_proto::UTMTerrain> t = std::make_shared<scrimmage_proto::UTMTerrain>(*terrain);
    interface_->push_utm_terrain(t);
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::SendSimInfo(grpc::ServerContext *context, const scrimmage_proto::SimInfo *sim_info, scrimmage_proto::BlankReply *reply) {
    scrimmage_proto::SimInfo si = *sim_info;
    interface_->push_sim_info(si);
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::SendGUIMsg(grpc::ServerContext *context, const scrimmage_proto::GUIMsg *gui_msg, scrimmage_proto::BlankReply *reply) {
    scrimmage_proto::GUIMsg si = *gui_msg;
    interface_->push_gui_msg(si);
    if (si.shutting_down()) {
        exit_requested.set_value();
    }
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::SendContactVisual(grpc::ServerContext *context, const scrimmage_proto::ContactVisual *contact_visual, scrimmage_proto::BlankReply *reply) {
    std::shared_ptr<scrimmage_proto::ContactVisual> cv = std::make_shared<scrimmage_proto::ContactVisual>(*contact_visual);
    interface_->push_contact_visual(cv);
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::SendShapes(grpc::ServerContext *context, const scrimmage_proto::Shapes *shape, scrimmage_proto::BlankReply *reply) {
    scrimmage_proto::Shapes s = *shape;
    interface_->push_shapes(s);
    return grpc::Status::OK;
}

grpc::Status scrimmage::ScrimmageServiceImpl::Ready(grpc::ServerContext* context,
                                                    const google::protobuf::Empty* shape,
                                                    scrimmage_proto::BlankReply* reply) {
    return grpc::Status::OK;
}

#endif // ENABLE_GRPC
