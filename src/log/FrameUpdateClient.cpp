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

#if ENABLE_GRPC == 1
#include <grpc++/grpc++.h>
#include <scrimmage/proto/Scrimmage.grpc.pb.h>
#endif

#include <scrimmage/log/FrameUpdateClient.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/Quaternion.h>

#include <iostream>
#include <memory>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#if ENABLE_GRPC == 1
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
#endif

using std::cout;
using std::endl;

namespace scrimmage {

FrameUpdateClient::FrameUpdateClient(const std::string &ip, int port) :
    angles_to_gps_(0, Angles::Type::EUCLIDEAN, Angles::Type::GPS) {}

bool FrameUpdateClient::send_frame(scrimmage_proto::Frame &frame) {

    // Change from local x, y, z to GPS coordinates:
    for (int i = 0; i < frame.contact_size(); i++) {
        // frame.contacts_[frame.contact(i).id().id()] = proto_2_contact(proto_frame.contact(i));
        double x = frame.contact(i).state().position().x();
        double y = frame.contact(i).state().position().y();
        double z = frame.contact(i).state().position().z();
        double lat, lon, h;
        proj_->Reverse(x, y, z, lat, lon, h);
        frame.mutable_contact(i)->mutable_state()->mutable_position()->set_x(lon);
        frame.mutable_contact(i)->mutable_state()->mutable_position()->set_y(lat);
        frame.mutable_contact(i)->mutable_state()->mutable_position()->set_z(h);

        x = frame.contact(i).state().orientation().x();
        y = frame.contact(i).state().orientation().y();
        z = frame.contact(i).state().orientation().z();
        double w = frame.contact(i).state().orientation().w();

        // scrimmage::Quaternion q(Eigen::Vector3d(x,y,z), w);
        scrimmage::Quaternion q(w, x, y, z);
        double yaw = q.yaw();

        angles_to_gps_.set_angle(Angles::rad2deg(yaw));
        yaw = Angles::deg2rad(angles_to_gps_.angle());
        q = scrimmage::Quaternion(q.roll(), q.pitch(), yaw);

        frame.mutable_contact(i)->mutable_state()->mutable_orientation()->set_x(q.x());
        frame.mutable_contact(i)->mutable_state()->mutable_orientation()->set_y(q.y());
        frame.mutable_contact(i)->mutable_state()->mutable_orientation()->set_z(q.z());
        frame.mutable_contact(i)->mutable_state()->mutable_orientation()->set_w(q.w());

        frame.mutable_contact(i)->set_type(scrimmage_proto::AIRCRAFT);
    }

#if ENABLE_GRPC == 1
    scrimmage_proto::BlankReply reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    grpc::ClientContext context;

    // The actual RPC.
    grpc::Status status;
    status = stub_->SendFrame(&context, frame, &reply);

    // Act upon its status.
    if (status.ok()) {
        return true;
    } else {
        cout << "Error code: " << status.error_code() << endl;
        cout << status.error_message() << endl;
        return false;
    }
#endif

    return true;
}

void FrameUpdateClient::set_projection(const std::shared_ptr<GeographicLib::LocalCartesian> &proj) {
    proj_ = proj;
}
} // namespace scrimmage
