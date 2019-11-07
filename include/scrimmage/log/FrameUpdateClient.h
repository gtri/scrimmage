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

#ifndef INCLUDE_SCRIMMAGE_LOG_FRAMEUPDATECLIENT_H_
#define INCLUDE_SCRIMMAGE_LOG_FRAMEUPDATECLIENT_H_

#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/fwd_decl.h>
#include <scrimmage/math/Angles.h>

#if ENABLE_GRPC == 1
#include <scrimmage/proto/Scrimmage.grpc.pb.h>
#endif

#include <memory>
#include <string>

namespace scrimmage {
class FrameUpdateClient {
 public:
    FrameUpdateClient(const std::string &ip, int port);
    bool send_frame(scrimmage_proto::Frame &frame);

    void set_projection(const std::shared_ptr<GeographicLib::LocalCartesian> &proj);
 private:
#if ENABLE_GRPC == 1
    std::unique_ptr<scrimmage_proto::ScrimmageService::Stub> stub_;
#endif
    std::shared_ptr<GeographicLib::LocalCartesian> proj_;
    Angles angles_to_gps_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_LOG_FRAMEUPDATECLIENT_H_
