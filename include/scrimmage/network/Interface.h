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

#ifndef INCLUDE_SCRIMMAGE_NETWORK_INTERFACE_H_
#define INCLUDE_SCRIMMAGE_NETWORK_INTERFACE_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/network/ScrimmageServiceImpl.h>
#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/proto/GUIControl.pb.h>
#include <scrimmage/proto/Shape.pb.h>

#if ENABLE_GRPC == 1
#include <scrimmage/proto/Scrimmage.grpc.pb.h>
#include <grpc++/grpc++.h>
#endif

#include <list>
#include <mutex> // NOLINT
#include <string>
#include <memory>

namespace scrimmage {
class Interface {
 public:
    typedef enum Mode {
        shared = 0,
        client = 1,
        server = 2
    } Mode_t;

    void set_mode(Mode_t mode) { mode_ = mode; }
    void set_ip(std::string &ip) { ip_ = ip; }
    void set_port(int port) { port_ = port; }
    bool init_network(Mode_t mode, const std::string &ip, int port);

    bool frames_update(double t);
    bool utm_terrain_update();
    bool contact_visual_update();
    bool gui_msg_update();
    bool world_point_clicked_msg_update();
    bool sim_info_update();
    bool shapes_update();

    // SimControl to GUI
    bool send_frame(double time, ContactMapPtr &contacts);

    bool send_frame(std::shared_ptr<scrimmage_proto::Frame> &frame);

    bool send_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain);
    bool send_contact_visual(std::shared_ptr<scrimmage_proto::ContactVisual> &cv);

    bool push_frame(std::shared_ptr<scrimmage_proto::Frame> &frame);
    bool push_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm_terrain);
    bool push_contact_visual(std::shared_ptr<scrimmage_proto::ContactVisual> &cv);

    // GUI to SimControl
    bool send_gui_msg(scrimmage_proto::GUIMsg &gui_msg);
    bool push_gui_msg(scrimmage_proto::GUIMsg &gui_msg);
    bool send_world_point_clicked_msg(scrimmage_proto::WorldPointClicked &msg);
    bool push_world_point_clicked_msg(scrimmage_proto::WorldPointClicked &msg);

    bool send_sim_info(scrimmage_proto::SimInfo &sim_info);
    bool push_sim_info(scrimmage_proto::SimInfo &sim_info);

    bool send_shapes(scrimmage_proto::Shapes &shapes);
    bool push_shapes(scrimmage_proto::Shapes &shapes);

    std::mutex frames_mutex;
    std::mutex utm_terrain_mutex;
    std::mutex gui_msg_mutex;
    std::mutex world_point_clicked_msg_mutex;
    std::mutex sim_info_mutex;
    std::mutex shapes_mutex;
    std::mutex contact_visual_mutex;

    std::list<std::shared_ptr<scrimmage_proto::Frame> > & frames()
    { return frames_list_; }

    std::list<std::shared_ptr<scrimmage_proto::UTMTerrain> > & utm_terrain()
    { return utm_terrain_list_; }

    std::list<std::shared_ptr<scrimmage_proto::ContactVisual> > & contact_visual()
    { return contact_visual_list_; }

    std::list<scrimmage_proto::GUIMsg> & gui_msg()
    { return gui_msg_list_; }

    std::list<scrimmage_proto::WorldPointClicked> & world_point_clicked_msg()
    { return world_point_clicked_msg_list_; }

    std::list<scrimmage_proto::SimInfo> & sim_info()
    { return sim_info_list_; }

    std::list<scrimmage_proto::Shapes> & shapes()
    { return shapes_list_; }

    void send_cached();
    bool check_ready();

 protected:
    void start_server();

    Mode_t mode_ = shared;
    int port_ = 50051;
    std::string ip_;

    std::list<std::shared_ptr<scrimmage_proto::Frame> > frames_list_;
    std::list<std::shared_ptr<scrimmage_proto::UTMTerrain> > utm_terrain_list_;
    std::list<std::shared_ptr<scrimmage_proto::ContactVisual> > contact_visual_list_;

    std::list<scrimmage_proto::GUIMsg> gui_msg_list_;
    std::list<scrimmage_proto::WorldPointClicked> world_point_clicked_msg_list_;
    std::list<scrimmage_proto::SimInfo> sim_info_list_;
    std::list<scrimmage_proto::Shapes> shapes_list_;

    unsigned int max_queue_size_ = 100;

#if ENABLE_GRPC
    std::unique_ptr<scrimmage_proto::ScrimmageService::Stub> scrimmage_stub_;
    std::unique_ptr<grpc::Server> server_;
#endif

    // Connection timeout in seconds
    unsigned int client_timeout_ = 1;

    // Cached variables that the gui can request:
    bool caching_enabled_ = true;
    std::shared_ptr<scrimmage_proto::UTMTerrain> utm_terrain_cache_;
    std::list<std::shared_ptr<scrimmage_proto::ContactVisual> > contact_visual_cache_;
};
using InterfacePtr = std::shared_ptr<Interface>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_NETWORK_INTERFACE_H_
